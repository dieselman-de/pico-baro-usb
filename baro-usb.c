/**
 * Copyright (c) 2025 Thomas Ulrich, DL1IAW
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

 /* Code to talk to a DPS310 temperature and pressure sensor

    References to 'DPS310 Datasheet' are referring to this version:
    Edition 2019-07-11

    This code is based on a Raspberry Pi example for another I2Csensor: BMP280
    The example code is available at
    https://github.com/raspberrypi/pico-examples/tree/master/i2c/bmp280_i2c
    The example code has been published under BSD-3-Clause license
    by Raspberry Pi (Trading) Ltd. -  so I decided to apply the same
    license, hoping that this is a friendly approach :-)

    DPS310 is driven at 3.3v NOT 5v.
    According to BMP280 example, the Pico GPIO (and therefore I2C)
    does not support 5v. Thus, 3.3V seems a natural choice

    Pin mappring has been modified to use Connector "I2C0" on a
    seeedstudio "Grove Shield", thus according to schematic at
    https://files.seeedstudio.com/wiki/Grove_Shield_for_Pi_Pico_V1.0/Grove_shield_for_PI_PICOv1.0SCH.pdf

    I2C_SDA_PIN = GP8
    I2C_SCL_PIN = GP9
    3.3v (pin 36) -> VCC on DPS310 board
    GND (pin 38)  -> GND on DPS310 board
 */


//
// Defines on "Pico Side", i.e. I2C Master
//
#define MEASUREMENT_INTERVAL_MILLISECS 10000
// Selection of I2C Hardware Block, either i2c0 or i2c1
#define I2C_HW_BLOCK i2c0
// SDA Options with i2c0: 0, 4,  8, 12, 16, 20, 24, 28
// SDA Options with i2c1: 2, 6, 10, 14, 18, 22, 26
#define LOCAL_I2C_SDA_PIN 8
// SCL is always one higher than SDA
#define LOCAL_I2C_SCL_PIN 9


//
// Defines on "DPS310 Side", i.e. I2C Client
//
// DPS310 device has default bus address of 0x77
#define DPS310_BUS_ADDR _u(0x77)
// We need to know the source of calibration coefficients for temperature
#define TMP_COEF_SRCE_ASIC 0
#define TMP_COEF_SRCE_MEMS 1
// DPS310 Registers
// corresponding to DPS310 Datasheet Table 15 "Register Map"
// Meaurement Data Registers - they have 3 Bytes each
#define PRS_DATA _u(0x00)
#define TMP_DATA _u(0x03)
#define DATA_SIZE 3
// Single Byte Registers
#define PRS_CFG _u(0x06)
#define TMP_CFG _u(0x07)
#define MEAS_CFG _u(0x08)
#define CFG_REG _u(0x09)
#define RESET _u(0x0C)
#define PRODUCT_ID _u(0x0D)
#define COEF_SRCE _u(0x28)
// the "calibration coefficients" have a special structure, so we need Start and Size
#define COEF_START _u(0x10)
#define COEF_SIZE 18

typedef struct {
    // According to section 8.10 of DPS310 Datasheet, both IDs are 4bit only
    // We nevertheless use a uint16 here
    uint16_t rev_id;
    uint16_t prod_id;
} dps310_id_t ;

typedef struct {
    // According to section 8.11 of DPS310 Datasheet.
    //
    // In Registers, Cmpensation Coefficients have different size in bits
    // e.g. coefficient c00 is the largest one and has a lenght of 20bits
    //
    // Coefficients can be negative, depending on the most significant bit
    //
    // We commonly use a signed 32bit integer here

    // The "meaning" of coefficients is taken over from
    // https://github.com/ruuvi/ruuvi.dps310.c/blob/master/src/dps310.h
    int32_t c0;     // Temperature offset.
    int32_t c1;     // Temperature 1st degree.
    int32_t c00;    // Pressure offset.
    int32_t c10;    // Pressure 1st degree rel to pressure.
    int32_t c01;    // Pressure 1st degree rel to temperature.
    int32_t c11;    // Pressure 1st degree rel to temperature + pressure.
    int32_t c20;    // Pressure 2nd degree rel to pressure.
    int32_t c21;    // Pressure 2nd degree rel to pressure + 1st to temperature.
    int32_t c30;    // Pressure 3rd degree rel to pressure.
} dps310_coefficients_t ;


// ************************************  HELPER FUNCTIONS  ****************************************
int32_t get_two_complement_of(uint32_t value, uint8_t length) {
    // Using '1<<x' instead of '2^x' or 'pow(2,x)'
    int32_t ret = value;
    int32_t maxvalue = (1 << (length-1)) - 1;

    if (value > maxvalue)
        ret = value - (1 << length);

    return ret;
}

// ************************************  DPS310 FUNCTIONS  ****************************************
void dps310_read_ids(dps310_id_t* identifiers) {
    // Buffer for reading: Single Byte is sufficient here
    uint8_t readbuf;
    // Address of Register with Product IDs
    uint8_t reg = PRODUCT_ID;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &reg, 1, true);      // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &readbuf, 1, false);  // false: finished with I2C bus

    // Conversion of the single, read Byte to uint16
    uint16_t productid = readbuf & 0x0F;
    uint16_t revisionid = readbuf & 0xF0;
    revisionid = revisionid >> 4;
    identifiers->prod_id = productid;
    identifiers->rev_id = revisionid;
}

void dps310_read_pressure_config(uint8_t* regbyte) {
    // Buffer for reading: Single Byte is sufficient here
    uint8_t readbuf;
    // Address of Register with Pressure Measurement Configuration
    // see section 8.3 of DPS310 Datasheet
    uint8_t reg = PRS_CFG;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &reg, 1, true);      // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &readbuf, 1, false);  // false: finished with I2C bus

    *regbyte = readbuf;
}

void dps310_write_pressure_config(uint8_t oversampling) {
    uint8_t registerbyte = 0;
    if (oversampling == 8)
        registerbyte = 0x03;
    else if (oversampling == 4)
        registerbyte = 0x02;
    else if (oversampling == 2)
        registerbyte = 0x01;
    else if (oversampling == 1)
        registerbyte = 0x00;
    else
        return;

    // Buffer for writing: Address and Value, each of size one Byte
    uint8_t writebuf[2];
    // Send a "Single Pressure Measurement Command" (0x01) to MEAS_CTRL in Register MEAS_CFG
    // see section 8.5 of DPS310 Datasheet
    // send register number followed by its corresponding value
    writebuf[0] = PRS_CFG;
    writebuf[1] = registerbyte;
    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, writebuf, 2, false); // false: finished with I2C bus
    
    sleep_ms(100);
}

void dps310_read_temperature_config(uint8_t* regbyte) {
    // Buffer for reading: Single Byte is sufficient here
    uint8_t readbuf;
    // Address of Register with Temperature Measurement Configuration
    // see section 8.4 of DPS310 Datasheet
    uint8_t reg = TMP_CFG;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &reg, 1, true);      // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &readbuf, 1, false);  // false: finished with I2C bus

    *regbyte = readbuf;
}

void dps310_write_temperature_config(uint8_t oversampling, uint8_t tmp_coefficient_source) {
    uint8_t registerbyte = 0;
    if (oversampling == 8)
        registerbyte = 0x03;
    else if (oversampling == 4)
        registerbyte = 0x02;
    else if (oversampling == 2)
        registerbyte = 0x01;
    else if (oversampling == 1)
        registerbyte = 0x00;
    else
        return;

    // Note that we silently skip this step if no valid "oversampling" is requested
    // ... thus nothingh is set, then
    if (tmp_coefficient_source == TMP_COEF_SRCE_ASIC)
        // Set Bit #7 to 0
        registerbyte = registerbyte & 0x7F;
    else if (tmp_coefficient_source == TMP_COEF_SRCE_MEMS)
        // Set Bit #7 to 1
        registerbyte = registerbyte | 0x80;
    else
        return;
        
    // Buffer for writing: Address and Value, each of size one Byte
    uint8_t writebuf[2];
    // Send a "Single Pressure Measurement Command" (0x01) to MEAS_CTRL in Register MEAS_CFG
    // see section 8.5 of DPS310 Datasheet
    // send register number followed by its corresponding value
    writebuf[0] = TMP_CFG;
    writebuf[1] = registerbyte;
    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, writebuf, 2, false); // false: finished with I2C bus
    
    sleep_ms(100);
}


void dps310_read_measurement_config(uint8_t* regbyte) {
    // Buffer for reading: Single Byte is sufficient here
    uint8_t readbuf;
    // Address of Register with Measurement Configuration (MEAS_CFG)
    // see section 8.5 of DPS310 Datasheet
    uint8_t reg = MEAS_CFG;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &reg, 1, true);      // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &readbuf, 1, false);  // false: finished with I2C bus

    *regbyte = readbuf;
}

void dps310_read_config_register(uint8_t* regbyte) {
    // Buffer for reading: Single Byte is sufficient here
    uint8_t readbuf;
    // Address of Register with Interrupt and FIFO Configuration (CFG_REG)
    // see section 8.6 of DPS310 Datasheet
    uint8_t reg = CFG_REG;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &reg, 1, true);      // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &readbuf, 1, false);  // false: finished with I2C bus

    *regbyte = readbuf;
}


void dps310_read_coefficient_source_register(uint8_t* regbyte) {
    // Buffer for reading: Single Byte is sufficient here
    uint8_t readbuf;
    // Address of Register with Coefficient Source (COEF_SRCE)
    // see section 8.12 of DPS310 Datasheet
    // Actually, there is only Bit 7 (0x80) of real interesr
    uint8_t reg = COEF_SRCE;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &reg, 1, true);      // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &readbuf, 1, false);  // false: finished with I2C bus

    *regbyte = readbuf;
}

void dps310_read_calibration_coefficients(dps310_coefficients_t* coefficients) {
    // Buffer for reading
    uint8_t readbuf[COEF_SIZE] = {0};
    // Address of Register
    uint8_t reg = COEF_START;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &reg, 1, true);             // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, readbuf, COEF_SIZE, false);  // false: finished with I2C bus

    for (int i=0;i<COEF_SIZE;i++)
        printf(" %02x", readbuf[i]);
    printf("\n");

    // Conversion of the read Bytes to Calibration Coefficients
    uint32_t value = 0;
    // *** Temperature Related Coefficients ***
    // c0 (12bit)
    value = readbuf[0] << 4;
    value = value + ((readbuf[1] & 0xF0) >> 4);
    printf("\n");
    printf("Coeff c0  / value inside:  %6d\n", value);
    printf("Coeff c0  / value hex:       %04x\n", value);
    coefficients->c0 = get_two_complement_of(value, 12);
    // c1 (12bit)
    value = (readbuf[1] & 0x0F) << 8;
    value = value + readbuf[2];
    printf("Coeff c1  / value inside:  %6d\n", value);
    printf("Coeff c1  / value hex:       %04x\n", value);
    coefficients->c1 = get_two_complement_of(value, 12);
    // *** Pressure Related Coefficients ***
    // c00 (20bit)
    value = readbuf[3] << 12;
    value = value + (readbuf[4] << 4);
    value = value + ((readbuf[5] & 0xF0) >> 4);
    printf("\n");
    printf("Coeff c00 / value inside:  %6d\n", value);
    printf("Coeff c00 / value hex:      %05x\n", value);
    coefficients->c00 = get_two_complement_of(value, 20);
    // c10 (20bit)
    value = (readbuf[5] & 0x0F) << 16;
    value = value + (readbuf[6] << 8);
    value = value + readbuf[7];
    printf("Coeff c10 / value inside:  %6d\n", value);
    printf("Coeff c10 / value hex:      %05x\n", value);
    coefficients->c10 = get_two_complement_of(value, 20);
    // c20 (16bit)
    value = readbuf[12] << 8;
    value = value + readbuf[13];
    printf("Coeff c20 / value inside:  %6d\n", value);
    printf("Coeff c20 / value hex:       %04x\n", value);
    coefficients->c20 = get_two_complement_of(value, 16);
    // c30 (16bit)
    value = readbuf[16] << 8;
    value = value + readbuf[17];
    printf("Coeff c30 / value inside:  %6d\n", value);
    printf("Coeff c30 / value hex:       %04x\n", value);
    coefficients->c30 = get_two_complement_of(value, 16);
    // *** Pressure Related Coefficients, depending on Temperature ***
    // c01 (16bit)
    value = readbuf[8] << 8;
    value = value + readbuf[9];
    printf("Coeff c01 / value inside:  %6d\n", value);
    printf("Coeff c01 / value hex:       %04x\n", value);
    coefficients->c01 = get_two_complement_of(value, 16);
    // c11 (16bit)
    value = readbuf[10] << 8;
    value = value + readbuf[11];
    printf("Coeff c11 / value inside:  %6d\n", value);
    printf("Coeff c11 / value hex:       %04x\n", value);
    coefficients->c11 = get_two_complement_of(value, 16);
    // c21 (16bit)
    value = readbuf[14] << 8;
    value = value + readbuf[15];
    printf("Coeff c21 / value inside:  %6d\n", value);
    printf("Coeff c21 / value hex:       %04x\n", value);
    coefficients->c21 = get_two_complement_of(value, 16);
}

void dps310_get_raw_pressure(int32_t* pressure_raw_value) {
    // Buffer for writing: Address and Value, each of size one Byte
    uint8_t writebuf[2];
    // Send a "Single Pressure Measurement Command" (0x01) to MEAS_CTRL in Register MEAS_CFG
    // see section 8.5 of DPS310 Datasheet
    // send register number followed by its corresponding value
    writebuf[0] = MEAS_CFG;
    writebuf[1] = 0x01;

    // Buffer for reading: Measurements have 3 Bytes (24bit)
    uint8_t readbuf[DATA_SIZE] = {0};
    // Address of Register with Temperature Measurement
    uint8_t data_reg = PRS_DATA;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, writebuf, 2, true);   // true: keep master of I2C bus
    // Wait for Measurement // TO DO: This should be calculated, based on "Oversampling"
    sleep_ms(500);
    // Get raw Pressure
    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &data_reg, 1, true);   // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, readbuf, DATA_SIZE, false);  // false: finished with I2C bus

    printf("\nRegisters PRS_DATA:\n");
    for (int i=0;i<DATA_SIZE;i++)
        printf(" %02x", readbuf[i]);
    printf("\n");

    // Mapping of MSB, LSB and xLSB
    uint32_t raw_measurement = 0;
    raw_measurement = raw_measurement + (readbuf[0] << 16);
    raw_measurement = raw_measurement + (readbuf[1] << 8);
    raw_measurement = raw_measurement + (readbuf[2]);
    *pressure_raw_value = get_two_complement_of(raw_measurement, 24);
}

void dps310_get_raw_temperature(int32_t* temperature_raw_value) {
    // Buffer for writing: Address and Value, each of size one Byte
    uint8_t writebuf[2];
    // Send a "Single Temperature Measurement Command" (0x02) to MEAS_CTRL in Register MEAS_CFG
    // see section 8.5 of DPS310 Datasheet
    // send register number followed by its corresponding value
    writebuf[0] = MEAS_CFG;
    writebuf[1] = 0x02;

    // Buffer for reading: Measurements have 3 Bytes (24bit)
    uint8_t readbuf[DATA_SIZE] = {0};
    // Address of Register with Temperature Measurement
    uint8_t data_reg = TMP_DATA;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, writebuf, 2, true);   // true: keep master of I2C bus
    // Wait for Measurement // TO DO: This should be calculated, based on "Oversampling"
    sleep_ms(500);
    // Get raw temperature
    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &data_reg, 1, true);   // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, readbuf, DATA_SIZE, false);  // false: finished with I2C bus

    printf("\nRegisters TMP_DATA:\n");
    for (int i=0;i<DATA_SIZE;i++)
        printf(" %02x", readbuf[i]);
    printf("\n");

    // Mapping of MSB, LSB and xLSB
    uint32_t raw_measurement = 0;
    raw_measurement = raw_measurement + (readbuf[0] << 16);
    raw_measurement = raw_measurement + (readbuf[1] << 8);
    raw_measurement = raw_measurement + (readbuf[2]);
    *temperature_raw_value = get_two_complement_of(raw_measurement, 24);
}

void dps310_read_registers_plain() {
    // Buffer for reading
    uint8_t readbuf[14] = {0};
    // Address of Register
    uint8_t reg = 0x00;

    i2c_write_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, &reg, 1, true);      // true: keep master of I2C bus
    i2c_read_blocking(I2C_HW_BLOCK, DPS310_BUS_ADDR, readbuf, 14, false);  // false: finished with I2C bus

    printf("\nRegisters 0x00 to 0x0D:\n");
    for (int i=0;i<14;i++)
        printf(" %02x", readbuf[i]);
    printf("\n");
}

// ******************************************  MAIN  **********************************************
int main() {
    // Waiting a bit before getting host-interface (USB/UART) up
    sleep_ms(2000);
    // Enable UART/USB so we can print status output
    stdio_init_all();
    // Waiting for UART/USV init to complete
    sleep_ms(2000);
    // Tell that we are 'alive'
    printf("\n*** Starting 'baro-usb' ***\n");

    // useful information for picotool
    bi_decl(bi_2pins_with_func( LOCAL_I2C_SDA_PIN,  LOCAL_I2C_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("DPS310 reading via I2C for the Raspberry Pi Pico"));

    // This example will use Connector I2C according to defines above
    i2c_init(I2C_HW_BLOCK, 100 * 1000);
    gpio_set_function(LOCAL_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LOCAL_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(LOCAL_I2C_SDA_PIN);
    gpio_pull_up(LOCAL_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(LOCAL_I2C_SDA_PIN, LOCAL_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Represents i2c0 as "0" i2c1 as "1"
    printf("\nI2C Interface (HW Block) in use: i2c%d\n", i2c_hw_index(I2C_HW_BLOCK));
    // Further Info
    printf("\nI2C PIN Assignment\n");
    printf("SDA PIN:     %2d\n", LOCAL_I2C_SDA_PIN);
    printf("SCL PIN:     %2d\n", LOCAL_I2C_SCL_PIN);

    // Read Product ID and Revision DPS310
    static dps310_id_t dps310_ids;
    dps310_read_ids(&dps310_ids);
    // Print these Values
    printf("\n");
    printf("Product ID:  %2d\n", dps310_ids.prod_id);
    printf("Revision ID: %2d\n", dps310_ids.rev_id);

    // Read the Pressure Measurement Configuration Register (PRS_CFG) see section 8.3
    uint8_t value_prs_cfg;
    dps310_read_pressure_config(&value_prs_cfg);
    // Read the Temperature Measurement Configuration Register (TMP_CFG) see section 8.4
    uint8_t value_tmp_cfg;
    dps310_read_temperature_config(&value_tmp_cfg);
    // Read the Measurement Configuration Register (MEAS_CFG) see section 8.5
    uint8_t value_meas_cfg;
    dps310_read_measurement_config(&value_meas_cfg);
    // Read the Configuration Register (CFG_REG) see section 8.6
    uint8_t value_cfg_reg;
    dps310_read_config_register(&value_cfg_reg);
    // Read the Coefficient Source Register (COEF_SRCE) see section 8.12
    uint8_t value_coef_srce;
    dps310_read_coefficient_source_register(&value_coef_srce);
    // Print these Values
    printf("\nInitial State of Registers:\n");
    printf("-- Pressure Meas. Config. (PRS_CFG):         %2x\n", value_prs_cfg);
    printf("-- Temperature Meas. Config. (TMP_CFG):      %2x\n", value_tmp_cfg);
    printf("-- Measurement Configuration (MEAS_CFG):     %2x\n", value_meas_cfg);
    printf("-- Configuration Register (CFG_REG):         %2x\n", value_cfg_reg);
    printf("-- Coefficient Source Register (COEF_SRCE):  %2x\n", value_coef_srce);

    // Check the source of the Temperature Calibration Coefficients
    uint8_t value_tmp_coef_srce;
    // In case of MEMS, the MSB (bit #7) is set, thus 0x80 is present
    if ((value_coef_srce & 0x80) == 0x80) {
        printf("Temperature Coefficients are based on External Sensor (MEMS Element)\n");
        value_tmp_coef_srce = TMP_COEF_SRCE_MEMS;
    }
    else {
        printf("Temperature Coefficients are based on Internal Sensor (ASIC)\n");
        value_tmp_coef_srce = TMP_COEF_SRCE_ASIC;
    }

    // Alter the Pressure Measurement Configuration Register (PRS_CFG)
    // to set "Oversampling" (Accepts Values from 1=single, 2, 4 and 8)
    dps310_write_pressure_config(8);
    // Re-Read the Pressure Measurement Configuration Register (PRS_CFG) see section 8.3
    dps310_read_pressure_config(&value_prs_cfg);
    // Alter the Temperature Measurement Configuration Register (TMP_CFG)
    // to set "Oversampling" (Accepts Values from 1=single, 2, 4 and 8)
    // and "Measurement Source", which we take from reading COEF_SRCE above
    dps310_write_temperature_config(4, value_tmp_coef_srce);
    // Re-Read the Temperature Measurement Configuration Register (TMP_CFG)
    dps310_read_temperature_config(&value_tmp_cfg);
    // Print these Values
    printf("\nReconfigured State of Registers:\n");
    printf("-- Pressure Meas. Config. (PRS_CFG) new:     %2x\n", value_prs_cfg);
    printf("-- Temperature Meas. Config. (TMP_CFG) new:  %2x\n", value_tmp_cfg);
    
    // Need to adapt Scaling Factors According to the Oversampling we have chosen
    // 
    // Scaling factor kT for Temperature
    // Based on "Oversampling" for Temperature = "4"
    // and then read in Table 9 from Datasheet (section 4.9.3)
    int32_t  sf_kT = 3670016;
    // Scaling factor kP for Pressure
    // Based on "Oversampling" for Pressure = "8"
    // and then read in Table 9 from Datasheet (section 4.9.3)
    int32_t  sf_kP = 7864320;

    // Read Calibration Coefficients
    static dps310_coefficients_t dps310_coefficients;
    dps310_read_calibration_coefficients(&dps310_coefficients);
    // Print these Values
    printf("\nCalibration Coefficients:\n");
    printf("Coeff c0:   %8d\n", dps310_coefficients.c0);
    printf("Coeff c1:   %8d\n", dps310_coefficients.c1);
    printf("Coeff c00:  %8d\n", dps310_coefficients.c00);
    printf("Coeff c10:  %8d\n", dps310_coefficients.c10);
    printf("Coeff c20:  %8d\n", dps310_coefficients.c20);
    printf("Coeff c30:  %8d\n", dps310_coefficients.c30);
    printf("Coeff c01:  %8d\n", dps310_coefficients.c01);
    printf("Coeff c11:  %8d\n", dps310_coefficients.c11);
    printf("Coeff c21:  %8d\n", dps310_coefficients.c21);

    dps310_read_registers_plain();

    // Preparing for "the Loop"
    // raw Measurement Registers have 24bit ... we use uint32
    bool     keeprunning = true;
    int32_t  raw_temperature;
    int32_t  raw_pressure;

    sleep_ms(500); // sleep so that data polling and register update don't collide
    
    /// *** The Loop ***
    while (keeprunning) {
        // *** Reading Temperature ***
        dps310_get_raw_temperature(&raw_temperature);
        // Print raw Temperature
        printf("\n");
        printf("Raw Temperature (24bit):  %8d\n", raw_temperature);
        // *** Temperature Calculation acc. to section 4.9.2 from Datasheet
        float t_raw_scaled = ((float) raw_temperature) / ((float) sf_kT);
        printf("Temperature raw,scaled (float):  %8f\n", t_raw_scaled);
        float t_comp = ((float)dps310_coefficients.c0 * 0.5f) + ((float)dps310_coefficients.c1 * t_raw_scaled);
        printf("Temperature °C compensated (float):  %8f\n", t_comp);
        
        sleep_ms(500);

        // *** Reading Pressure ***
        dps310_get_raw_pressure(&raw_pressure);
        // Print raw Pressure
        printf("\n");
        printf("Raw Pressure (24bit):  %8d\n", raw_pressure);
        // *** Pressure Calculation acc. to section 4.9.1 from Datasheet
        float p_raw_scaled = ((float) raw_pressure) / ((float) sf_kP);
        printf("Pressure raw,scaled (float):  %8f\n", p_raw_scaled);
        float p_comp = (float)dps310_coefficients.c00 
                       + p_raw_scaled * ((float)dps310_coefficients.c10
                                         + p_raw_scaled * ((float)dps310_coefficients.c20
                                                           + (float)dps310_coefficients.c30 * p_raw_scaled))
                       + t_raw_scaled * (float)dps310_coefficients.c01
                       + t_raw_scaled * p_raw_scaled * ((float)dps310_coefficients.c11
                                                        + (float)dps310_coefficients.c21 * p_raw_scaled);
        printf("Pressure (Pa) compensated (float):  %8f\n", p_comp);
        printf("Pressure (hPa) ...thus QFE:         %4.2f\n", p_comp / 100.0f);
        
        sleep_ms(MEASUREMENT_INTERVAL_MILLISECS);
    }

    printf("Done.\n");
    return 0;
}
