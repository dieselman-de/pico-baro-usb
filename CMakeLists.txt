cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(baro-usb C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(baro-usb
    baro-usb.c
)

# We want to "Print" the Output to USB and/or UART
# You can choose to print to USB only, UART only or both
#
# Current Setting is "USB Only"
pico_enable_stdio_usb(baro-usb 1)
pico_enable_stdio_uart(baro-usb 0)

pico_add_extra_outputs(baro-usb)

# Pull in Libraries for Linking:
# > pico_stdlib :  Raspberry Pi Pico Standard Library
# > hardware_i2c : Required to talk to DPS310 via I2C Interface
target_link_libraries(baro-usb pico_stdlib hardware_i2c)
