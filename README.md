# pico-baro-usb
Reading Pressure Sensor DPS310 from Raspberry Pi Pico (C Language)

## Keywords
Barometric Sensor, Temperature Sensor, DPS310  
Raspberry Pi Pico RP2040

## Hardware
Whilst you could physically connect the Pico and the Baro-/Temp-Sensor in quite many ways, there is a "solder free" option by using (buying) this hardware from [Seeed Studio](https://www.seeedstudio.com):
 - [DPS310 Ready-to-use-PCB](https://wiki.seeedstudio.com/Grove-High-Precision-Barometric-Pressure-Sensor-DPS310)
 - [Grove Shield for Pi Pico](https://wiki.seeedstudio.com/Grove-Starter-Kit-for-Raspberry-Pi-Pico)

## Preconditions for Building
You should have the "**Raspberry Pi Pico SDK**" installed on your computer. Installation and "First Steps" are well described in the official Documentation:  
[Getting Started with Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)  
The instructions below are assuming cross-compilation/build on a "Linux" computer (i.e. a "Linux Host"), but instructions for Windows are as well contained in that document above.

## Download Instructions
Usually, during SDK Installation you'll have created a folder named `pico`. Therein, you can now download the source code using [git](https://en.wikipedia.org/wiki/Git):  
`git clone https://github.com/dieselman-de/pico-baro-usb.git`  

This method will automatically create a new sub-folder `pico-baro-usb` and download all the files from repository  (i.e. including License-File and this README)

Alternatively (e.g. if you don't want to use `git clone`) you can create a folder on your own and download at least these two files:
 - **baro-usb.c**
 - **CMakeLists.txt**

## Preparing for Build
As described in chapter 8 of "[Getting Started with Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)", you'll have to copy the `pico_sdk_import.cmake` file into the very same project-folder, where the two files above reside. If you installed the SDK and this project's code in the usual way, this can be accomplished like this:  
`cp ../pico-sdk/external/pico_sdk_import.cmake .`

## Creating Makefile with cmake
As likely well known from a number of similar projects, create a `build` sub-folder, dive into it an run `cmake` once similar to this example:  
`mkdir build`  
`cd build`  
**`cmake -DPICO_SDK_PATH=../../pico-sdk -DPICO_BOARD=pico_w ..`**

Some words on the parameters used here for cmake:
 - the parameter `PICO_SDK_PATH`is simply pointing to that folder, in which you've installed your "Raspberry Pi Pico SDK"
 - the parameter `PICO_BOARD` needs to match your Pico-Hardware. The example `pico_w` above is targeting an RP2040 with "WiFi" on Hardware (albeit WiFi is not necessary for this project)

And don't forget the closing two `..` dots in order to tell cmake, that the `CMakeLists.txt`-file can be found in folder above :smirk:

Of course, instead of handing the parameters to cmake with `-D` you can as well use "Environment Variables" (e.g. `export PICO_SDK_PATH=/home/myhome/pico/pico-sdk`)

## Building
Within the `build`subfolder of this project run  
`make`
