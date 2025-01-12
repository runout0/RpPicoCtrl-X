cmake_minimum_required(VERSION 3.12)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(RpPicoCtrl-X01 C CXX ASM)
project(verfw VERSION 1.3.1)
include(PicoSDKConfig.cmake)
pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(ignoreMe "${WIFI_PASSWORD}${WIFI_SSID}${ELF2UF2_FOUND}${Pioasm_FOUND}")
add_definitions( -DPROJECT_VERSION_STRING="${CMAKE_PROJECT_VERSION}" )

add_executable(ctrl-x
        src/main.c
        src/uart/tt_uart.c
        src/i2c/RV3028/RV3028.c
        C:/pico-sdk/src/rp2_common/hardware_i2c/i2c.c
        src/gpio/xgpio.c
        src/i2c/xi2c.c
        src/i2c/oled_sh1106/BitBang_I2C.c
        src/i2c/oled_sh1106/ss_oled.c
        src/i2c/ioexp/ioexp.c
        src/spi/xspi.c
        C:/pico-sdk/src/rp2_common/hardware_spi/spi.c
        src/spi/max31865/MAX31865.c
        src/adc/xadc.c
        C:/pico-sdk/src/rp2_common/hardware_adc/adc.c
        src/spi/mcp2515/mcp2515.c
        src/spi/sram/23LCV512.c
        src/gpio/xpwm.c
        src/i2c/eeprom/xeeprom.c
        src/plc/config.c
        src/plc/iec_std_FB.c
        src/plc/iec_std_lib.c
        src/plc/Resource1.c
        src/xplc.c
        src/plc/plc_ioconfig.c
        src/plc/PLCTest03_pico.c
        src/i2c/joy2/joy2.c
        readme.txt)

target_compile_options(ctrl-x PRIVATE -DCTRLX -Wall -Wvla -Wunused-variable -O0 -ggdb)
target_include_directories(ctrl-x PRIVATE C:/pico-sdk/src/rp2_common/hardware_i2c/include C:/pico-sdk/src/rp2_common/hardware_spi/include C:/pico-sdk/src/rp2_common/hardware_adc/include c:/pico-sdk/src/rp2_common/hardware_pwm/include)

# pull in common dependencies
target_link_libraries(ctrl-x pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(ctrl-x)

# add url via pico_set_program_url
# example_auto_set_url(blink)
