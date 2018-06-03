################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/utils/Logger.cpp 

LINK_OBJ += \
./src/utils/Logger.cpp.o 

CPP_DEPS += \
./src/utils/Logger.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
src/utils/Logger.cpp.o: ../src/utils/Logger.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++" -DESP_PLATFORM '-DMBEDTLS_CONFIG_FILE="mbedtls/esp_config.h"' -DHAVE_CONFIG_H "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/config" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/bluedroid" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/app_trace" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/app_update" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/bootloader_support" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/bt" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/driver" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/esp32" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/esp_adc_cal" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/ethernet" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/fatfs" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/freertos" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/heap" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/jsmn" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/log" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/mdns" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/mbedtls" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/mbedtls_port" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/newlib" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/nvs_flash" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/openssl" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/spi_flash" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/sdmmc" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/spiffs" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/tcpip_adapter" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/ulp" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/vfs" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/wear_levelling" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/xtensa-debug-module" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/coap" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/console" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/expat" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/json" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/lwip" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/newlib" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/nghttp" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/soc" "-I/home/ziash/WorkSpaceEsp32/arduino-esp32/tools/sdk/include/wpa_supplicant" -std=gnu++11 -fno-exceptions -Os -g3 -Wpointer-arith -fexceptions -fstack-protector -ffunction-sections -fdata-sections -fstrict-volatile-bitfields -mlongcalls -nostdlib -Wall -Werror=all -Wextra -Wno-error=unused-function -Wno-error=unused-but-set-variable -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wno-unused-parameter -Wno-sign-compare -fno-rtti -c -DF_CPU=240000000L -DARDUINO=10802 -DARDUINO_LOLIN32 -DARDUINO_ARCH_ARDUINO-ESP32 '-DARDUINO_BOARD="LOLIN32"' '-DARDUINO_VARIANT="lolin32"' -DESP32 -DCORE_DEBUG_LEVEL=0  -I"/home/ziash/WorkSpaceEsp32/arduino-esp32/cores/esp32" -I"/home/ziash/WorkSpaceEsp32/PetoiEsp32/libraries/MPU6050" -I"/home/ziash/WorkSpaceEsp32/PetoiEsp32/libraries/I2Cdev" -I"/home/ziash/WorkSpaceEsp32/PetoiEsp32/libraries/Adafruit-PWM-Servo-Driver" -I"/home/ziash/WorkSpaceEsp32/PetoiEsp32/libraries" -I"/home/ziash/WorkSpaceEsp32/PetoiEsp32/src" -I"/home/ziash/WorkSpaceEsp32/arduino-esp32/variants/lolin32" -I"/home/ziash/WorkSpaceEsp32/arduino-esp32/libraries/EEPROM" -I"/home/ziash/WorkSpaceEsp32/arduino-esp32/libraries/Wire" -I"/home/ziash/WorkSpaceEsp32/arduino-esp32/libraries/Wire/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


