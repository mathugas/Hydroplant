# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

# compile C with /home/chamat/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc
C_DEFINES = -DESP_PLATFORM -DIDF_VER=\"v5.1-dev-1264-g1c84cfde14\" -DMBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\" -D_GNU_SOURCE -D_POSIX_READER_WRITER_LOCKS

C_INCLUDES = -I/home/chamat/Hydroplant/project/hydroplant/build/config -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_local_ctrl/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_local_ctrl/proto-c -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_local_ctrl/src -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/protocomm/proto-c -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/newlib/platform_include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/freertos/FreeRTOS-Kernel/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/freertos/esp_additions/include/freertos -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/freertos/FreeRTOS-Kernel/portable/xtensa/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/freertos/esp_additions/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_hw_support/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_hw_support/include/soc -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_hw_support/include/soc/esp32 -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_hw_support/port/esp32/. -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_hw_support/port/esp32/private_include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/heap/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/log/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/soc/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/soc/esp32/. -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/soc/esp32/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/hal/esp32/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/hal/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/hal/platform_port/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_rom/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_rom/include/esp32 -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_rom/esp32 -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_common/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_system/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_system/port/soc -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_system/port/include/private -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/xtensa/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/xtensa/esp32/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/lwip/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/lwip/include/apps -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/lwip/include/apps/sntp -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/lwip/lwip/src/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/lwip/port/esp32/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/lwip/port/esp32/include/arch -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/protocomm/include/common -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/protocomm/include/security -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/protocomm/include/transports -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_timer/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_wifi/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_event/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_phy/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_phy/esp32/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_netif/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp_http_server/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/http_parser -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp-tls -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/esp-tls/esp-tls-crypto -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/mbedtls/port/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/mbedtls/mbedtls/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/mbedtls/mbedtls/library -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/mbedtls/esp_crt_bundle/include -I/home/chamat/Hydroplant/esp_rtos/esp-idf/components/protobuf-c/protobuf-c

C_FLAGS = -mlongcalls -Wno-frame-address  -fdiagnostics-color=always -ffunction-sections -fdata-sections -Wall -Werror=all -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-enum-conversion -gdwarf-4 -ggdb -Og -fmacro-prefix-map=/home/chamat/Hydroplant/project/hydroplant=. -fmacro-prefix-map=/home/chamat/Hydroplant/esp_rtos/esp-idf=/IDF -fstrict-volatile-bitfields -Wno-error=unused-but-set-variable -fno-jump-tables -fno-tree-switch-conversion -DconfigENABLE_FREERTOS_DEBUG_OCDAWARE=1 -std=gnu17 -Wno-old-style-declaration -Wno-format

