file(GLOB_RECURSE srcs
        *.cpp
        ble/*.c
        ui/*.c
        ui/components/*.c
        ui/fonts/*.c
        ui/images/*.c
        ui/screens/*.c
)

idf_component_register(
        SRCS ${srcs}
        PRIV_REQUIRES
                "nvs_flash"
                # "spi_flash"
                "esp_driver_gpio"
                "esp_driver_uart"
                "esp_lcd_sh8601"
                "esp_adc"
                "esp_timer"
                "user_encoder_bsp"
                "bt"
                "i2c_bsp"
                "SensorLib"
                "lcd_bl_pwm_bsp"
                "lcd_touch_bsp"
                "lvgl"
                # "sdcard_bsp"
                # "fatfs"
                # "sdmmc"
                # "spiffs"
                # "esp_wifi_bsp"
                # "audio_bsp"
        INCLUDE_DIRS
                "."
                "ui"
                "ble"
)

# spiffs_create_partition_image(storage ../spiffs_partition FLASH_IN_PROJECT)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-deprecated-enum-enum-conversion)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-but-set-variable)
