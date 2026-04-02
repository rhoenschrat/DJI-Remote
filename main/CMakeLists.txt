set(SRCS "app_main.c"
         "ui.c"
         "ui_layout.c"
         "ui_screen_main.c"
         "ui_screen_pairing.c"
         "ui_screen_settings.c"
         "ui_screen_mode_switch.c"
         "icons.c"
         "lvgl_icons.c"
         "../utils/crc/custom_crc16.c"
         "../utils/crc/custom_crc32.c"
         "../protocol/dji_protocol_parser.c"
         "../protocol/dji_protocol_data_processor.c"
         "../protocol/dji_protocol_data_descriptors.c"
         "../protocol/dji_protocol_data_structures.c"
         "../ble/ble.c"
         "../data/data.c"
         "../logic/connect_logic.c"
         "../logic/command_logic.c"
         "../logic/status_logic.c"
         "../logic/enums_logic.c"
         "../logic/light_logic.c"
         "splash_logo.c"
         "ui_screen_splash.c"
         "../gps/gps_reader.c"
         "../log_config.c")

# Board-specific HAL source — only the active HAL is compiled
if(CONFIG_BOARD_M5STACK_BASIC_V27)
    list(APPEND SRCS "m5stack_basic_v27_hal.c")
elseif(CONFIG_BOARD_WAVESHARE_S3_LCD19)
    list(APPEND SRCS "waveshare_s3_lcd19_hal.c")
endif()

idf_component_register(SRCS ${SRCS}
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio led_strip esp_driver_spi esp_lcd driver esp_lvgl_port
                    INCLUDE_DIRS "." "../utils/crc" "../protocol" "../ble" "../data" "../logic" "../gps" "../")

# Board compile-time define — driven by Kconfig
if(CONFIG_BOARD_M5STACK_BASIC_V27)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC BOARD_M5STACK_BASIC_V27)
elseif(CONFIG_BOARD_WAVESHARE_S3_LCD19)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC BOARD_WAVESHARE_S3_LCD19)
endif()
