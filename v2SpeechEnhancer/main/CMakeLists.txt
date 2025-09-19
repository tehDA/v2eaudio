idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "twolf_control.srec"
    REQUIRES
        is31fl3216_custom
                audio_pipeline
        audio_stream
        audio_hal
        esp_peripherals
        esp_driver_i2c
        audio_board
        esp-adf-libs
        esp_common
		input_key_service
)
