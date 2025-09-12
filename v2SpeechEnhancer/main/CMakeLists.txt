idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        is31fl3216_custom
		audio_pipeline
        audio_stream
        audio_hal
        esp_peripherals
        audio_board
        esp-adf-libs
        esp_common
		input_key_service
)
