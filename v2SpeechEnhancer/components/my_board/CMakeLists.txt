# Custom audio board component (ADF)
# Updated for ESP-IDF v5.3 / ADF master: use idf_component_register

idf_component_register(
    SRCS
        "my_board_v1_0/board.c"
        "my_board_v1_0/board_pins_config.c"
        "my_codec_driver/new_codec.c" # replaced below
    INCLUDE_DIRS
        "my_board_v1_0"
        "my_codec_driver"
    PRIV_REQUIRES
        audio_sal
        audio_hal
        esp_dispatcher
        esp_peripherals
        display_service
)

# Ensure the audio_board target links against this custom board so apps that depend on
# audio_board get our implementation too.
idf_component_get_property(audio_board_lib audio_board COMPONENT_LIB)
set_property(TARGET ${audio_board_lib} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})