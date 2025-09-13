## Overview

Leverage a ESP32-LyraTD-MSC V2.2 board to enhance speech clarity through the use of the DSP to process the audio signals, the top board button and LEDs, and the 3 microphone array.
https://docs.espressif.com/projects/esp-adf/en/latest/design-guide/dev-boards/get-started-esp32-lyratd-msc.html#what-you-need

## Features
- Uses the microphone array to capture sound in either a omni directional or beam focused configuration, this mode can be changed by the user by pressing the mode button.
- Uses the DSP to process the audio captured by the microphone array and processes it to enhance the quality and clarity of speech.
- Pipes this improved audio to the headphone jack on the bottom board.
- Volume up and down buttons control amplitude of audio piped to headphone jack.
- Set, play, and rec buttons are used to enable / disable various DSP processing that is being performed and reconfigures the audio stream to the headphones accordingly.

## IDF and ADF version information

- idf.py --version
ESP-IDF v5.3.4-145-gb019b2e63d-dirty

- git -C %ADF_PATH% describe --tags --always
v2.7-149-gad4ac707