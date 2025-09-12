
# Migration: IDF v5.1 + ADF v2.7 -> IDF v5.3 + ADF master

This project has been adjusted to build with ESP-IDF **v5.3.x** and ESP-ADF **master**.

## What changed

- **Top-level CMake**: include ADF first, then IDF's `project.cmake` (per ADF docs).
- **Component manager**: lock file removed (`dependencies.lock`) so CMake can re-resolve components for IDF 5.3.
- **`main/idf_component.yml`**: set `idf` range to `>=5.3.0,<5.4` to ensure 5.3.x is used.
- **`components/my_board/CMakeLists.txt`**: migrated to `idf_component_register()` and kept linkage so `audio_board` depends on this component.
- **No code changes were required** for ADF APIs in this snapshot; if you use `audio_recorder` or `periph_wifi_init`, see the ADF v2.7 release notes (both changed).

## Build (Linux/macOS)

```bash
# 1) Get ADF master and its IDF 5.3 submodule
git clone https://github.com/espressif/esp-adf.git
cd esp-adf
./install.sh
. ./export.sh   # note the leading dot to source the env
# Verify:
idf.py --version  # should print IDF 5.3.x

# 2) Configure & build this project
cd /path/to/my_speech_enhancer_idf53
idf.py set-target esp32
idf.py menuconfig   # if needed
idf.py build flash monitor
```

## Windows (PowerShell)

```powershell
git clone https://github.com/espressif/esp-adf.git
cd esp-adf
.\install.ps1
.\export.ps1
idf.py --version

cd C:\path\to\my_speech_enhancer_idf53
idf.py set-target esp32
idf.py build flash monitor
```

## Notes

- IDF v5.3 split driver libraries (e.g. `esp_driver_i2c`). The umbrella `driver` component still works, so `REQUIRES driver` remains valid.
- I2S `i2s_event_data_t.data` is deprecated. If you use it directly, switch to `dma_buf` (IDF 5.3 migration guide).
- If you use legacy ADC calibration (`esp_adc_cal`), add `esp_adc` to your component requirements or migrate to the new ADC calibration APIs.

