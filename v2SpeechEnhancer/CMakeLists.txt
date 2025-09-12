cmake_minimum_required(VERSION 3.16)

# Pick up ESP-IDF and ADF paths from environment
set(IDF_PATH $ENV{IDF_PATH})
set(ADF_PATH $ENV{ADF_PATH})

# Add ESP-ADF and your custom component
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
    ${ADF_PATH}/components
)

include($ENV{ADF_PATH}/CMakeLists.txt)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(my_speech_enhancer)

# NOTE: For ADF Master + IDF v5.3, include ADF first per docs.
