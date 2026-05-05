cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE LIB_SOURCES
    "${CMAKE_SOURCE_DIR}/libraries/SX126x-Arduino/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/libraries/SX126x-Arduino/src/*.c"
    "${CMAKE_SOURCE_DIR}/libraries/TFT_eSPI/TFT_eSPI.cpp"
    "${CMAKE_SOURCE_DIR}/libraries/ModbusMaster/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/libraries/FT6336-arduino/*.cpp"
)

idf_component_register(
    SRCS
        "main.cpp"
        "touch.cpp"
        ${LIB_SOURCES}
    INCLUDE_DIRS
        "."
        "${CMAKE_SOURCE_DIR}/libraries/SX126x-Arduino/src"
        "${CMAKE_SOURCE_DIR}/libraries/TFT_eSPI"
        "${CMAKE_SOURCE_DIR}/libraries/ModbusMaster/src"
        "${CMAKE_SOURCE_DIR}/libraries/FT6336-arduino"
    REQUIRES
        espressif__arduino-esp32
        driver
        esp_timer
        freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=maybe-uninitialized
    -Wno-error=misleading-indentation
)
