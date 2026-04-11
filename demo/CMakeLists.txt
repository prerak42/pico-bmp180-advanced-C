cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(pico-bmp180-advanced-C C CXX ASM)
pico_sdk_init()

add_executable(pico-bmp180-advanced-C
    demo/demo.c
    bmp180.c
    bmp180_advanced.c
)

target_include_directories(pico-bmp180-advanced-C PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(pico-bmp180-advanced-C
    pico_stdlib
    hardware_i2c
    pico_sync
    m
)

pico_enable_stdio_usb(pico-bmp180-advanced-C 1)
pico_enable_stdio_uart(pico-bmp180-advanced-C 0)
pico_add_extra_outputs(pico-bmp180-advanced-C)