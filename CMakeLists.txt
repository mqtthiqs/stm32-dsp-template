cmake_minimum_required(VERSION 3.16)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "0.0.0")
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project("stm32-dsp-template" VERSION 1.0.0)

add_subdirectory(src)
add_subdirectory(lib)
