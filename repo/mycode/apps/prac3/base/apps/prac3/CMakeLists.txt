# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(prac3)

# Collect all the source files in app/prac3/src
FILE(GLOB app_sources src/main.c src/board_init.c)

# Collect all the source files in mylib
# Relative to CMakeLists root, app/prac3
FILE(GLOB lib_sources ../../mylib/*.c)

# Exclude specific files from lib_sources
list(FILTER lib_sources EXCLUDE REGEX "sensors.*")
list(FILTER lib_sources EXCLUDE REGEX "rtc.*")
list(FILTER lib_sources EXCLUDE REGEX "Si1133.*")

# tell CMake to build with the app and lib sources
target_sources(app PRIVATE ${app_sources} ${lib_sources})

# tell CMake where our header files are
target_include_directories(app PRIVATE ../../include)
