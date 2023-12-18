# SPDX-License-Identifier: Apache-2.0
set(BOARD zest_core_stm32l4a6rg)
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("zephyr honeywell hpma115 example")

add_subdirectory(drivers)

target_sources(app PRIVATE src/main.c)
