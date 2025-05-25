cmake_minimum_required(VERSION 3.13.1)

set(DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/m5stack_core2.overlay
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hyades_cyan_mobile)

target_include_directories(app PRIVATE
  include
)

zephyr_include_directories(lib/tjpgd)

target_sources(app PRIVATE
  src/main.c
  lib/tjpgd/tjpgd.c
)