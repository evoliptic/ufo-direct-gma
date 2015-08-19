cmake_minimum_required(VERSION 2.6)
project(ufo C)

set(PKG_UFO_CORE_MIN_REQUIRED "0.6")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(src)
