cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smart_helmet_mon)

# Suppress warnings from external Arduino libraries
idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers;-Wno-cpp" APPEND)
