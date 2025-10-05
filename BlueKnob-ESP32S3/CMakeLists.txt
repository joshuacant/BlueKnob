# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(BlueKnob)

# Workaround for a compilation error on Windows (see: https://github.com/espressif/esp-idf/issues/10568)
set_source_files_properties(C:/ProgramData/esp-idf/v5.5.1/esp-idf/components/esp_lcd/rgb/esp_lcd_panel_rgb.c PROPERTIES HEADER_FILE_ONLY TRUE)