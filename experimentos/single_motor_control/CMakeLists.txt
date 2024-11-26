# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

add_compile_options(-Wno-format)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(single_motor_control)

# Commented because components directory is added automatically
# set(EXTRA_COMPONENT_DIRS components)
