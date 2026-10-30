# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# common/protocol holds the radio frame definition shared with the node firmware.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../common")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(app-template)
