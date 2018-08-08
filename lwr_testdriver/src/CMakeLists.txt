set(CMAKE_CXX_STANDARD 11)
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

find_package(RST-RT REQUIRED)
IF (RST-RT_FOUND)
    message(STATUS "RST-RT found. Version: ${RST-RT_VERSION}")
    include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
    add_definitions(${RST-RT_CFLAGS})
    link_directories(${RST-RT_LIBRARY_DIRS})
ENDIF()

# Creates a component library liblwr_testdriver-<target>.so
# and installs in the directory lib/orocos/lwr_testdriver/
#
orocos_component(lwr_testdriver lwr_testdriver-component.hpp lwr_testdriver-component.cpp) # ...you may add multiple source files
target_link_libraries(${CMAKE_PROJECT_NAME} ${RST-RT_LIBRARIES})
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/lwr_testdriver/ directory
#
orocos_install_headers(lwr_testdriver-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

