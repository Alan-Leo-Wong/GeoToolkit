###########################################
# Root CMakeList
###########################################

# General information
cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0079 NEW)
cmake_policy(SET CMP0135 NEW)

###########################################
# Options
option(ENABLE_TEST "Enable test" ON)

# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR "Use AntTweakBar" OFF)
option(LIBIGL_WITH_CGAL "Use CGAL" OFF)
option(LIBIGL_WITH_COMISO "Use CoMiso" OFF)
option(LIBIGL_WITH_CORK "Use Cork" OFF)
option(LIBIGL_WITH_LIM "Use LIM" OFF)
option(LIBIGL_WITH_MATLAB "Use Matlab" OFF)
option(LIBIGL_WITH_MOSEK "Use MOSEK" OFF)
option(LIBIGL_WITH_PNG "Use PNG" OFF)
option(LIBIGL_WITH_PYTHON "Use Python" OFF)
option(LIBIGL_WITH_TETGEN "Use Tetgen" OFF)
option(LIBIGL_WITH_TRIANGLE "Use Triangle" OFF)
option(LIBIGL_WITH_XML "Use XML" OFF)

# gui
option(LIBIGL_WITH_EMBREE "Use Embree" ON)
option(LIBIGL_WITH_NANOGUI "Use Nanogui menu" ON)
option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
option(LIBIGL_WITH_VIEWER "Use OpenGL viewer" ON)

# geogram
option(GEOGRAM_WITH_TRIANGLE "Use Triangle" OFF)

# used for GeoboxDownloadExternal.cmake
#option(AUTO_DOWNLOAD "Auto download dependencies" ON)

###########################################
# Project settings
project(GeoToolkit)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GEOToolKit_EXTERNAL ${CMAKE_BINARY_DIR}/external/)

# RelWithDebug for MSVC
if (MSVC)
    # https://github.com/mozilla/sccache/issues/242
    if (CMAKE_CXX_COMPILER_LAUNCHER STREQUAL "sccache")
        string(REGEX REPLACE "/Z[iI7]" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Z7")
    endif ()
endif ()

###########################################
# CMake module path
list(PREPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_LIST_DIR}/cmake/
    ${CMAKE_CURRENT_LIST_DIR}/cmake/find
    ${CMAKE_CURRENT_LIST_DIR}/cmake/common
    ${CMAKE_CURRENT_LIST_DIR}/cmake/external)

# Include helper functions
include(add_lib)
include(add_test)
include(get_sub_dirs)
include(igl_copy_dll)

###########################################
# 3rdParty
include(GeoToolKitDependencies)

# Enable OpenMP
find_package(OpenMP REQUIRED)
if (OpenMP_CXX_FOUND)
    add_compile_options(${OpenMP_CXX_FLAGS})
endif ()

###########################################
# Project libraries and executables
add_subdirectory(src)
add_subdirectory(app)