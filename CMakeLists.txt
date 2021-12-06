# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2021, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-slam-gtsam LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola-common REQUIRED)

# find dependencies:
find_package(mrpt-obs REQUIRED)
find_package(mrpt-gui) # TODO: Remove! when the gui -> other mapviz module
find_package(mrpt-graphs) # TODO: Remove! when the gui -> other mapviz module
find_mola_package(mola-kernel)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mola-kernel
		gtsam
		#gtsam_unstable
	PRIVATE_LINK_LIBRARIES
		mrpt::obs
		mrpt::gui
		mrpt::graphs
	CMAKE_DEPENDENCIES
		mola-kernel
		gtsam
		mrpt-obs
		mrpt-graphs
)

if (CMAKE_COMPILER_IS_GNUCXX)
	target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-shadow" "-Wno-unused-parameter")
endif()


target_include_directories(${PROJECT_NAME}
    PRIVATE
    "${GTSAM_SOURCE_DIR}/gtsam/"
    "${GTSAM_BINARY_DIR}/"
    )

# -----------------------
# define tests:
enable_testing()
add_subdirectory(tests)
