# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2019, José Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-slam-gtsam LANGUAGES CXX)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

add_library(${PROJECT_NAME} SHARED
    ${LIB_SRCS}
    ${LIB_PUBLIC_HDRS}
    )

# Define common flags:
mola_set_target_build_options(${PROJECT_NAME})
mola_configure_library(${PROJECT_NAME})

target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-shadow" "-Wno-unused-parameter")

# find dependencies:
find_package(mrpt-obs)
find_package(mrpt-gui) # TODO: Remove! when the gui -> other mapviz module
find_package(mrpt-graphs) # TODO: Remove! when the gui -> other mapviz module

# Public lib Dependencies:
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mola-kernel
    gtsam
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    "${GTSAM_SOURCE_DIR}/gtsam/"
    "${GTSAM_BINARY_DIR}/"
    )

# Private lib Dependencies:
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    mrpt::obs
    mrpt::gui
    mrpt::graphs
)
