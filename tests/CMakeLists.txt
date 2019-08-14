# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2) ICP C++ library
#
# Copyright (C) 2018-2019, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under FreeBSD 3. See LICENSE file
# ------------------------------------------------------------------------------

mola_add_executable(
    TARGET  test-gtsam-basic-graph
    SOURCES test-gtsam-basic-graph.cpp
	LINK_LIBRARIES
	    gtsam
)
add_test(SLAM_GTSAM_basic_graph ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test-gtsam-basic-graph)


target_include_directories(test-gtsam-basic-graph
    PRIVATE
    "${GTSAM_SOURCE_DIR}/gtsam/"
    "${GTSAM_BINARY_DIR}/"
    )
