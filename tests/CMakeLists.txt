# Unit tests:
mola_add_executable(
  DONT_INSTALL
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
