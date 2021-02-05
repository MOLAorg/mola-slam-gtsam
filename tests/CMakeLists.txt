# Unit tests:
mola_add_test(
  TARGET  test-gtsam-basic-graph
  SOURCES test-gtsam-basic-graph.cpp
  LINK_LIBRARIES
    gtsam
)
target_include_directories(test-gtsam-basic-graph
    PRIVATE
    "${GTSAM_SOURCE_DIR}/gtsam/"
    "${GTSAM_BINARY_DIR}/"
    )
