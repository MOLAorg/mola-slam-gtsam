# mola-slam-gtsam
MOLA module: Absolute and Relative SLAM back-ends based on GTSAM factor graphs

These are *the reference implementations* of SLAM for MOLA as the time of writing,
although users are free of creating derived or brand-new SLAM modules as needed.

Provided MOLA modules:
* `ASLAM_gtsam`, type BackEndBase. SLAM in one absolute frame of reference.
* `RSLAM_gtsam`, type BackEndBase. SLAM in relative coordinates.

## Build and install
Refer to instructions in the root MOLA repo.
