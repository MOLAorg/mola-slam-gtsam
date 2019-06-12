# mola-slam-gtsam
MOLA module: Absolute and Relative SLAM back-ends based on GTSAM factor graphs

These are *the reference implementations* of SLAM for MOLA as the time of writing,
although users are free of creating derived or brand-new SLAM modules as needed.

Provided MOLA modules:
* `ASLAM_gtsam`, type BackEndBase. SLAM in one absolute frame of reference.
* `RSLAM_gtsam`, type BackEndBase. SLAM in relative coordinates. [Under development!]

## Build and install
Refer to the [root MOLA repository](https://github.com/MOLAorg/mola).

## Docs and examples
See this package page [in the documentation](https://docs.mola-slam.org/latest/modules.html).

## License
This package is released under the GNU GPL v3 license.
