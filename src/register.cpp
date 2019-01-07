/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   register.cpp
 * @brief  Register MOLA modules in the factory
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2018
 */

/** \defgroup mola_slam_gtsam_grp mola-slam-gtsam
 * MOLA module: Relative and absolute-coordinates SLAM back-ends based on GTSAM
 * factor graphs.
 *
 *  This is *the reference implementation* of SLAM for MOLA as the time of
 * writing, although users are free of creating derived or brand new SLAM
 * modules as needed.
 *
 */

#include <mola-slam-gtsam/ASLAM_gtsam.h>
#include <mola-slam-gtsam/RSLAM_gtsam.h>
#include <mrpt/core/initializer.h>
using namespace mola;

MRPT_INITIALIZER(do_register)
{
    MOLA_REGISTER_MODULE(RSLAM_gtsam);
    MOLA_REGISTER_MODULE(ASLAM_gtsam);
}
