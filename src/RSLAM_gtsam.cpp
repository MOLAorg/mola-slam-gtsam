/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RSLAM_gtsam.cpp
 * @brief  Reference implementation of relative SLAM with GTSAM factor graphs
 * @author Jose Luis Blanco Claraco
 * @date   Dec 21, 2018
 */

/** \defgroup mola_slam_gtsam_grp mola-slam-gtsam
 * MOLA module: Relative SLAM back-end based on GTSAM factor graphs.
 *
 *  This is *the reference implementation* of SLAM for MOLA as the time of
 * writing, although users are free of creating derived or brand new SLAM
 * modules as needed.
 *
 */

#include <mola-kernel/yaml_helpers.h>
#include <mola-slam-gtsam/RSLAM_gtsam.h>
#include <mrpt/core/initializer.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(RSLAM_gtsam)}

RSLAM_gtsam::RSLAM_gtsam()
{
    this->setLoggerName("RSLAM_gtsam");

    gtsam::BetweenFactor<double> f;
}

void RSLAM_gtsam::initialize(const std::string& cfg_block)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg_block);

    // Mandatory parameters:
    auto cfg = YAML::Load(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(cfg, "params");
    auto params = cfg["params"];

    MRPT_END
}
void RSLAM_gtsam::spinOnce()
{
    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    MRPT_END
}

BackEndBase::ProposeKF_Output RSLAM_gtsam::doProposeNewKeyFrame(
    const ProposeKF_Input& i)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doProposeNewKeyFrame");
    ProposeKF_Output o;

    MRPT_LOG_DEBUG("Creating new KeyFrame");

    // worldmodel_->

    return o;

    MRPT_END
}
