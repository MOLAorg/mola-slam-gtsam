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

#include <mola-kernel/yaml_helpers.h>
#include <mola-slam-gtsam/RSLAM_gtsam.h>
#include <mrpt/containers/yaml.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(RSLAM_gtsam, BackEndBase, mola)

RSLAM_gtsam::RSLAM_gtsam() = default;

void RSLAM_gtsam::initialize(const std::string& cfg_block)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg_block);

    // Mandatory parameters:
    auto cfg = mrpt::containers::yaml::FromText(cfg_block);

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

BackEndBase::ProposeKF_Output RSLAM_gtsam::doAddKeyFrame(
    const ProposeKF_Input& i)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddKeyFrame");
    ProposeKF_Output o;

    MRPT_LOG_DEBUG("Creating new KeyFrame");

    // worldmodel_-> ...;
    MRPT_TODO("Continue here!");

    // auto e = std::make_shared<EntityBase>();
    // o.new_kf_id = ents.emplace_back(e);
    // o.success = true;
    return o;

    MRPT_END
}

BackEndBase::AddFactor_Output RSLAM_gtsam::doAddFactor(Factor& f)
{
    MRPT_START

    THROW_EXCEPTION("Implement me!");

    MRPT_END
}
void RSLAM_gtsam::doAdvertiseUpdatedLocalization(
    AdvertiseUpdatedLocalization_Input l)
{
    MRPT_START

    THROW_EXCEPTION("Implement me!");

    MRPT_END
}
