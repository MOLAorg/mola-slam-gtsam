/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ASLAM_gtsam.cpp
 * @brief  SLAM in absolute coordinates with GTSAM factor graphs
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2018
 */

// MRPT headers must come first (due to Eigen plugin)
#include <mola-kernel/variant_helper.h>
#include <mola-kernel/yaml_helpers.h>
#include <mola-slam-gtsam/ASLAM_gtsam.h>
#include <yaml-cpp/yaml.h>

// GTSAM second:
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace mola;

static gtsam::Pose3 toPose3(const mrpt::math::TPose3D& p)
{
    gtsam::Pose3                  ret;
    mrpt::math::CQuaternionDouble q;
    mrpt::poses::CPose3D(p).getAsQuaternion(q);
    return gtsam::Pose3(
        gtsam::Rot3::Quaternion(q.r(), q.x(), q.y(), q.z()),
        gtsam::Point3(p.x, p.y, p.z));
}

static mrpt::math::TPose3D toTPose3D(const gtsam::Pose3& p)
{
    const auto HM = p.matrix();
    const auto H  = mrpt::math::CMatrixDouble44(HM);
    return mrpt::poses::CPose3D(H).asTPose();
}

ASLAM_gtsam::ASLAM_gtsam() = default;

void ASLAM_gtsam::initialize(const std::string& cfg_block)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg_block);

    // Mandatory parameters:
    auto cfg = YAML::Load(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(cfg, "params");
    auto params = cfg["params"];

    MRPT_TODO("Add priorities to initialize() for the map to be set-up 1st");

    // Ensure we have access to the worldmodel:
    ASSERT_(worldmodel_);
    ASSERT_(worldmodel_->entities_);
    // ASSERT_(worldmodel_->factors_);

    MRPT_TODO("Load existing map from world model?");

    MRPT_END
}
void ASLAM_gtsam::spinOnce()
{
    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    // Incremental SAM solution:
    gtsam::Values result;

    if (!state_.newfactors.empty())
    {
        std::lock_guard<typeof(isam2_lock_)> lock(isam2_lock_);
        ProfilerEntry tleg2(profiler_, "spinOnce.isam2_update");

        state_.isam2.update(state_.newfactors, state_.newvalues);
        result = state_.isam2.calculateEstimate();

        // reset accumulators of new slam factors:
        state_.newfactors = gtsam::NonlinearFactorGraph();
        state_.newvalues.clear();
    }

    if (result.size())
    {
        // MRPT_TODO("gtsam Values: add print(ostream) method");
        // result.print("isam2 result:");
        MRPT_LOG_DEBUG("iSAM2 new optimization results:");
        for (auto key_value = result.begin(); key_value != result.end();
             ++key_value)
        {
            const mola::id_t kf_id   = key_value->key;
            gtsam::Pose3     kf_pose = key_value->value.cast<gtsam::Pose3>();
            const auto       p       = toTPose3D(kf_pose);
            state_.last_kf_estimates[kf_id] = p;

            MRPT_LOG_DEBUG_STREAM("KF#" << kf_id << ": " << p.asString());
        }
    }

    MRPT_END
}

BackEndBase::ProposeKF_Output ASLAM_gtsam::doAddKeyFrame(
    const ProposeKF_Input& i)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddKeyFrame");
    ProposeKF_Output o;

    MRPT_LOG_DEBUG("Creating new KeyFrame");

    // Create entities in the worldmodel:
    auto& ents = *worldmodel_->entities_;
    MRPT_TODO("Map lock?");

    // If this is the first KF, create an absolute coordinate reference frame in
    // the map:
    if (state_.root_kf_id == INVALID_ID)
    {
        mola::RefPose3 root;
        state_.root_kf_id = ents.emplace_back(root);

        // And add a prior to iSAM2:
        auto priorModel = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4)
                .finished());

        state_.newvalues.insert(state_.root_kf_id, gtsam::Pose3::identity());
        state_.newfactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            state_.root_kf_id, gtsam::Pose3(), priorModel);

        // Return:
        o.new_kf_id = state_.root_kf_id;
        o.success   = true;
        return o;
    }
    else
    {
        // Regular KF:
        mola::RelPose3KF new_kf;
        new_kf.base_id_ = state_.root_kf_id;

        // Copy the raw observations (shallow copy):
        if (i.observations)
            new_kf.raw_observations_ =
                mrpt::obs::CSensoryFrame::Create(i.observations.value());

        // Add to the map:
        o.new_kf_id = ents.emplace_back(new_kf);
        o.success   = true;
        return o;
    }

    MRPT_END
}

BackEndBase::AddFactor_Output ASLAM_gtsam::doAddFactor(Factor& newF)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddFactor");
    AddFactor_Output o;

    // Create in the worldmodel:
    auto&       facts   = *worldmodel_->factors_;
    mola::fid_t newf_id = INVALID_FID;

    std::visit(
        overloaded{
            [&](const FactorRelativePose3& f) {
                MRPT_LOG_DEBUG("Adding new FactorRelativePose3");

                // Relative pose:
                const gtsam::Pose3 measure = toPose3(f.rel_pose_);

                // Initial estimation of the new KF:
                const gtsam::Pose3 to_pose_est =
                    toPose3(state_.last_kf_estimates[f.from_kf_]);
                state_.newvalues.insert(f.to_kf_, to_pose_est);

                MRPT_TODO("Actual GTSAM factor ID");
                newf_id    = 1123;
                auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
                state_.newfactors
                    .emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                        f.from_kf_, f.to_kf_, measure, noise);
            },
            [this, newF](auto f) { MRPT_LOG_ERROR("Unknown factor type!"); },
        },
        newF);

    o.success       = true;
    o.new_factor_id = newf_id;

    return o;

    MRPT_END
}
