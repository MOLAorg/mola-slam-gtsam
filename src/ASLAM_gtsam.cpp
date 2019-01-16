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
#include <mrpt/opengl/graph_tools.h>  // TODO: Remove after vizmap module
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

    // Ensure we have access to the worldmodel:
    ASSERT_(worldmodel_);

    MRPT_TODO("Load existing map from world model?");

    // Init iSAM2:
    gtsam::ISAM2Params parameters;
    // parameters.relinearizeThreshold   = 0.1;
    // parameters.relinearizeSkip        = 10;
    parameters.cacheLinearizedFactors = false;
    parameters.enableDetailedResults  = false;

    state_.isam2 = std::make_unique<gtsam::ISAM2>(parameters);

    MRPT_END
}
void ASLAM_gtsam::spinOnce()
{
    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    // Incremental SAM solution:
    gtsam::Values result;

    gtsam::ISAM2Result isam2_res;
    {
        std::lock_guard<typeof(isam2_lock_)> lock(isam2_lock_);
        if (!state_.newfactors.empty())
        {
            {
                ProfilerEntry tle(profiler_, "spinOnce.isam2_update");
                state_.isam2->update(state_.newfactors, state_.newvalues);
            }

            {
                ProfilerEntry tle(profiler_, "spinOnce.isam2_calcEstimate");
                result = state_.isam2->calculateEstimate();
            }

            // reset accumulators of new slam factors:
            state_.newfactors = gtsam::NonlinearFactorGraph();
            state_.newvalues.clear();
        }
    }

    if (result.size())
    {
        std::lock_guard<decltype(last_kf_estimates_lock_)> lock(
            last_kf_estimates_lock_);

        // MRPT_TODO("gtsam Values: add print(ostream) method");
        // result.print("isam2 result:");

        MRPT_LOG_INFO_STREAM(
            "iSAM2 ran for " << result.size() << " variables.");

        MRPT_LOG_DEBUG("iSAM2 new optimization results:");
        for (auto key_value : result)
        {
            const mola::id_t kf_id   = key_value.key;
            gtsam::Pose3     kf_pose = key_value.value.cast<gtsam::Pose3>();
            const auto       p       = toTPose3D(kf_pose);
            state_.last_kf_estimates[kf_id] = p;
            // mapviz:
            state_.vizmap.nodes[kf_id] = mrpt::poses::CPose3D(p);

            // MRPT_LOG_DEBUG_STREAM("KF#" << kf_id << ": " << p.asString());
        }

        MRPT_TODO("Send to the world model");
    }

    if (isam2_res.detail)
    {
        for (auto keyedStatus : isam2_res.detail->variableStatus)
        {
            using std::cout;
            const auto& status = keyedStatus.second;
            gtsam::PrintKey(keyedStatus.first);
            cout << " {"
                 << "\n";
            cout << "reeliminated: " << status.isReeliminated << "\n";
            cout << "relinearized above thresh: "
                 << status.isAboveRelinThreshold << "\n";
            cout << "relinearized involved: " << status.isRelinearizeInvolved
                 << "\n";
            cout << "relinearized: " << status.isRelinearized << "\n";
            cout << "observed: " << status.isObserved << "\n";
            cout << "new: " << status.isNew << "\n";
            cout << "in the root clique: " << status.inRootClique << "\n";
            cout << "}"
                 << "\n";
        }
    }

    // Show in GUI:
    // -------------------
    auto di = std::make_shared<DisplayInfo>();
    {
        std::lock_guard<decltype(last_kf_estimates_lock_)> lock(
            last_kf_estimates_lock_);
        di->vizmap = state_.vizmap;  // make a copy
    }
    gui_updater_pool_.enqueue(&ASLAM_gtsam::doUpdateDisplay, this, di);

    MRPT_END
}

BackEndBase::ProposeKF_Output ASLAM_gtsam::doAddKeyFrame(
    const ProposeKF_Input& i)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddKeyFrame");
    ProposeKF_Output o;

    MRPT_LOG_DEBUG("Creating new KeyFrame");

    // If this is the first KF, create an absolute coordinate reference frame in
    // the map:
    if (state_.root_kf_id == INVALID_ID)
    {
        mola::RefPose3 root;

        // Add to the WorldModel
        worldmodel_->entities_lock();
        state_.root_kf_id = worldmodel_->entity_push_back(root);
        worldmodel_->entities_unlock();

        // And add a prior to iSAM2:
        auto priorModel = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4)
                .finished());

        {
            std::lock_guard<decltype(isam2_lock_)> lock(isam2_lock_);
            state_.newvalues.insert(
                state_.root_kf_id, gtsam::Pose3::identity());
            state_.newfactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                state_.root_kf_id, gtsam::Pose3(), priorModel);
        }

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

        // Add to the WorldModel:
        worldmodel_->entities_lock();
        o.new_kf_id = worldmodel_->entity_emplace_back(new_kf);
        worldmodel_->entities_unlock();

        o.success = true;
        return o;
    }

    MRPT_END
}

BackEndBase::AddFactor_Output ASLAM_gtsam::doAddFactor(Factor& newF)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddFactor");
    AddFactor_Output o;

    mola::fid_t fid = INVALID_FID;

    std::visit(
        overloaded{
            [&](const FactorRelativePose3& f) { fid = addFactor(f); },
            [this, newF]([[maybe_unused]] auto f) {
                MRPT_LOG_ERROR("Unknown factor type!");
            },
        },
        newF);

    o.success       = true;
    o.new_factor_id = fid;

    return o;

    MRPT_END
}

fid_t ASLAM_gtsam::addFactor(const FactorRelativePose3& f)
{
    MRPT_START
    MRPT_LOG_DEBUG("Adding new FactorRelativePose3");

    // Add to the WorldModel:
    worldmodel_->factors_lock();
    const fid_t new_fid = worldmodel_->factor_push_back(f);
    worldmodel_->factors_unlock();

    // Relative pose:
    const gtsam::Pose3 measure = toPose3(f.rel_pose_);

    // Initial estimation of the new KF:
    mrpt::math::TPose3D to_pose_est;
    bool                to_id_observed_first_time = false;
    {
        std::lock_guard<decltype(last_kf_estimates_lock_)> lock(
            last_kf_estimates_lock_);

        const auto& from_pose_est = state_.last_kf_estimates[f.from_kf_];
        from_pose_est.composePose(f.rel_pose_, to_pose_est);

        // Store the result just in case we need it as a quick guess in next
        // factors, before running the actual optimizer:
        auto [it, was_new] =
            state_.last_kf_estimates.insert_or_assign(f.to_kf_, to_pose_est);
        to_id_observed_first_time = was_new;

        // mapviz:
        state_.vizmap.nodes[f.to_kf_] = mrpt::poses::CPose3D(to_pose_est);
        state_.vizmap.insertEdgeAtEnd(
            f.from_kf_, f.to_kf_, mrpt::poses::CPose3D(to_pose_est));
    }

    // Noise model:
    MRPT_TODO("Shared noise models");
    auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);

    {
        MRPT_TODO("add a new lock for last_kf_estimates");
        std::lock_guard<decltype(isam2_lock_)> lock(isam2_lock_);

        // Add to list of initial guess (if not done already with a former
        // factor):
        if (to_id_observed_first_time)
            state_.newvalues.insert(f.to_kf_, toPose3(to_pose_est));

        state_.newfactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            f.from_kf_, f.to_kf_, measure, noise);
    }

    return new_fid;

    MRPT_END
}

bool ASLAM_gtsam::doFactorExistsBetween(id_t a, id_t b)
{
    MRPT_START

    MRPT_END
}

void ASLAM_gtsam::doUpdateDisplay(std::shared_ptr<DisplayInfo> di)
{
    try
    {
        using namespace mrpt::gui;
        using namespace mrpt::opengl;
        using namespace std::string_literals;

        ProfilerEntry tleg(profiler_, "doUpdateDisplay");

        if (!display_)
        {
            display_ = CDisplayWindow3D::Create(
                ExecutableBase::getModuleInstanceName(), 650, 480);
        }

        mrpt::poses::CPose3D last_kf_pos;
        if (!di->vizmap.nodes.empty())
        {
            // point camera to last KF:
            last_kf_pos = di->vizmap.nodes.rbegin()->second;
        }

        {  // lock scene
            COpenGLScene::Ptr      scene;
            CDisplayWindow3DLocker lock(*display_, scene);

            // Update scene:
            scene->clear();

            mrpt::system::TParametersDouble params;
            params["show_ID_labels"]    = 1;
            params["show_ground_grid"]  = 1;
            params["show_edges"]        = 1;
            params["show_node_corners"] = 1;

            auto gl_graph =
                mrpt::opengl::graph_tools::graph_visualize(di->vizmap, params);

            scene->insert(gl_graph);
        }
        display_->setCameraPointingToPoint(
            last_kf_pos.x(), last_kf_pos.y(), last_kf_pos.z());

        display_->repaint();
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    }
}
