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

#include <gtsam/inference/Symbol.h>  // X(), V() symbols
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <mola-kernel/entities/entities-common.h>
#include <mola-kernel/lock_helper.h>
#include <mola-kernel/variant_helper.h>  // overloaded{}
#include <mola-kernel/yaml_helpers.h>
#include <mola-slam-gtsam/ASLAM_gtsam.h>
#include <mola-slam-gtsam/gtsam_mola_bridge.h>
#include <mrpt/opengl/CSetOfLines.h>  // TODO: Remove after vizmap module
#include <mrpt/opengl/graph_tools.h>  // TODO: Remove after vizmap module
#include <mrpt/opengl/stock_objects.h>  // TODO: Remove after vizmap module
#include <yaml-cpp/yaml.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(ASLAM_gtsam, BackEndBase, mola);

namespace gtsam
{
using namespace std;
using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

struct IMUHelper
{
    IMUHelper()
    {
        {
            auto gaussian = noiseModel::Diagonal::Sigmas(
                (Vector(6) << Vector3::Constant(5.0e-2),
                 Vector3::Constant(5.0e-3))
                    .finished());
            auto huber = noiseModel::Robust::Create(
                noiseModel::mEstimator::Huber::Create(1.345), gaussian);

            biasNoiseModel = huber;
        }

        {
            auto gaussian = noiseModel::Isotropic::Sigma(3, 0.01);
            auto huber    = noiseModel::Robust::Create(
                noiseModel::mEstimator::Huber::Create(1.345), gaussian);

            velocityNoiseModel = huber;
        }

        // expect IMU to be rotated in image space co-ords
        auto p = boost::make_shared<PreintegratedCombinedMeasurements::Params>(
            Vector3(0.0, 9.8, 0.0));

        p->accelerometerCovariance =
            I_3x3 * pow(0.0565, 2.0);  // acc white noise in continuous
        p->integrationCovariance =
            I_3x3 * 1e-9;  // integration uncertainty continuous
        p->gyroscopeCovariance =
            I_3x3 * pow(4.0e-5, 2.0);  // gyro white noise in continuous
        p->biasAccCovariance =
            I_3x3 * pow(0.00002, 2.0);  // acc bias in continuous
        p->biasOmegaCovariance =
            I_3x3 * pow(0.001, 2.0);  // gyro bias in continuous
        p->biasAccOmegaInt = Matrix::Identity(6, 6) * 1e-5;

        // body to IMU rotation
        // clang-format off
        Rot3 iRb(
            0.036129, -0.998727, 0.035207,
            0.045417, -0.033553, -0.998404,
            0.998315, 0.037670, 0.044147);
        // clang-format on

        // body to IMU translation (meters)
        Point3 iTb(0, 0, 0);  // 0.03, -0.025, -0.06);

        // body in this example is the left camera
        p->body_P_sensor = Pose3(iRb, iTb);

        Rot3  prior_rotation = Rot3(I_3x3);
        Pose3 prior_pose(prior_rotation, Point3(0, 0, 0));

        Vector3 acc_bias(0.0, -0.0942015, 0.0);  // in camera frame
        Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);

        priorImuBias = imuBias::ConstantBias(acc_bias, gyro_bias);

        prevState = NavState(prior_pose, Vector3(0, 0, 0));
        propState = prevState;
        prevBias  = priorImuBias;

        preintegrated = new PreintegratedCombinedMeasurements(p, priorImuBias);
    }

    imuBias::ConstantBias          priorImuBias;  // assume zero initial bias
    noiseModel::Robust::shared_ptr velocityNoiseModel;
    noiseModel::Robust::shared_ptr biasNoiseModel;
    NavState                       prevState;
    NavState                       propState;
    imuBias::ConstantBias          prevBias;
    PreintegratedCombinedMeasurements* preintegrated;
};

}  // namespace gtsam

gtsam::IMUHelper imu;

// ----------------------------------------------

ASLAM_gtsam::ASLAM_gtsam() = default;

void ASLAM_gtsam::initialize(const std::string& cfg_block)
{
    MRPT_START
    ProfilerEntry tle(profiler_, "initialize");

    MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg_block);

    // Mandatory parameters:
    auto c = YAML::Load(cfg_block);

    ENSURE_YAML_ENTRY_EXISTS(c, "params");
    auto cfg = c["params"];

    {
        ENSURE_YAML_ENTRY_EXISTS(cfg, "state_vector");
        std::string s = cfg["state_vector"].as<std::string>();
        params_.state_vector =
            mrpt::typemeta::TEnumType<StateVectorType>::name2value(s);
    }

    YAML_LOAD_REQ(params_, use_incremental_solver, bool);
    YAML_LOAD_OPT(params_, save_trajectory_file_prefix, std::string);
    YAML_LOAD_OPT(params_, isam2_additional_update_steps, int);
    YAML_LOAD_OPT(params_, isam2_relinearize_threshold, double);
    YAML_LOAD_OPT(params_, isam2_relinearize_skip, int);

    YAML_LOAD_OPT(params_, const_vel_model_std_pos, double);
    YAML_LOAD_OPT(params_, const_vel_model_std_vel, double);
    YAML_LOAD_OPT(params_, max_interval_between_kfs_for_dynamic_model, double);

    // Ensure we have access to the worldmodel:
    ASSERT_(worldmodel_);

    MRPT_TODO("Load existing map from world model?");

    // Init iSAM2:
    if (params_.use_incremental_solver)
    {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold   = params_.isam2_relinearize_threshold;
        parameters.relinearizeSkip        = params_.isam2_relinearize_skip;
        parameters.cacheLinearizedFactors = false;  // for smart factors to work
        parameters.enableDetailedResults  = true;
        MRPT_TODO("make a param");
        parameters.evaluateNonlinearError = true;

        state_.isam2 = std::make_unique<gtsam::ISAM2>(parameters);
    }

    MRPT_END
}
void ASLAM_gtsam::spinOnce()
{
    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    MRPT_TODO("Refactor into 2-3 methods");

    // Incremental SAM solution:
    gtsam::Values                      result;
    gtsam::ISAM2Result                 isam2_res, isam2_res_refine;
    std::map<std::size_t, mola::fid_t> processedFactor2molaid;

    if (params_.use_incremental_solver)
    {
        auto lock = lockHelper(isam2_lock_);

        // smart factors are not re-added to newfactors, but we should
        // re-optimize if needed anyway:
        if (!state_.newfactors.empty() || !state_.newvalues.empty() ||
            !state_.changedSmartFactors.empty())
        {
            // Let iSAM2 know about smart factors that might have changed:
            gtsam::ISAM2UpdateParams updateParams;
            updateParams.newAffectedKeys =
                std::move(state_.changedSmartFactors);

            {
                ProfilerEntry tle(profiler_, "spinOnce.isam2_update");
                isam2_res = state_.isam2->update(
                    state_.newfactors, state_.newvalues, updateParams);

                // Extra refining steps:
                for (int i = 0; i < params_.isam2_additional_update_steps; i++)
                    isam2_res_refine = state_.isam2->update();
            }

            {
                ProfilerEntry tle(profiler_, "spinOnce.isam2_calcEstimate");
                // result = state_.isam2->calculateEstimate();
                result = state_.isam2->calculateBestEstimate();
            }

            state_.last_values = result;

            // If we processed a "newvalue" that was the first gross estimate of
            // a KF, it was not marked as "already existing" in kf_has_value, in
            // the hope that a Factor arrived before running the optimization.
            // If that didn't happen, now is the moment to mark it in
            // "kf_has_value" to avoid iSAM2 to complain about an attempt to
            // duplicate a symbol:
            {
                auto lk = lockHelper(keys_map_lock_);

                for (const auto p : state_.newvalues)
                {
                    if (auto it_kf = state_.gtsam2mola[KF_KEY_POSE].find(p.key);
                        it_kf != state_.gtsam2mola[KF_KEY_POSE].end())
                    {
                        const mola::id_t kf_id = it_kf->second;
                        state_.kf_has_value.insert(kf_id);
                    }
                }
            }

            processedFactor2molaid = state_.newFactor2molaid;

            // reset accumulators of new slam factors:
            state_.newfactors.resize(0);
            state_.newvalues.clear();
            state_.changedSmartFactors.clear();
            state_.newFactor2molaid.clear();
        }
    }
    else
    {
        auto lock = lockHelper(isam2_lock_);

        state_.newfactors.print("factors =====================\n");
        state_.newvalues.print("values =====================\n");

        if (!state_.newfactors.empty())
        {
            ProfilerEntry tle(
                profiler_, "spinOnce.LevenbergMarquardtOptimizer");

            gtsam::LevenbergMarquardtOptimizer optimizer(
                state_.newfactors, state_.newvalues);
            result = optimizer.optimize();

            MRPT_LOG_DEBUG_STREAM(
                "LevenbergMarquardt ran: error=" << optimizer.error()
                                                 << "  iterations="
                                                 << optimizer.iterations());
        }

        state_.newvalues   = result;
        state_.last_values = state_.newvalues;

#if 0
        gtsam::Marginals marginals(state_.newfactors, result);

        using gtsam::symbol_shorthand::V;
        using gtsam::symbol_shorthand::X;

        for (const auto kv : state_.last_values)
        {
            std::cout << "cov key: " << kv.key << "\n"
                      << marginals.marginalCovariance(kv.key) << "\n";
        };
#endif
    }

    if (result.size())
    {
        // MRPT_TODO("gtsam Values: add print(ostream) method");
        if (this->isLoggingLevelVisible(mrpt::system::LVL_DEBUG))
            result.print("isam2 result:");

        if (isam2_res.errorBefore && isam2_res.errorAfter)
        {
            MRPT_LOG_DEBUG_STREAM(
                "Error initial  : " << *isam2_res.errorBefore);
            MRPT_LOG_DEBUG_STREAM("Error 1st pass : " << *isam2_res.errorAfter);
        }
        if (isam2_res_refine.errorAfter)
            MRPT_LOG_DEBUG_STREAM(
                "Error final    : " << *isam2_res_refine.errorAfter);

        // Process new factor IDs:
        for (const auto& f2id : processedFactor2molaid)
        {
            const auto mola_id  = f2id.second;
            const auto in_idx   = f2id.first;
            const auto gtsam_id = isam2_res.newFactorsIndices.at(in_idx);
            auto&      ids      = state_.stereo_factors.ids;

            MRPT_LOG_DEBUG_STREAM(
                "Processed: factor id #" << mola_id << "  <==> GTSAM factor #"
                                         << gtsam_id);

            ids.mola2gtsam[mola_id]  = gtsam_id;
            ids.gtsam2mola[gtsam_id] = mola_id;
        }

        MRPT_LOG_INFO_STREAM(
            "iSAM2 ran for " << result.size() << " variables.");

        // Send only those variables that have been updated:
        gtsam::KeySet changedKeys;
        if (params_.use_incremental_solver)
        {
            ASSERT_(isam2_res.detail);
            for (auto keyedStatus : isam2_res.detail->variableStatus)
            {
                // const auto&       status    = keyedStatus.second;
                const gtsam::Key& key = keyedStatus.first;
                changedKeys.insert(key);
            }
        }
        else
        {
            // Batch: all keys
            for (const auto& keyVal : result) changedKeys.insert(keyVal.key);
        }

        auto lk = lockHelper(keys_map_lock_);

        // Send values to the world model:
        worldmodel_->entities_lock_for_write();

        for (auto key : changedKeys)
        {
            using std::cout;
            const gtsam::Value& value = result.at(key);

            if (auto it_kf = state_.gtsam2mola[KF_KEY_POSE].find(key);
                it_kf != state_.gtsam2mola[KF_KEY_POSE].end())
            {
                const mola::id_t kf_id   = it_kf->second;
                gtsam::Pose3     kf_pose = value.cast<gtsam::Pose3>();

                // Dont update the pose of the global reference, fixed to
                // Identity()
                if (kf_id != state_.root_kf_id)
                    updateEntityPose(worldmodel_->entity_by_id(kf_id), kf_pose);

                // mapviz:
                const auto p               = toTPose3D(kf_pose);
                state_.vizmap.nodes[kf_id] = mrpt::poses::CPose3D(p);
            }
            else if (auto it_kf = state_.gtsam2mola[KF_KEY_VEL].find(key);
                     it_kf != state_.gtsam2mola[KF_KEY_VEL].end())
            {
                const mola::id_t kf_id  = it_kf->second;
                gtsam::Velocity3 kf_vel = value.cast<gtsam::Velocity3>();

                // worldmodel:
                updateEntityVel(worldmodel_->entity_by_id(kf_id), kf_vel);
                // mapviz:
                state_.vizmap_dyn[kf_id].vx = kf_vel.x();
                state_.vizmap_dyn[kf_id].vy = kf_vel.y();
                state_.vizmap_dyn[kf_id].vz = kf_vel.z();
            }
        }
        worldmodel_->entities_unlock_for_write();
    }

#if 0
    MRPT_LOG_DEBUG("iSAM2 detail status:");
    for (auto keyedStatus : isam2_res.detail->variableStatus)
    {
        using std::cout;
        const auto& status = keyedStatus.second;
        gtsam::PrintKey(keyedStatus.first);
        cout << " {"
             << "\n";
        cout << "reeliminated: " << status.isReeliminated << "\n";
        cout << "relinearized above thresh: " << status.isAboveRelinThreshold
             << "\n";
        cout << "relinearized involved: " << status.isRelinearizeInvolved
             << "\n";
        cout << "relinearized: " << status.isRelinearized << "\n";
        cout << "observed: " << status.isObserved << "\n";
        cout << "new: " << status.isNew << "\n";
        cout << "in the root clique: " << status.inRootClique << "\n";
        cout << "}"
             << "\n";
    }
#endif

    // Show in GUI:
    // -------------------
    auto di = std::make_shared<DisplayInfo>();
    {
        auto lock = lockHelper(vizmap_lock_);

        if (state_.last_created_kf_id != mola::INVALID_ID)
        {
            worldmodel_->entities_lock_for_read();
            di->current_tim = mola::entity_get_timestamp(
                worldmodel_->entity_by_id(state_.last_created_kf_id));
            worldmodel_->entities_unlock_for_read();
        }
        di->vizmap = state_.vizmap;  // make a copy
    }
    gui_updater_pool_.enqueue(&ASLAM_gtsam::doUpdateDisplay, this, di);

    MRPT_END
}

/** This creates TWO entities:
 * - The global coordinate reference frame (state_.root_kf_id), and
 * - The actual first *Keyframe* (with the desired state space model).
 * It returns the ID of the latter.
 * isam2_lock_ is locked from the caller site.
 */
mola::id_t ASLAM_gtsam::internal_addKeyFrame_Root(const ProposeKF_Input& i)
{
    MRPT_START

    // Add to the WorldModel
    {
        worldmodel_->entities_lock_for_write();
        mola::RefPose3 root;
        root.timestamp_   = i.timestamp;
        state_.root_kf_id = worldmodel_->entity_emplace_back(root);
        worldmodel_->entities_unlock_for_write();
    }

    // And add a prior to iSAM2:
    const double prior_std_rot = mrpt::DEG2RAD(0.01);  // [rad]
    const double prior_std_pos = 1e-4;  // [m]

    // mapviz:
    {
        auto lock = lockHelper(vizmap_lock_);
        state_.vizmap.nodes[state_.root_kf_id] =
            mrpt::poses::CPose3D::Identity();
    }

    // Create gtsam value & prior factor (the global reference KF needs to
    // be "fixed" in absolute coordinates SLAM!):
    using namespace gtsam::symbol_shorthand;  // X(), V()

    // We'll insert a prior for the root pose:
    // and also for the new_id , but that's done indirectly via addFactor()
    // (see below).
    state_.kf_has_value.insert(state_.root_kf_id);
    mola2gtsam_register_new_kf(state_.root_kf_id);
    // Don't call state_.updateLastCreatedKF() for ROOT, since it's not an
    // actual KF, just a reference of coordinates.
    // state_.last_created_kf_id = state_.root_kf_id;

    // Next, we can create the actual Key-frame (with observations, etc.)
    // relative to the global root frame.
    // We will add a strong "fix" factor between this new KF and the root,
    // so it shows up attached to the origin of coordinates.
    auto new_id = internal_addKeyFrame_Regular(i);

    internal_add_gtsam_prior_vel(new_id);

    switch (params_.state_vector)
    {
        case StateVectorType::SE2:
            THROW_EXCEPTION("to do!");
            break;
        case StateVectorType::DynSE2:
            THROW_EXCEPTION("to do!");
            break;

        case StateVectorType::DynSE3:
        case StateVectorType::SE3:
        {
            // Index map: MOLA worldmodel <-> gtsam
            const auto key_root    = X(state_.root_kf_id);
            const auto key_kf_pose = X(new_id);

            // Rot, Pos:
            const auto state0 = gtsam::Pose3::identity();

            const gtsam::Vector6 diag_stds =
                (gtsam::Vector6() << prior_std_rot * gtsam::ones(3, 1),
                 prior_std_pos * gtsam::ones(3, 1))
                    .finished();

            // RefPose:
            state_.newvalues.insert(key_root, state0);
            state_.newfactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                key_root, state0,
                gtsam::noiseModel::Diagonal::Sigmas(diag_stds));
            // First actual KeyFrame:
            state_.newvalues.insert(key_kf_pose, state0);

            break;
        };
        default:
            THROW_EXCEPTION("Unhandled state vector type");
    }

    {
        mola::FactorRelativePose3 f(
            state_.root_kf_id, new_id, mrpt::math::TPose3D::Identity());
        f.noise_model_diag_xyz_ = 0.1;
        f.noise_model_diag_rot_ = mrpt::DEG2RAD(0.1);

        this->addFactor(f);
    }

    mola2gtsam_register_new_kf(new_id);

    MRPT_LOG_DEBUG_STREAM("updateLastCreatedKF: " << new_id);
    state_.updateLastCreatedKF(new_id);

    if (!state_.active_imu_factors.empty())
    {
        // state_.newvalues: B() & V() already created in
        // internal_addKeyFrame_Regular() above. Just add the prior factors:

        // Bias prior
        state_.newfactors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
            B(new_id), imu.priorImuBias, imu.biasNoiseModel));

        // Velocity prior - assume stationary
        state_.newfactors.add(gtsam::PriorFactor<gtsam::Vector3>(
            V(new_id), gtsam::Vector3(0, 0, 0), imu.velocityNoiseModel));
    }

    return new_id;

    MRPT_END
}

// isam2_lock_ is locked from the caller site.
mola::id_t ASLAM_gtsam::internal_addKeyFrame_Regular(const ProposeKF_Input& i)
{
    MRPT_START

    using namespace gtsam::symbol_shorthand;  // X(), V()

    // Do we already have a KF for this timestamp?
    if (auto it_kf = state_.time2kf.find(i.timestamp);
        it_kf != state_.time2kf.end())
    {
        // Yes: return it since we cannot have two KFs for the same
        // timestamp.
        return it_kf->second;
    }

    MRPT_TODO("refactor this to avoid code duplication -> template?");
    mola::Entity new_ent;
    switch (params_.state_vector)
    {
        case StateVectorType::SE3:
        {
            mola::RelPose3KF new_kf;
            new_kf.base_id_   = state_.root_kf_id;
            new_kf.timestamp_ = i.timestamp;
            // Copy the raw observations (shallow copy):
            if (i.observations)
                new_kf.raw_observations_ =
                    mrpt::obs::CSensoryFrame::Create(i.observations.value());
            new_ent = std::move(new_kf);
        }
        break;
        case StateVectorType::DynSE3:
        {
            RelDynPose3KF new_kf;
            new_kf.base_id_   = state_.root_kf_id;
            new_kf.timestamp_ = i.timestamp;
            // Copy the raw observations (shallow copy):
            if (i.observations)
                new_kf.raw_observations_ =
                    mrpt::obs::CSensoryFrame::Create(i.observations.value());
            new_ent = std::move(new_kf);
        }
        break;

        default:
            THROW_EXCEPTION("TODO");
            break;
    };

    // Add to the WorldModel:
    worldmodel_->entities_lock_for_write();
    const auto new_kf_id = worldmodel_->entity_emplace_back(std::move(new_ent));
    worldmodel_->entities_unlock_for_write();

    // Add to timestamp register:
    state_.time2kf[i.timestamp] = new_kf_id;

    const gtsam::Key key_kf_pose = X(new_kf_id), key_kf_vel = V(new_kf_id);

    // Let's use the value of the last KF as a gross initial value,
    // in case no other Factor makes things easier:
    // Dont add this KF to the list `kf_has_value`, since it's created, but
    // doesn't have an actual "quality" initial value.
    bool init_value_added = false;
    if (state_.last_created_kf_id != mola::INVALID_ID)
    {
        const gtsam::Key last_pose_key =
            state_.mola2gtsam.at(state_.last_created_kf_id)[KF_KEY_POSE];

        bool have_prev_value = false;
        auto it_prev         = state_.last_values.find(last_pose_key);
        if (it_prev != state_.last_values.end())
            have_prev_value = true;
        else if (it_prev = state_.newvalues.find(last_pose_key);
                 it_prev != state_.newvalues.end())
            have_prev_value = true;

        if (have_prev_value)
        {
            init_value_added = true;

            // Init values:
            switch (params_.state_vector)
            {
                case StateVectorType::SE3:
                {
                    state_.newvalues.insert(key_kf_pose, it_prev->value);
                }
                break;
                case StateVectorType::DynSE3:
                {
                    state_.newvalues.insert(key_kf_pose, it_prev->value);

                    const gtsam::Vector3 vel0 = gtsam::Z_3x1;
                    if (!state_.newvalues.exists(key_kf_vel))
                        state_.newvalues.insert(key_kf_vel, vel0);
                }
                break;
                case StateVectorType::DynSE2:
                {
                    THROW_EXCEPTION("TODO");
                }
                break;

                default:
                    THROW_EXCEPTION("TODO");
                    break;
            };
        }
    }

    if (!state_.active_imu_factors.empty())
    {
        if (!state_.newvalues.exists(V(new_kf_id)))
            state_.newvalues.insert(V(new_kf_id), gtsam::Vector3(0, 0, 0));

        if (!state_.newvalues.exists(B(new_kf_id)))
            state_.newvalues.insert(B(new_kf_id), imu.prevBias);
    }

    if (!init_value_added)
    {
        MRPT_LOG_WARN_STREAM(
            "Creating KF #" << new_kf_id << " without a good initial guess.");
    }

    mola2gtsam_register_new_kf(new_kf_id);

    // Add a dynamics factor, if the time difference between this new KF
    // and the latest one is small enough, and we are not already using an
    // IMU:
    if (state_.last_created_kf_id_tim != INVALID_TIMESTAMP)
    {
        // Only if we dont have an IMU:
        if (state_.active_imu_factors.empty() ||
            state_.former_last_created_kf_id == INVALID_ID)
        {
            const double kf2kf_tim = mrpt::system::timeDifference(
                state_.last_created_kf_id_tim, i.timestamp);
            if (kf2kf_tim < params_.max_interval_between_kfs_for_dynamic_model)
            {
                FactorDynamicsConstVel fDyn(
                    state_.last_created_kf_id, new_kf_id);
                addFactor(fDyn);
            }
        }
        else
        {
            ASSERT_(state_.last_created_kf_id != INVALID_ID);
            ASSERT_(state_.former_last_created_kf_id != INVALID_ID);

            MRPT_LOG_WARN_STREAM(
                "new_kf_id: " << new_kf_id << " last_created_kf_id="
                              << state_.last_created_kf_id
                              << " former_last_created_kf_id:"
                              << state_.former_last_created_kf_id);

            for (const auto pIMU : state_.active_imu_factors)
                pIMU->createIMUFactor(state_.last_created_kf_id, new_kf_id);
        }
    }
    else
    {
        // If we dont have dynamics, add a dynamic prior at least:
        internal_add_gtsam_prior_vel(new_kf_id);
    }

    // This one must be updated here, since it's used in the if() above.
    state_.last_created_kf_id_tim = i.timestamp;
    MRPT_LOG_DEBUG_STREAM("updateLastCreatedKF: " << new_kf_id);
    state_.updateLastCreatedKF(new_kf_id);

    return new_kf_id;

    MRPT_END
}

BackEndBase::ProposeKF_Output ASLAM_gtsam::doAddKeyFrame(
    const ProposeKF_Input& i)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddKeyFrame");
    ProposeKF_Output o;

    MRPT_TODO("Merge KFs too close in time?");

    MRPT_LOG_DEBUG_FMT(
        "Creating new KeyFrame (timestamp=%s)",
        mrpt::system::dateTimeLocalToString(i.timestamp).c_str());

    auto lock = lockHelper(isam2_lock_);

    // If this is the first KF, create an absolute coordinate reference
    // frame in the map:
    if (state_.root_kf_id == INVALID_ID)
    {
        o.new_kf_id = internal_addKeyFrame_Root(i);
        o.success   = true;
        return o;
    }
    else
    {
        // Regular KF:
        o.new_kf_id = internal_addKeyFrame_Regular(i);

        // No need to add anything else to the gtsam graph.
        // A keyframe will be added when the first factor involving that KF
        // is created.

        o.success = true;
        return o;
    }

    MRPT_END
}

void ASLAM_gtsam::doAdvertiseUpdatedLocalization(
    AdvertiseUpdatedLocalization_Input l)
{
    MRPT_START

    ProfilerEntry tleg(profiler_, "doAdvertiseUpdatedLocalization");

    ASSERT_(l.timestamp != INVALID_TIMESTAMP);

    //
    latest_localization_data_mtx_.lock();
    latest_localization_data_ = l;
    latest_localization_data_mtx_.unlock();

    MRPT_LOG_DEBUG_STREAM(
        "AdvertiseUpdatedLocalization: timestamp="
        << mrpt::Clock::toDouble(l.timestamp) << " ref_kf=#" << l.reference_kf
        << " rel_pose=" << l.pose.asString());

    // Insert into trajectory path?
    if (!params_.save_trajectory_file_prefix.empty())
        state_.trajectory[l.timestamp] = l;

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

        // Load "decorations" (OpenGL render objects)
        // from new keyframes, from the WorldModel:
        std::vector<CRenderizable::Ptr> pending_add_to_scene;
        worldmodel_->entities_lock_for_read();
        for (const auto& id_pose : di->vizmap.nodes)
        {
            const mola::id_t id = id_pose.first;
            if (0 == display_state_.kf_checked_decorations.count(id))
            {
                display_state_.kf_checked_decorations.insert(id);
                const auto& annots = worldmodel_->entity_annotations_by_id(id);
                MRPT_TODO("add kernel hdr for standarized annotation names");
                if (const auto it_a = annots.find("render_decoration");
                    it_a != annots.end())
                {
                    const mrpt::serialization::CSerializable::Ptr& o =
                        it_a->second.value();
                    const auto gl_obj = mrpt::ptr_cast<CRenderizable>::from(o);
                    if (gl_obj)
                    {
                        display_state_.kf_decorations[id] = gl_obj;
                        pending_add_to_scene.push_back(gl_obj);
                    }
                }
            }
        }
        worldmodel_->entities_unlock_for_read();

        {  // lock scene
            COpenGLScene::Ptr      scene;
            CDisplayWindow3DLocker lock(*display_, scene);

            // Update scene:
            if (!display_state_.slam_graph_gl)
            {
                auto o = CSetOfObjects::Create();
                o->setName("SLAM_graph");
                display_state_.slam_graph_gl = o;
                scene->insert(o);
            }

            // New KF decorations?
            for (const auto& obj : pending_add_to_scene) scene->insert(obj);

            // Update keyframe decoration poses:
            for (const auto& id_deco : display_state_.kf_decorations)
            {
                const mola::id_t id     = id_deco.first;
                const auto&      gl_ptr = id_deco.second;

                gl_ptr->setPose(di->vizmap.nodes.at(id));
            }

            mrpt::system::TParametersDouble params;
            params["show_ID_labels"]    = 1;
            params["show_ground_grid"]  = 1;
            params["show_edges"]        = 1;
            params["show_node_corners"] = 1;

            display_state_.slam_graph_gl->clear();
            auto gl_graph =
                mrpt::opengl::graph_tools::graph_visualize(di->vizmap, params);
            display_state_.slam_graph_gl->insert(gl_graph);

            // Draw current latest pose:
            latest_localization_data_mtx_.lock();
            const AdvertiseUpdatedLocalization_Input latest_loc =
                latest_localization_data_;
            latest_localization_data_mtx_.unlock();

            if (const auto ref_kf =
                    di->vizmap.nodes.find(latest_loc.reference_kf);
                ref_kf != di->vizmap.nodes.end())
            {
                const auto ref_pose = ref_kf->second;
                const auto abs_pose =
                    ref_pose + mrpt::poses::CPose3D(latest_loc.pose);

                auto gl_cur_pose =
                    mrpt::opengl::stock_objects::CornerXYZSimple(1.5f, 4.0f);

                gl_cur_pose->setPose(abs_pose);
                display_state_.slam_graph_gl->insert(gl_cur_pose);
            }

            // Draw velocities:
            if (1)
            {
                const double vel_time = 0.3;

                auto gl_vels = mrpt::opengl::CSetOfLines::Create();

                for (const auto& v : state_.vizmap_dyn)
                {
                    auto it_p = state_.vizmap.nodes.find(v.first);
                    if (it_p == state_.vizmap.nodes.end()) continue;

                    mrpt::math::TSegment3D sg;
                    sg.point1 = mrpt::math::TPoint3D(it_p->second.asTPose());
                    sg.point2 =
                        sg.point1 + mrpt::math::TPoint3D(
                                        v.second.vx, v.second.vy, v.second.vz) *
                                        vel_time;

                    gl_vels->appendLine(sg);
                }

                gl_vels->setLineWidth(4.0f);
                gl_vels->setColor_u8(0x00, 0xff, 0x00, 0xc0);  // RGBA

                display_state_.slam_graph_gl->insert(gl_vels);
            }

            if (di->current_tim != INVALID_TIMESTAMP)
            {
                const auto s = mrpt::format(
                    "Timestamp: %s",
                    mrpt::system::dateTimeLocalToString(di->current_tim)
                        .c_str());
                display_->addTextMessage(
                    5, 5, s, mrpt::img::TColorf(1, 1, 1), "sans", 11,
                    mrpt::opengl::NICE, 0 /*id*/, 1.5, 0.1, true /*shadow*/);
            }
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

void ASLAM_gtsam::mola2gtsam_register_new_kf(const mola::id_t kf_id)
{
    using namespace gtsam::symbol_shorthand;  // X(), V()

    const auto pose_key = X(kf_id);
    const auto vel_key  = V(kf_id);

    auto lock = lockHelper(keys_map_lock_);

    state_.mola2gtsam[kf_id][KF_KEY_POSE] = pose_key;
    state_.mola2gtsam[kf_id][KF_KEY_VEL]  = vel_key;

    state_.gtsam2mola[KF_KEY_POSE][pose_key] = kf_id;
    state_.gtsam2mola[KF_KEY_VEL][vel_key]   = kf_id;
}

ASLAM_gtsam::whole_path_t ASLAM_gtsam::reconstruct_whole_path() const
{
    MRPT_START

    using mrpt::math::TPose3D;
    using mrpt::math::TTwist3D;
    using mrpt::poses::CPose3D;
    using mrpt::poses::CPose3DInterpolator;

    // Reconstruct the optimized vehicle/robot path:
    // Keyframes already are optimized in the world-model.
    // non-keyframes are stored in "state_.trajectory". See docs of that
    // member.

    // 1st pass: key-frames:
    whole_path_t path;

    worldmodel_->entities_lock_for_read();

    const auto lst_kf_ids = worldmodel_->entity_all_ids();
    for (auto id : lst_kf_ids)
    {
        const auto&    e   = worldmodel_->entity_by_id(id);
        const TPose3D  p   = mola::entity_get_pose(e);
        const TTwist3D tw  = mola::entity_get_twist(e);
        const auto     tim = mola::entity_get_timestamp(e);

        path.id2time[id]  = tim;
        path.time2id[tim] = id;
        path.poses.insert(tim, p);
        path.twists[tim] = tw;
    }

    worldmodel_->entities_unlock_for_read();

    // 2nd pass: non key-frames:
    for (const auto& localiz_pts : state_.trajectory)
    {
        const auto                                tim = localiz_pts.first;
        const AdvertiseUpdatedLocalization_Input& li  = localiz_pts.second;

        // if we have a KF for this timestamp, the KF is more trustful since
        // it underwent optimization:
        if (path.poses.find(tim) != path.poses.end()) continue;

        // otherwise, we have a non KF: compute its pose wrt some other KF:
        TPose3D abs_pose;
        path.poses.at(path.id2time.at(li.reference_kf))
            .composePose(li.pose, abs_pose);

        path.poses.insert(tim, abs_pose);
    }

    return path;
    MRPT_END
}

void ASLAM_gtsam::onQuit()
{
    MRPT_START

    using mrpt::math::TPose3D;
    using mrpt::poses::CPose3D;
    using mrpt::poses::CPose3DInterpolator;
    using namespace std::string_literals;

    // save path?
    if (!params_.save_trajectory_file_prefix.empty())
    {
        const auto path = reconstruct_whole_path();

        // Save in MRPT CPose3DInterpolator format:
        const auto filp = params_.save_trajectory_file_prefix + "_path.txt"s;
        MRPT_LOG_WARN_STREAM("Saving final estimated poses to: " << filp);
        path.poses.saveToTextFile(filp);

        // Save twist too:
        const auto filt = params_.save_trajectory_file_prefix + "_twists.txt"s;
        MRPT_LOG_WARN_STREAM("Saving final estimated twists to: " << filt);
        std::ofstream f(filt);
        if (f.is_open())
        {
            f << "% timestamp vx vy vz wx wy wz\n";
            for (const auto& d : path.twists)
                f << mrpt::format(
                    "%.06f %18.04f %18.04f %18.04f %18.04f %18.04f "
                    "%18.04f\n",
                    mrpt::Clock::toDouble(d.first), d.second.vx, d.second.vy,
                    d.second.vz, d.second.wx, d.second.wy, d.second.wz);
        }

    }  // end save path

    MRPT_END
}
void ASLAM_gtsam::internal_add_gtsam_prior_vel(const mola::id_t kf_id)
{
    using namespace gtsam::symbol_shorthand;  // X(), V()

    const double prior_std_vel = 0.5;  // [m/s]

    const auto key_kf_vel = V(kf_id);

    switch (params_.state_vector)
    {
        case StateVectorType::SE2:
            // Nothing to do
            break;
        case StateVectorType::DynSE2:
            THROW_EXCEPTION("to do!");
            break;

        case StateVectorType::DynSE3:
        {
            const gtsam::Vector3 vel0     = gtsam::Z_3x1;
            const gtsam::Vector3 vel_stds = prior_std_vel * gtsam::ones(3, 1);
            // First actual KeyFrame:
            if (!state_.newvalues.exists(key_kf_vel))
                state_.newvalues.insert(key_kf_vel, vel0);
            else
                state_.newvalues.update(key_kf_vel, vel0);

            state_.newfactors
                .emplace_shared<gtsam::PriorFactor<gtsam::Velocity3>>(
                    key_kf_vel, vel0,
                    gtsam::noiseModel::Diagonal::Sigmas(vel_stds));
        }
        case StateVectorType::SE3:
            // Nothing to do
            break;
        default:
            THROW_EXCEPTION("Unhandled state vector type");
    }
}

void ASLAM_gtsam::onSmartFactorChanged(
    mola::fid_t id, const mola::FactorBase* f)
{
    MRPT_START

    MRPT_TODO("Refactor in a more elegant way?");

    // NOTE: The caller must use the one-call slam_lock() instead of us doing
    // the lock for every single observation. auto lock =
    // lockHelper(isam2_lock_);

    using namespace gtsam::symbol_shorthand;  // X()

    if (const auto* fstptr =
            dynamic_cast<const mola::SmartFactorStereoProjectionPose*>(f);
        fstptr != nullptr)
    {
        const auto& fst = *fstptr;

        const auto& all_obs = fst.allObservations();
        ASSERT_(all_obs.size() > 0);

        // We have been called because of the last entry in the list, so
        // process it:
        const auto& last_obs = *all_obs.rbegin();

        const auto sp = gtsam::StereoPoint2(
            last_obs.pixel_coords.x_left, last_obs.pixel_coords.x_right,
            last_obs.pixel_coords.y);

        const gtsam::Key pose_key = X(last_obs.observing_kf);

#if 0
        MRPT_LOG_DEBUG_STREAM(
            "SmartFactorStereoProjectionPose.add(): fid="
            << id << " from kf id#" << last_obs.observing_kf);
#endif

        // Notify iSAM2 that this factor now has new affected Keys:
        // Only if the factor *already* existed:
        const auto& mola2gtsam_ids = state_.stereo_factors.ids.mola2gtsam;
        if (mola2gtsam_ids.count(id) != 0)
        {
            const auto gtsam_factor_id = mola2gtsam_ids.at(id);
            state_.changedSmartFactors[gtsam_factor_id].insert(pose_key);
        }

        // Actually add observation to factor:
        state_.stereo_factors.factors.at(id)->add(
            sp, pose_key, state_.stereo_factors.camera_K);
    }
    else if (const auto* fstptr = dynamic_cast<const mola::SmartFactorIMU*>(f);
             fstptr != nullptr)
    {
        const auto& fimu = *fstptr;
        switch (fimu.new_state_)
        {
            case SmartFactorIMU::NewState::MEASURE:
            {
                const gtsam::Vector3 acc(fimu.ax_, fimu.ay_, fimu.az_);
                const gtsam::Vector3 gyr(fimu.wx_, fimu.wy_, fimu.wz_);
                imu.preintegrated->integrateMeasurement(acc, gyr, fimu.dt_);
            }
            break;
            case SmartFactorIMU::NewState::FACTOR:
            {
                const auto kf_m1  = fimu.prev_pose_kf_;
                const auto kf_cur = fimu.new_pose_kf_;

                // We cannot create an IMU factor for t=0
                if (kf_m1 == mola::INVALID_ID) break;
                ASSERT_(kf_cur != mola::INVALID_ID);

                MRPT_LOG_DEBUG_STREAM(
                    "Creating IMU factor for KFs: #" << kf_m1 << " ==> "
                                                     << kf_cur);

                using gtsam::symbol_shorthand::B;
                using gtsam::symbol_shorthand::V;
                using gtsam::symbol_shorthand::X;

                MRPT_TODO("Use dynamically-created instead of `imu`");

                gtsam::CombinedImuFactor imuFactor(
                    X(kf_m1), V(kf_m1), X(kf_cur), V(kf_m1), B(kf_m1),
                    B(kf_cur), *imu.preintegrated);
                imuFactor.print("new IMU factor:");
                state_.newfactors.add(imuFactor);

                // Reset IMU integrator:
                imu.propState =
                    imu.preintegrated->predict(imu.prevState, imu.prevBias);
                imu.prevState = gtsam::NavState(
                    state_.at_new_or_last_values<gtsam::Pose3>(X(kf_cur)),
                    state_.at_new_or_last_values<gtsam::Vector3>(V(kf_cur)));
                imu.prevBias =
                    state_.at_new_or_last_values<gtsam::imuBias::ConstantBias>(
                        B(kf_cur));
                imu.preintegrated->resetIntegrationAndSetBias(imu.prevBias);
            }
            break;

            default:
                THROW_EXCEPTION(
                    "onChange() called for IMU factor but no new data "
                    "found");
        };
    }

    MRPT_END
}

mola::id_t ASLAM_gtsam::temp_createStereoCamera(
    const mrpt::img::TCamera& left, const mrpt::img::TCamera& right,
    const double baseline)
{
    MRPT_START

    MRPT_TODO("Add into the world-model and get a real entity id");
    mola::id_t cam_K_id = 10000000;

    MRPT_LOG_DEBUG_STREAM(
        "Defining stereo camera: l.fx=" << left.fx() << " r.fx=" << right.fx()
                                        << " l.fy=" << left.fy()
                                        << " r.fy=" << right.fy());
    // Camera parameters
    ASSERT_(std::abs(left.fx() - right.fx()) < 1e-6);
    ASSERT_(std::abs(left.fy() - right.fy()) < 1e-6);
    ASSERT_(std::abs(left.cx() - right.cx()) < 1e-6);
    ASSERT_(std::abs(left.cy() - right.cy()) < 1e-6);

    const double fx = left.fx(), fy = left.fy(), cx = left.cx(), cy = left.cy();

    state_.stereo_factors.camera_K = gtsam::Cal3_S2Stereo::shared_ptr(
        new gtsam::Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline));

    return cam_K_id;

    MRPT_END
}

void ASLAM_gtsam::lock_slam() { isam2_lock_.lock(); }
void ASLAM_gtsam::unlock_slam() { isam2_lock_.unlock(); }

mola::id_t ASLAM_gtsam::temp_createLandmark(
    const mrpt::math::TPoint3D& init_value)
{
    MRPT_START
    //
    worldmodel_->entities_lock_for_write();
    mola::LandmarkPoint3 lm;
    auto                 new_id = worldmodel_->entity_emplace_back(lm);
    worldmodel_->entities_unlock_for_write();

    using namespace gtsam::symbol_shorthand;  // X(), L()

    const gtsam::Key lm_key = L(new_id);

    state_.newvalues.insert(lm_key, toPoint3(init_value));

    return new_id;

    MRPT_END
}
