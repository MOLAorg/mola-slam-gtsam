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
#include <mola-kernel/entities/entities-common.h>
#include <mola-kernel/variant_helper.h>
#include <mola-kernel/yaml_helpers.h>
#include <mola-slam-gtsam/ASLAM_gtsam.h>
#include <mrpt/opengl/CSetOfLines.h>  // TODO: Remove after vizmap module
#include <mrpt/opengl/graph_tools.h>  // TODO: Remove after vizmap module
#include <yaml-cpp/yaml.h>

// GTSAM second:
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Symbol.h>  // X(), V() symbols
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace mola;

template <class T>
class LockHelper
{
   public:
    LockHelper(T* l) : l_{l} { l_->lock(); }
    ~LockHelper()
    {
        if (l_) l_->unlock();
    }

    LockHelper(const LockHelper& o) = delete;
    LockHelper& operator=(const LockHelper& o) = delete;

    LockHelper(LockHelper&& o) : l_{o.l} { o.l = nullptr; }
    LockHelper& operator=(LockHelper&& o)
    {
        l_  = o.l;
        o.l = nullptr;
        return *this;
    }

   private:
    T* l_{nullptr};
};

template <class T>
LockHelper<T> lockHelper(T& t)
{
    return LockHelper<T>(&t);
}

MRPT_TODO("Move all these aux funcs somewhere else");

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

static mrpt::math::TTwist3D toTTwist3D(const gtsam::Velocity3& v)
{
    mrpt::math::TTwist3D t;
    t.vx = v.x();
    t.vy = v.y();
    t.vz = v.z();
    return t;
}

static void updateEntityPose(mola::Entity& e, const gtsam::Pose3& x)
{
    MRPT_TRY_START
    const auto p = toTPose3D(x);

    std::visit(
        overloaded{
            [&](RefPose3&) {
                ASSERTMSG_(
                    p == mrpt::math::TPose3D::Identity(),
                    "RefPose3 cannot be assigned a pose != Identity()");
            },
            [&](RelDynPose3KF& ee) { ee.relpose_value = p; },
            [&](RelPose3& ee) { ee.relpose_value = p; },
            [&](RelPose3KF& ee) { ee.relpose_value = p; },
            []([[maybe_unused]] auto ee) {
                throw std::runtime_error(
                    mrpt::format("[updateEntityPose] Unknown Entity type!"));
            },
        },
        e);
    MRPT_TRY_END
}
static void updateEntityVel(mola::Entity& e, const gtsam::Velocity3& v)
{
    MRPT_TRY_START

    std::visit(
        overloaded{
            [&](RefPose3&) {
                THROW_EXCEPTION("RefPose3 cannot be assigned a velocity");
            },
            [&](RelDynPose3KF& ee) {
                ee.twist_value.vx = v.x();
                ee.twist_value.vy = v.y();
                ee.twist_value.vz = v.z();
            },
            [&](RelPose3& ee) {},
            [&](RelPose3KF& ee) {},
            []([[maybe_unused]] auto ee) {
                throw std::runtime_error(
                    mrpt::format("[updateEntityPose] Unknown Entity type!"));
            },
        },
        e);
    MRPT_TRY_END
}

static mrpt::math::TPose3D getEntityPose(const mola::Entity& e)
{
    MRPT_TRY_START
    //
    mrpt::math::TPose3D ret;
    std::visit(
        overloaded{
            [&](const RefPose3&) { ret = mrpt::math::TPose3D::Identity(); },
            [&](const RelDynPose3KF& ee) { ret = ee.relpose_value; },
            [&](const RelPose3& ee) { ret = ee.relpose_value; },
            [&](const RelPose3KF& ee) { ret = ee.relpose_value; },
            []([[maybe_unused]] auto ee) {
                throw std::runtime_error(
                    mrpt::format("[getEntityPose] Unknown Entity type!"));
            },
        },
        e);

    return ret;
    MRPT_TRY_END
}

static mrpt::Clock::time_point getEntityTimeStamp(const mola::Entity& e)
{
    MRPT_TRY_START
    //
    mrpt::Clock::time_point ret{};
    std::visit(
        overloaded{
            [&](const EntityBase& ee) { ret = ee.timestamp_; },
            [&](const EntityOther& ee) { ret = ee->timestamp_; },
            [](std::monostate) {
                throw std::runtime_error(
                    mrpt::format("[getEntityTimeStamp] Unknown Entity type!"));
            },
        },
        e);

    ASSERT_(ret != INVALID_TIMESTAMP);

    return ret;
    MRPT_TRY_END
}

namespace gtsam
{
/**
 * Factor for constant velocity model between two pairs Pose3+Velocity3
 */
class ConstVelocityFactor
    : public NoiseModelFactor4<Pose3, Velocity3, Pose3, Velocity3>
{
   public:
    //    using T = NavState;

   private:
    using This = ConstVelocityFactor;
    using Base = NoiseModelFactor4<Pose3, Velocity3, Pose3, Velocity3>;

    using Measure = double;

    /** Time between the states key1 & key2 */
    double deltaTime_;

   public:
    // shorthand for a smart pointer to a factor
    using shared_ptr = boost::shared_ptr<ConstVelocityFactor>;

    /** default constructor - only use for serialization */
    ConstVelocityFactor() {}

    /** Constructor.  */
    ConstVelocityFactor(
        Key pose1, Key vel1, Key pose2, Key vel2, const double deltaTime,
        const SharedNoiseModel& model)
        : Base(model, pose1, vel1, pose2, vel2), deltaTime_(deltaTime)
    {
    }

    virtual ~ConstVelocityFactor() override = default;

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(
        const std::string&  s,
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override
    {
        std::cout << s << "ConstVelocityBetweenNavState("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ","
                  << keyFormatter(this->key3()) << ","
                  << keyFormatter(this->key4()) << ")\n";
        traits<double>::Print(deltaTime_, "  deltaTime: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(
        const NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               traits<Measure>::Equals(this->deltaTime_, e->deltaTime_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(
        const Pose3& p1, const Velocity3& v1, const Pose3& p2,
        const Velocity3& v2, boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none) const override
    {
        Vector6 err;
        err.head<3>() = p1.translation() + v1 * deltaTime_ - p2.translation();
        err.tail<3>() = v2 - v1;

        if (H1)
        {
            auto& H1v = H1.value();
            H1v.zeros(6, 6);
            H1v.block<3, 3>(0, 3) = gtsam::I_3x3;
        }
        if (H2)
        {
            auto& H2v = H2.value();
            H2v.resize(6, 3);
            H2v.block<3, 3>(0, 0) = gtsam::I_3x3 * deltaTime_;
            H2v.block<3, 3>(3, 0) = -gtsam::I_3x3;
        }
        if (H3)
        {
            auto& H3v = H3.value();
            H3v.zeros(6, 6);
            H3v.block<3, 3>(0, 3) = -gtsam::I_3x3;
        }
        if (H4)
        {
            auto& H4v = H4.value();
            H4v.resize(6, 3);
            H4v.block<3, 3>(3, 0) = gtsam::I_3x3;
        }

        return err;
    }

    /** return the measured */
    // const Measure& measured() const { return measured_; }

    /** number of variables attached to this factor */
    std::size_t size() const { return 2; }

   private:
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& boost::serialization::make_nvp(
            "RelPoseBetweenNavState",
            boost::serialization::base_object<Base>(*this));
        ar& BOOST_SERIALIZATION_NVP(deltaTime_);
    }

    // Alignment, see
    // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace gtsam

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

    YAML_LOAD_OPT(params_, const_vel_model_std_pos, double);
    YAML_LOAD_OPT(params_, const_vel_model_std_vel, double);

    // Ensure we have access to the worldmodel:
    ASSERT_(worldmodel_);

    MRPT_TODO("Load existing map from world model?");

    // Init iSAM2:
    if (params_.use_incremental_solver)
    {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold   = 0.1;
        parameters.relinearizeSkip        = 2;
        parameters.cacheLinearizedFactors = false;  // true;
        parameters.enableDetailedResults  = false;

        state_.isam2 = std::make_unique<gtsam::ISAM2>(parameters);
    }

    MRPT_END
}
void ASLAM_gtsam::spinOnce()
{
    MRPT_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    gtsam::Values result;

    // Incremental SAM solution:
    gtsam::ISAM2Result isam2_res;

    if (params_.use_incremental_solver)
    {
        auto lock = lockHelper(isam2_lock_);
        if (!state_.newfactors.empty())
        {
            {
                ProfilerEntry tle(profiler_, "spinOnce.isam2_update");
                state_.isam2->update(state_.newfactors, state_.newvalues);
            }

            {
                ProfilerEntry tle(profiler_, "spinOnce.isam2_calcEstimate");
                // result = state_.isam2->calculateEstimate();
                result = state_.isam2->calculateBestEstimate();
            }

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

            // reset accumulators of new slam factors:
            state_.newfactors = gtsam::NonlinearFactorGraph();
            state_.newvalues.clear();
        }
    }
    else
    {
        auto lock = lockHelper(isam2_lock_);
        if (!state_.newfactors.empty())
        {
            ProfilerEntry tle(
                profiler_, "spinOnce.LevenbergMarquardtOptimizer");

            gtsam::LevenbergMarquardtOptimizer optimizer(
                state_.newfactors, state_.newvalues);
            result = optimizer.optimize();
        }

        state_.newvalues = result;
    }

    if (result.size())
    {
        // MRPT_TODO("gtsam Values: add print(ostream) method");
        if (this->isLoggingLevelVisible(mrpt::system::LVL_DEBUG))
            result.print("isam2 result:");

        MRPT_LOG_INFO_STREAM(
            "iSAM2 ran for " << result.size() << " variables.");

        MRPT_TODO("Do it incrementally, so we dont need to send everything");

        auto lk = lockHelper(keys_map_lock_);

        // Send values to the world model:
        worldmodel_->entities_lock_for_write();
        for (auto key_value : result)
        {
            if (auto it_kf = state_.gtsam2mola[KF_KEY_POSE].find(key_value.key);
                it_kf != state_.gtsam2mola[KF_KEY_POSE].end())
            {
                const mola::id_t kf_id   = it_kf->second;
                gtsam::Pose3     kf_pose = key_value.value.cast<gtsam::Pose3>();

                // Dont update the pose of the global reference, fixed to
                // Identity()
                if (kf_id != state_.root_kf_id)
                    updateEntityPose(worldmodel_->entity_by_id(kf_id), kf_pose);

                // mapviz:
                const auto p               = toTPose3D(kf_pose);
                state_.vizmap.nodes[kf_id] = mrpt::poses::CPose3D(p);
            }
            else if (auto it_kf =
                         state_.gtsam2mola[KF_KEY_VEL].find(key_value.key);
                     it_kf != state_.gtsam2mola[KF_KEY_VEL].end())
            {
                const mola::id_t kf_id = it_kf->second;
                gtsam::Velocity3 kf_vel =
                    key_value.value.cast<gtsam::Velocity3>();

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
        auto lock = lockHelper(vizmap_lock_);

        if (state_.last_created_kf_id != mola::INVALID_ID)
        {
            worldmodel_->entities_lock_for_read();
            di->current_tim = getEntityTimeStamp(
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
    const double prior_std_vel = 0.5;  // [m/s]

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
    // and also for the new_id , but that's done indirectly via addFactor() (see
    // below).
    state_.kf_has_value.insert(state_.root_kf_id);

    mola2gtsam_register_new_kf(state_.root_kf_id);

    // Next, we can create the actual Key-frame (with observations, etc.)
    // relative to the global root frame.
    // We will add a strong "fix" factor between this new KF and the root,
    // so it shows up attached to the origin of coordinates.
    auto new_id = internal_addKeyFrame_Regular(i);

    switch (params_.state_vector)
    {
        case StateVectorType::SE2:
            THROW_EXCEPTION("to do!");
            break;
        case StateVectorType::DynSE2:
            THROW_EXCEPTION("to do!");
            break;

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
        }
        break;
        case StateVectorType::DynSE3:
        {
            const auto key_root_pose = X(state_.root_kf_id);
            const auto key_kf_pose   = X(new_id);
            const auto key_kf_vel    = V(new_id);

            // Rot, Pos, Vel:
            const auto           state0 = gtsam::Pose3::identity();
            const gtsam::Vector3 vel0   = gtsam::Z_3x1;

            const gtsam::Vector6 diag_stds =
                (gtsam::Vector6() << prior_std_rot * gtsam::ones(3, 1),
                 prior_std_pos * gtsam::ones(3, 1))
                    .finished();

            const gtsam::Vector3 vel_stds = prior_std_vel * gtsam::ones(3, 1);

            // RefPose:
            state_.newvalues.insert(key_root_pose, state0);
            state_.newfactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                key_root_pose, state0,
                gtsam::noiseModel::Diagonal::Sigmas(diag_stds));
            // First actual KeyFrame:
            state_.newvalues.insert(key_kf_pose, state0);
            state_.newvalues.insert(key_kf_vel, vel0);
            state_.newfactors
                .emplace_shared<gtsam::PriorFactor<gtsam::Velocity3>>(
                    key_kf_vel, vel0,
                    gtsam::noiseModel::Diagonal::Sigmas(vel_stds));
        }
        break;
        default:
            THROW_EXCEPTION("Unhandled state vector type");
    };

    {
        mola::FactorRelativePose3 f(
            state_.root_kf_id, new_id, mrpt::math::TPose3D::Identity());
        f.noise_model_diag_xyz_ = 0.1e-3;
        f.noise_model_diag_rot_ = mrpt::DEG2RAD(0.001);

        this->addFactor(f);
    }

    mola2gtsam_register_new_kf(new_id);
    state_.last_created_kf_id = new_id;

    return new_id;

    MRPT_END
}

// isam2_lock_ is locked from the caller site.
mola::id_t ASLAM_gtsam::internal_addKeyFrame_Regular(const ProposeKF_Input& i)
{
    MRPT_START

    using namespace gtsam::symbol_shorthand;  // X(), V()

    mola::RelPose3KF new_kf;
    new_kf.base_id_   = state_.root_kf_id;
    new_kf.timestamp_ = i.timestamp;

    // Copy the raw observations (shallow copy):
    if (i.observations)
        new_kf.raw_observations_ =
            mrpt::obs::CSensoryFrame::Create(i.observations.value());

    // Add to the WorldModel:
    worldmodel_->entities_lock_for_write();
    const auto new_kf_id = worldmodel_->entity_emplace_back(new_kf);
    worldmodel_->entities_unlock_for_write();

    const gtsam::Key key_kf_pose = X(new_kf_id), key_kf_vel = V(new_kf_id);

    // Let's use the value of the last KF as a gross initial value,
    // in case no other Factor makes things easier:
    // Dont add this KF to the list `kf_has_value`, since it's created, but
    // doesn't have an actual "quality" initial value.
    if (state_.last_created_kf_id != mola::INVALID_ID)
    {
        const gtsam::Key last_pose_key =
            state_.mola2gtsam.at(state_.last_created_kf_id)[KF_KEY_POSE];

        const auto it_prev = state_.newvalues.find(last_pose_key);
        if (it_prev != state_.newvalues.end())
        {
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

    mola2gtsam_register_new_kf(new_kf_id);
    state_.last_created_kf_id = new_kf_id;

    return new_kf_id;

    MRPT_END
}

BackEndBase::ProposeKF_Output ASLAM_gtsam::doAddKeyFrame(
    const ProposeKF_Input& i)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddKeyFrame");
    ProposeKF_Output o;

    MRPT_LOG_DEBUG("Creating new KeyFrame");

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

BackEndBase::AddFactor_Output ASLAM_gtsam::doAddFactor(Factor& newF)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddFactor");
    AddFactor_Output o;

    auto lock = lockHelper(isam2_lock_);

    mola::fid_t fid = INVALID_FID;

    std::visit(
        overloaded{
            [&](const FactorRelativePose3& f) { fid = addFactor(f); },
            [&](const FactorRelativePose3ConstVel& f) { fid = addFactor(f); },
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
    return internal_addFactorRelPose(f, false);
    MRPT_END
}
fid_t ASLAM_gtsam::addFactor(const FactorRelativePose3ConstVel& f)
{
    MRPT_START
    MRPT_LOG_DEBUG("Adding new FactorRelativePose3ConstVel");
    return internal_addFactorRelPose(f.relPoseFactor_, true);
    MRPT_END
}

fid_t ASLAM_gtsam::internal_addFactorRelPose(
    const FactorRelativePose3& f, const bool addDynamicsFactor)
{
    MRPT_START

    // Add to the WorldModel:
    worldmodel_->factors_lock_for_write();
    const fid_t new_fid = worldmodel_->factor_push_back(f);
    worldmodel_->factors_unlock_for_write();

    // Relative pose:
    const gtsam::Pose3 measure = toPose3(f.rel_pose_);

    // Initial estimation of the new KF:
    mrpt::math::TPose3D to_pose_est;

    worldmodel_->entities_lock_for_write();

    const auto& kf_from = worldmodel_->entity_by_id(f.from_kf_);
    const auto& kf_to   = worldmodel_->entity_by_id(f.to_kf_);

    const auto from_tim      = getEntityTimeStamp(kf_from);
    const auto from_pose_est = getEntityPose(kf_from);
    const auto to_tim        = getEntityTimeStamp(kf_to);

    from_pose_est.composePose(f.rel_pose_, to_pose_est);

    // Store the result just in case we need it as a quick guess in next
    // factors, before running the actual optimizer:
    // Dont update the pose of the global reference, fixed to Identity()
    if (f.to_kf_ != state_.root_kf_id)
        updateEntityPose(
            worldmodel_->entity_by_id(f.to_kf_), toPose3(to_pose_est));

    worldmodel_->entities_unlock_for_write();

    // mapviz:
    {
        auto lock = lockHelper(vizmap_lock_);

        state_.vizmap.nodes[f.to_kf_] = mrpt::poses::CPose3D(to_pose_est);
        state_.vizmap.insertEdgeAtEnd(
            f.from_kf_, f.to_kf_, mrpt::poses::CPose3D(to_pose_est));
    }

    // Measure noise model:
    MRPT_TODO("handle custom noise matrix from input factor");

    const double std_xyz = f.noise_model_diag_xyz_;
    const double std_ang = f.noise_model_diag_rot_;

    auto noise_relpose = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << std_ang, std_ang, std_ang, std_xyz, std_xyz,
         std_xyz)
            .finished());

    const auto to_pose_key   = state_.mola2gtsam.at(f.to_kf_)[KF_KEY_POSE];
    const auto from_pose_key = state_.mola2gtsam.at(f.from_kf_)[KF_KEY_POSE];
    // (vel keys may be used or not; declare here for convenience anyway)
    const auto to_vel_key   = state_.mola2gtsam.at(f.to_kf_)[KF_KEY_VEL];
    const auto from_vel_key = state_.mola2gtsam.at(f.from_kf_)[KF_KEY_VEL];

    // Add to list of initial guess (if not done already with a former
    // factor):
    if (state_.kf_has_value.count(f.to_kf_) == 0)
    {
        const gtsam::Pose3 p_to_pose_est = toPose3(to_pose_est);
        if (!state_.newvalues.exists(to_pose_key))
            state_.newvalues.insert(to_pose_key, p_to_pose_est);
        else
            state_.newvalues.update(to_pose_key, p_to_pose_est);

        // Init vel (if applicable):
        switch (params_.state_vector)
        {
            case StateVectorType::DynSE3:
            {
                const gtsam::Vector3 vel0 = gtsam::Z_3x1;
                if (!state_.newvalues.exists(to_vel_key))
                    state_.newvalues.insert(to_vel_key, vel0);
                else
                    state_.newvalues.update(to_vel_key, vel0);
            }
            break;
            case StateVectorType::DynSE2:
            {
                THROW_EXCEPTION("TODO");
            }
            break;

            default:
                break;
        };

        state_.kf_has_value.insert(f.to_kf_);
    }

    // Add relative pose factor:
    switch (params_.state_vector)
    {
        case StateVectorType::SE2:
            THROW_EXCEPTION("to do!");
            break;
        case StateVectorType::DynSE2:
            THROW_EXCEPTION("to do!");
            break;

        case StateVectorType::SE3:
        case StateVectorType::DynSE3:
            state_.newfactors
                .emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                    from_pose_key, to_pose_key, measure, noise_relpose);
            break;
        default:
            THROW_EXCEPTION("Unhandled state vector type");
    };

    if (addDynamicsFactor && params_.state_vector == StateVectorType::DynSE3)
    {
        const double dt = mrpt::system::timeDifference(from_tim, to_tim);
        ASSERT_(dt > 0);

        // 0-2: uncertainty in velocity vector (constant velocity
        // assumption)
        // TODO: 3-5 ditto, rotvel

        // errors in constant vel:
        const double std_pos = params_.const_vel_model_std_pos;
        const double std_vel = params_.const_vel_model_std_vel;

        const gtsam::Vector6 diag_stds = (gtsam::Vector6() << std_pos, std_pos,
                                          std_pos, std_vel, std_vel, std_vel)
                                             .finished();

        auto noise_velModel = gtsam::noiseModel::Diagonal::Sigmas(diag_stds);

        if (dt > 10.0)
        {
            MRPT_LOG_WARN_FMT(
                "A constant-time velocity factor has been added for KFs "
                "too far-away in time: dT=%.03f s. Adding it, anyway, as "
                "requested.",
                dt);
        }

        state_.newfactors.emplace_shared<gtsam::ConstVelocityFactor>(
            from_pose_key, from_vel_key, to_pose_key, to_vel_key, dt,
            noise_velModel);
    }

    return new_fid;

    MRPT_END
}

void ASLAM_gtsam::doAdvertiseUpdatedLocalization(
    AdvertiseUpdatedLocalization_Input l)
{
    MRPT_START

    ProfilerEntry tleg(profiler_, "doAdvertiseUpdatedLocalization");

    ASSERT_(l.timestamp != INVALID_TIMESTAMP);

    MRPT_LOG_DEBUG_STREAM(
        "AdvertiseUpdatedLocalization: timestamp="
        << mrpt::Clock::toDouble(l.timestamp) << " ref_kf=#" << l.reference_kf
        << " rel_pose=" << l.pose.asString());

    // Insert into trajectory path?
    if (!params_.save_trajectory_file_prefix.empty())
        state_.trajectory[l.timestamp] = l;

    MRPT_TODO("notify to all modules subscribed to localization updates");

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

                scene->insert(gl_vels);
            }

            scene->insert(gl_graph);

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

mrpt::poses::CPose3DInterpolator ASLAM_gtsam::reconstruct_whole_path() const
{
    MRPT_START

    using mrpt::math::TPose3D;
    using mrpt::poses::CPose3D;
    using mrpt::poses::CPose3DInterpolator;

    // Reconstruct the optimized vehicle/robot path:
    // Keyframes already are optimized in the world-model.
    // non-keyframes are stored in "state_.trajectory". See docs of that member.

    // 1st pass: key-frames:
    CPose3DInterpolator                                 path;
    mola::fast_map<mola::id_t, mrpt::Clock::time_point> id2time;

    worldmodel_->entities_lock_for_read();

    const auto lst_kf_ids = worldmodel_->entity_all_ids();
    for (auto id : lst_kf_ids)
    {
        const auto&   e   = worldmodel_->entity_by_id(id);
        const TPose3D p   = getEntityPose(e);
        const auto    tim = getEntityTimeStamp(e);

        id2time[id] = tim;
        path.insert(tim, p);
        std::fixed(std::cout);
    }

    worldmodel_->entities_unlock_for_read();

    // 2nd pass: non key-frames:
    for (const auto& localiz_pts : state_.trajectory)
    {
        const auto                                tim = localiz_pts.first;
        const AdvertiseUpdatedLocalization_Input& li  = localiz_pts.second;

        // if we have a KF for this timestamp, the KF is more trustful since it
        // underwent optimization:
        if (path.find(tim) != path.end()) continue;

        // otherwise, we have a non KF: compute its pose wrt some other KF:
        TPose3D abs_pose;
        path.at(id2time.at(li.reference_kf)).composePose(li.pose, abs_pose);

        path.insert(tim, abs_pose);
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
        const CPose3DInterpolator path = reconstruct_whole_path();

        // Save in MRPT CPose3DInterpolator format:
        const auto fil = params_.save_trajectory_file_prefix + "_path.txt"s;
        MRPT_LOG_WARN_STREAM("Saving final estimated trajectory to: " << fil);

        path.saveToTextFile(fil);

    }  // end save path

    MRPT_END
}
