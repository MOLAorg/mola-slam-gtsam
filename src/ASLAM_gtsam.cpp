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
//#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace mola;

static const bool SAVE_KITTI_PATH_FILE = true;

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

static gtsam::NavState toNavState(const mrpt::math::TPose3D& p)
{
    gtsam::Vector3  vel(0, 0, 0);
    gtsam::NavState ret(toPose3(p), vel);
    return ret;
}

static mrpt::math::TPose3D toTPose3D(const gtsam::Pose3& p)
{
    const auto HM = p.matrix();
    const auto H  = mrpt::math::CMatrixDouble44(HM);
    return mrpt::poses::CPose3D(H).asTPose();
}

static mrpt::math::TTwist3D toTTwist3D(const gtsam::NavState& x)
{
    mrpt::math::TTwist3D t;
    t.vx = x.velocity().x();
    t.vy = x.velocity().y();
    t.vz = x.velocity().z();
    // t.wx = x. ...
    return t;
}

static void updateEntityPose(mola::Entity& e, const gtsam::NavState& x)
{
    MRPT_TRY_START
    const auto p = toTPose3D(x.pose());

    std::visit(
        overloaded{
            [&](RefPose3&) {
                ASSERTMSG_(
                    p == mrpt::math::TPose3D::Identity(),
                    "RefPose3 cannot be assigned a pose != Identity()");
            },
            [&](RelDynPose3KF& ee) {
                ee.relpose_value  = p;
                ee.twist_value.vx = x.velocity().x();
                ee.twist_value.vy = x.velocity().y();
                ee.twist_value.vz = x.velocity().z();
            },
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

static void updateEntityPose(mola::Entity& e, const mrpt::math::TPose3D& p)
{
    gtsam::NavState x(toPose3(p), gtsam::Vector3::Zero());
    updateEntityPose(e, x);
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
    mrpt::Clock::time_point ret;
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

    return ret;
    MRPT_TRY_END
}

namespace gtsam
{
/**
 * Factor for relative SE(3) observations between NavState's
 */
class RelPoseBetweenNavState : public NoiseModelFactor2<NavState, NavState>
{
   public:
    using T = NavState;

   private:
    using This = RelPoseBetweenNavState;
    using Base = NoiseModelFactor2<NavState, NavState>;

    using Measure = Pose3;

    /** The measurement */
    Measure measured_;

   public:
    // shorthand for a smart pointer to a factor
    using shared_ptr = boost::shared_ptr<RelPoseBetweenNavState>;

    /** default constructor - only use for serialization */
    RelPoseBetweenNavState() {}

    /** Constructor.  */
    RelPoseBetweenNavState(
        Key key1, Key key2, const Measure& relPose2wrt1,
        const SharedNoiseModel& model)
        : Base(model, key1, key2), measured_(relPose2wrt1)
    {
    }

    virtual ~RelPoseBetweenNavState() override = default;

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
        std::cout << s << "RelPoseBetweenNavState("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ")\n";
        traits<Measure>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(
        const NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               traits<Measure>::Equals(this->measured_, e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(
        const NavState& p1, const NavState& p2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const override
    {
        Matrix H1_p, H2_p;
        // h(x): relative pose observation:
        const Pose3 hx =
            traits<Pose3>::Between(p1.pose(), p2.pose(), H1_p, H2_p);

        Vector6 err;
        // manifold equivalent of h(x)-z -> log(z,h(x))
#ifdef SLOW_BUT_CORRECT_BETWEENFACTOR
        using Hlocal = typename traits<Pose3>::ChartJacobian::Jacobian;

        Vector rval = traits<Pose3>::Local(
            measured_, hx, boost::none, (H1 || H2) ? &Hlocal : nullptr);
        if (H1) *H1_1 = Hlocal * (*H1);
        if (H2) *H1_2 = Hlocal * (*H2);
        err = rval;
#else
        err = traits<Pose3>::Local(measured_, hx);
#endif

        if (H1)
        {
            auto& H1v = H1.value();
            H1v.zeros(6, 9);
            H1v.block<6, 6>(0, 0) = H1_p;
        }
        if (H2)
        {
            auto& H2v = H2.value();
            H2v.zeros(6, 9);
            H2v.block<6, 6>(0, 0) = H2_p;
        }

        return err;
    }

    /** return the measured */
    const Measure& measured() const { return measured_; }

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
        ar& BOOST_SERIALIZATION_NVP(measured_);
    }

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Factor for constant velocity model between NavState's
 */
class ConstVelocityBetweenNavState
    : public NoiseModelFactor2<NavState, NavState>
{
   public:
    using T = NavState;

   private:
    using This = ConstVelocityBetweenNavState;
    using Base = NoiseModelFactor2<NavState, NavState>;

    using Measure = double;

    /** Time between the states key1 & key2 */
    double deltaTime_;

   public:
    // shorthand for a smart pointer to a factor
    using shared_ptr = boost::shared_ptr<ConstVelocityBetweenNavState>;

    /** default constructor - only use for serialization */
    ConstVelocityBetweenNavState() {}

    /** Constructor.  */
    ConstVelocityBetweenNavState(
        Key key1, Key key2, const double deltaTime,
        const SharedNoiseModel& model)
        : Base(model, key1, key2), deltaTime_(deltaTime)
    {
    }

    virtual ~ConstVelocityBetweenNavState() override = default;

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
                  << keyFormatter(this->key2()) << ")\n";
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
        const NavState& p1, const NavState& p2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const override
    {
        // h2(x): constant velocity model
        const Vector6 err =
            (Vector6() << Vector3(
                 p1.position() + deltaTime_ * p1.velocity() - p2.position()),
             p1.velocity() - p2.velocity())
                .finished();

        // Stack error vectors & Jacobians:
        if (H1)
        {
            auto& H1v = H1.value();
            H1v.zeros(6, 9);
            H1v.block<3, 3>(0, 3) = gtsam::I_3x3;
            H1v.block<3, 3>(0, 6) = deltaTime_ * gtsam::I_3x3;

            H1v.block<3, 3>(3, 6) = gtsam::I_3x3;
        }
        if (H2)
        {
            auto& H2v = H2.value();
            H2v.zeros(6, 9);
            H2v.block<3, 3>(0, 0) = -gtsam::I_3x3;

            H2v.block<3, 3>(3, 6) = -gtsam::I_3x3;
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
        // MRPT_TODO("gtsam Values: add print(ostream) method");
        // result.print("isam2 result:");
        MRPT_LOG_INFO_STREAM(
            "iSAM2 ran for " << result.size() << " variables.");

        MRPT_TODO("Do it incrementally, so we dont need to send everything");
        // Send values to the world model:
        worldmodel_->entities_lock();
        for (auto key_value : result)
        {
            const mola::id_t kf_id    = key_value.key;
            gtsam::NavState  kf_navst = key_value.value.cast<gtsam::NavState>();

            // Dont update the pose of the global reference, fixed to Identity()
            if (kf_id != state_.root_kf_id)
                updateEntityPose(worldmodel_->entity_by_id(kf_id), kf_navst);

            // mapviz:
            const auto p               = toTPose3D(kf_navst.pose());
            state_.vizmap.nodes[kf_id] = mrpt::poses::CPose3D(p);
            state_.vizmap_dyn[kf_id]   = toTTwist3D(kf_navst);

            // MRPT_LOG_DEBUG_STREAM("KF#" << kf_id << ": " << p.asString());
        }
        worldmodel_->entities_unlock();
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
        std::lock_guard<decltype(vizmap_lock_)> lock(vizmap_lock_);
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
        // Add to the WorldModel
        {
            worldmodel_->entities_lock();
            mola::RefPose3 root;
            root.timestamp_   = i.timestamp;
            state_.root_kf_id = worldmodel_->entity_emplace_back(root);
            worldmodel_->entities_unlock();
        }

        // And add a prior to iSAM2:
        const double prior_std_rot = mrpt::DEG2RAD(0.01);  // [rad]
        const double prior_std_pos = 1e-4;  // [m]
        const double prior_std_vel = 0.5;  // [m/s]

        gtsam::Vector diag_stds(9);

        // rotation:
        diag_stds(0) = diag_stds(1) = diag_stds(2) = prior_std_rot;
        // position:
        diag_stds(3) = diag_stds(4) = diag_stds(5) = prior_std_pos;
        // velocity:
        diag_stds(6) = diag_stds(7) = diag_stds(8) = prior_std_vel;

        auto priorModel = gtsam::noiseModel::Diagonal::Sigmas(diag_stds);

        {
            std::lock_guard<decltype(isam2_lock_)> lock(isam2_lock_);

            state_.kf_has_value.insert(state_.root_kf_id);

            const gtsam::Rot3 prior_rotation =
                gtsam::Rot3(Eigen::Matrix3d::Identity());
            const gtsam::Vector3 prior_pos = gtsam::Vector3(0, 0, 0);
            const gtsam::Vector3 prior_vel = gtsam::Vector3(0, 0, 0);

            gtsam::NavState state0(prior_rotation, prior_pos, prior_vel);

            state_.newvalues.insert(state_.root_kf_id, state0);
            state_.newfactors
                .emplace_shared<gtsam::PriorFactor<gtsam::NavState>>(
                    state_.root_kf_id, state0, priorModel);
        }

        // mapviz:
        {
            std::lock_guard<decltype(vizmap_lock_)> lock(vizmap_lock_);
            state_.vizmap.nodes[state_.root_kf_id] =
                mrpt::poses::CPose3D::Identity();
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
        new_kf.base_id_   = state_.root_kf_id;
        new_kf.timestamp_ = i.timestamp;

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
    worldmodel_->factors_lock();
    const fid_t new_fid = worldmodel_->factor_push_back(f);
    worldmodel_->factors_unlock();

    // Relative pose:
    const gtsam::Pose3 measure = toPose3(f.rel_pose_);

    // Initial estimation of the new KF:
    mrpt::math::TPose3D to_pose_est;

    worldmodel_->entities_lock();

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
        updateEntityPose(worldmodel_->entity_by_id(f.to_kf_), to_pose_est);

    worldmodel_->entities_unlock();

    // mapviz:
    {
        std::lock_guard<decltype(vizmap_lock_)> lock(vizmap_lock_);

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

    {
        std::lock_guard<decltype(isam2_lock_)> lock(isam2_lock_);

        // Add to list of initial guess (if not done already with a former
        // factor):
        if (state_.kf_has_value.count(f.to_kf_) == 0)
        {
            state_.kf_has_value.insert(f.to_kf_);
            state_.newvalues.insert(f.to_kf_, toNavState(to_pose_est));
        }

        state_.newfactors.emplace_shared<gtsam::RelPoseBetweenNavState>(
            f.from_kf_, f.to_kf_, measure, noise_relpose);

        if (addDynamicsFactor)
        {
            const double dt = mrpt::system::timeDifference(from_tim, to_tim);

            // 0-2: uncertainty in velocity vector (constant velocity
            // assumption)
            // TODO: 3-5 ditto, rotvel

            // error in position:
            const double noise_std_pos = 1.0 + 0.1 * dt;
            // error in constant vel:
            const double noise_std_vel = 1.0 + 0.5 * dt;

            const gtsam::Vector6 diag_stds =
                (gtsam::Vector6() << noise_std_pos, noise_std_pos,
                 noise_std_pos, noise_std_vel, noise_std_vel, noise_std_vel)
                    .finished();

            auto noise_velModel =
                gtsam::noiseModel::Diagonal::Sigmas(diag_stds);

            if (dt > 10.0)
            {
                MRPT_LOG_WARN_FMT(
                    "Disabling the addition of a constant-time velocity factor "
                    "for KFs too far-away in time: dT=%.03f s",
                    dt);
            }
            else
            {
                state_.newfactors
                    .emplace_shared<gtsam::ConstVelocityBetweenNavState>(
                        f.from_kf_, f.to_kf_, dt, noise_velModel);
            }
        }
    }

    return new_fid;

    MRPT_END
}

void ASLAM_gtsam::doAdvertiseUpdatedLocalization(
    const AdvertiseUpdatedLocalization_Input& l)
{
    MRPT_START

    using mrpt::poses::CPose3D;

#if 0
    worldmodel_->entities_lock();

    const auto    p = getEntityPose(worldmodel_->entity_by_id(l.reference_kf));
    const CPose3D ref_pose(p);

    worldmodel_->entities_unlock();

    CPose3D cur_global_pose = ref_pose + CPose3D(l.pose);
    MRPT_LOG_WARN_STREAM(
        "timestamp=" << std::fixed << mrpt::Clock::toDouble(l.timestamp)
                     << ". Advertized new pose=" << cur_global_pose.asString());

    // Insert into trajectory path:
    state_.trajectory.insert(l.timestamp, cur_global_pose.asTPose());


    if (SAVE_KITTI_PATH_FILE)
    {
        static std::ofstream f("kitti_path.txt");

        const auto M =
            cur_global_pose
                .getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>();

        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 4; c++) f << mrpt::format("%f ", M(r, c));
        f << "\n";
    }
#endif

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
        }
        /*        display_->setCameraPointingToPoint(
                    last_kf_pos.x(), last_kf_pos.y(), last_kf_pos.z());
        */

        display_->repaint();
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    }
}
