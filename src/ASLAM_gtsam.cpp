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

static void updateEntityPose(mola::Entity& e, const mrpt::math::TPose3D& p)
{
    MRPT_TRY_START
    //
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
                    mrpt::format("[updateEntity] Unknown Entity type!"));
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
                    mrpt::format("[updateEntity] Unknown Entity type!"));
            },
        },
        e);

    return ret;
    MRPT_TRY_END
}

namespace gtsam
{
/**
 * Factor for SE(3) between NavState:
 */
class BetweenNavStateFactor : public NoiseModelFactor2<NavState, NavState>
{
    // Check that VALUE type is a testable Lie group
    // BOOST_CONCEPT_ASSERT((IsTestable<NavState>));
    // BOOST_CONCEPT_ASSERT((IsLieGroup<NavState>));

   public:
    using T = NavState;

   private:
    using This = BetweenNavStateFactor;
    using Base = NoiseModelFactor2<NavState, NavState>;

    using Measure = Pose3;

    Measure measured_; /** The measurement */

   public:
    // shorthand for a smart pointer to a factor
    using shared_ptr = boost::shared_ptr<BetweenNavStateFactor>;

    /** default constructor - only use for serialization */
    BetweenNavStateFactor() {}

    /** Constructor */
    BetweenNavStateFactor(
        Key key1, Key key2, const Measure& measured,
        const SharedNoiseModel& model)
        : Base(model, key1, key2), measured_(measured)
    {
    }

    virtual ~BetweenNavStateFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(
        const std::string&  s,
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const
    {
        std::cout << s << "BetweenNavStateFactor(" << keyFormatter(this->key1())
                  << "," << keyFormatter(this->key2()) << ")\n";
        traits<Measure>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(
        const NonlinearFactor& expected, double tol = 1e-9) const
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != NULL && Base::equals(*e, tol) &&
               traits<Measure>::Equals(this->measured_, e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(
        const NavState& p1, const NavState& p2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const override
    {
        auto hx = traits<Pose3>::Between(p1.pose(), p2.pose(), H1, H2);  // h(x)

        // manifold equivalent of h(x)-z -> log(z,h(x))
#ifdef SLOW_BUT_CORRECT_BETWEENFACTOR
        typename traits<T>::ChartJacobian::Jacobian Hlocal;
        Vector                                      rval = traits<T>::Local(
            measured_, hx, boost::none, (H1 || H2) ? &Hlocal : 0);
        if (H1) *H1 = Hlocal * (*H1);
        if (H2) *H2 = Hlocal * (*H2);
        return rval;
#else
        return traits<Pose3>::Local(measured_, hx);
#endif
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
            "BetweenNavStateFactor",
            boost::serialization::base_object<Base>(*this));
        ar& BOOST_SERIALIZATION_NVP(measured_);
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
            const mola::id_t kf_id   = key_value.key;
            gtsam::NavState  kf_pose = key_value.value.cast<gtsam::NavState>();
            const auto       p       = toTPose3D(kf_pose.pose());

            // Dont update the pose of the global reference, fixed to Identity()
            if (kf_id != state_.root_kf_id)
                updateEntityPose(worldmodel_->entity_by_id(kf_id), p);

            // mapviz:
            state_.vizmap.nodes[kf_id] = mrpt::poses::CPose3D(p);

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
            state_.root_kf_id = worldmodel_->entity_emplace_back(root);
            worldmodel_->entities_unlock();
        }

        // And add a prior to iSAM2:
        auto priorModel = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(9) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,
             1e-4)
                .finished());

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

    worldmodel_->entities_lock();

    const auto from_pose_est =
        getEntityPose(worldmodel_->entity_by_id(f.from_kf_));
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

    // Noise model:
    MRPT_TODO("Grab uncertainty from front-end");
    auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);

    {
        MRPT_TODO("add a new lock for last_kf_estimates");
        std::lock_guard<decltype(isam2_lock_)> lock(isam2_lock_);

        // Add to list of initial guess (if not done already with a former
        // factor):
        if (state_.kf_has_value.count(f.to_kf_) == 0)
        {
            state_.kf_has_value.insert(f.to_kf_);
            state_.newvalues.insert(f.to_kf_, toNavState(to_pose_est));
        }

        state_.newfactors.emplace_shared<gtsam::BetweenNavStateFactor>(
            f.from_kf_, f.to_kf_, measure, noise);
    }

    return new_fid;

    MRPT_END
}

bool ASLAM_gtsam::doFactorExistsBetween(id_t a, id_t b)
{
    MRPT_START

    THROW_EXCEPTION("to do!");

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
