/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ASLAM_gtsam.h
 * @brief  SLAM in absolute coordinates with GTSAM factor graphs
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2018
 */
#pragma once

// mrpt includes first:
#include <mola-kernel/BackEndBase.h>
#include <mola-kernel/WorkerThreadsPool.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/typemeta/TEnumType.h>
// gtsam next:
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <mutex>

namespace mola
{
/** Reference implementation of absolute-coordinates SLAM with GTSAM factor
 * graphs.
 * See docs in: \ref mola_slam_gtsam_grp
 * \ingroup mola_slam_gtsam_grp */
class ASLAM_gtsam : public BackEndBase
{
   public:
    ASLAM_gtsam();
    ~ASLAM_gtsam() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

    /** Type selector for kind of KeyFrame state vector representation */
    enum class StateVectorType : int8_t
    {
        SE2 = 0,
        SE3,
        DynSE2,
        DynSE3,
        Undefined = -1
    };

    struct Parameters
    {
        StateVectorType state_vector{StateVectorType::Undefined};
    };

    Parameters params_;

    // Impl. if virtual methods. See base class docs:
    ProposeKF_Output doAddKeyFrame(const ProposeKF_Input& i) override;
    AddFactor_Output doAddFactor(Factor& newF) override;
    void             doAdvertiseUpdatedLocalization(
                    const AdvertiseUpdatedLocalization_Input& l) override;

   private:
    /** Indices for accessing the KF_gtsam_keys array */
    enum kf_key_index_t
    {
        KF_KEY_POSE = 0,
        KF_KEY_VEL,
        //-- end of list --
        KF_KEY_COUNT
    };

    using KF_gtsam_keys = std::array<gtsam::Key, KF_KEY_COUNT>;

    struct SLAM_state
    {
        /** Incremental estimator */
        std::unique_ptr<gtsam::ISAM2> isam2;

        /** Pending new elements to add to the map */
        gtsam::NonlinearFactorGraph newfactors;
        gtsam::Values               newvalues;
        std::set<mola::id_t>        kf_has_value;

        /** History of vehicle poses over time */
        // mrpt::poses::CPose3DInterpolator trajectory;

        // locked by last_kf_estimates_lock_ as well:
        mrpt::graphs::CNetworkOfPoses3D            vizmap;
        std::map<mola::id_t, mrpt::math::TTwist3D> vizmap_dyn;

        /** Absolute coordinates single reference frame (WorldModel index) */
        id_t root_kf_id{mola::INVALID_ID};

        id_t last_created_kf_id{mola::INVALID_ID};

        /** Map between mola WorldModel KF indices and the corresponding gtsam
         * Key(s) value(s). When in SE2/SE3 mode, only the pose Key is used.
         * When in DynSE2/DynSE3 mode, the extra key for the velocity variable
         * is stored a well */
        std::map<mola::id_t, KF_gtsam_keys> mola2gtsam;
        /** Inverse map for `mola2gtsam` (indexed by gtsam *pose* ID) */
        std::array<std::map<gtsam::Key, mola::id_t>, KF_KEY_COUNT> gtsam2mola;
    };

    SLAM_state                 state_;
    std::recursive_timed_mutex isam2_lock_;
    std::recursive_timed_mutex vizmap_lock_;
    std::recursive_timed_mutex keys_map_lock_;  //!< locks mola2gtsam/gtsam2mola

    fid_t addFactor(const FactorRelativePose3& f);
    fid_t addFactor(const FactorRelativePose3ConstVel& f);

    fid_t internal_addFactorRelPose(
        const FactorRelativePose3& f, const bool addDynamicsFactor);

    mola::id_t internal_addKeyFrame_Root(const ProposeKF_Input& i);
    mola::id_t internal_addKeyFrame_Regular(const ProposeKF_Input& i);

    // TODO: Temporary code, should be moved to a new module "MapViz":
    // --------------
    mola::WorkerThreadsPool gui_updater_pool_{
        1, mola::WorkerThreadsPool::POLICY_DROP_OLD};

    struct DisplayInfo
    {
        mrpt::graphs::CNetworkOfPoses3D vizmap;
    };
    /** This will be run in a dedicated thread inside gui_updater_pool_ */
    void doUpdateDisplay(std::shared_ptr<DisplayInfo> di);

    mrpt::gui::CDisplayWindow3D::Ptr display_;
    // ----------------------------
};

}  // namespace mola

MRPT_ENUM_TYPE_BEGIN(mola::ASLAM_gtsam::StateVectorType)
MRPT_FILL_ENUM_MEMBER(mola::ASLAM_gtsam::StateVectorType, SE2);
MRPT_FILL_ENUM_MEMBER(mola::ASLAM_gtsam::StateVectorType, SE3);
MRPT_FILL_ENUM_MEMBER(mola::ASLAM_gtsam::StateVectorType, DynSE2);
MRPT_FILL_ENUM_MEMBER(mola::ASLAM_gtsam::StateVectorType, DynSE3);
MRPT_ENUM_TYPE_END()
