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

    // Impl. if virtual methods. See base class docs:
    ProposeKF_Output doAddKeyFrame(const ProposeKF_Input& i) override;
    AddFactor_Output doAddFactor(Factor& newF) override;
    bool             doFactorExistsBetween(id_t a, id_t b) override;
    void             doAdvertiseUpdatedLocalization(
                    const AdvertiseUpdatedLocalization_Input& l) override;

   private:
    struct SLAM_state
    {
        /** Incremental estimator */
        std::unique_ptr<gtsam::ISAM2> isam2;

        /** Pending new elements to add to the map */
        gtsam::NonlinearFactorGraph newfactors;
        gtsam::Values               newvalues;
        std::set<mola::id_t>        kf_has_value;

        /** History of vehicle poses over time */
        mrpt::poses::CPose3DInterpolator trajectory;

        // locked by last_kf_estimates_lock_ as well:
        mrpt::graphs::CNetworkOfPoses3D vizmap;

        /** Absolute coordinates single reference frame */
        id_t root_kf_id{mola::INVALID_ID};
    };

    SLAM_state                 state_;
    std::recursive_timed_mutex isam2_lock_;
    std::recursive_timed_mutex vizmap_lock_;

    fid_t addFactor(const FactorRelativePose3& f);

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
