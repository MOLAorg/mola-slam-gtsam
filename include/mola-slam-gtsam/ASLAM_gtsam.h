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
// gtsam next:
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
    AddFactor_Output doAddFactor(Factor& f) override;

   private:
    struct SLAM_state
    {
        /** Incremental estimator */
        gtsam::ISAM2 isam2;

        /** Pending new elements to add to the map */
        gtsam::NonlinearFactorGraph newfactors;
        gtsam::Values               newvalues;

        /** Absolute coordinates single reference frame */
        id_t root_kf_id{mola::INVALID_ID};
    };

    SLAM_state                 state_;
    std::recursive_timed_mutex isam2_lock_;
};

}  // namespace mola
