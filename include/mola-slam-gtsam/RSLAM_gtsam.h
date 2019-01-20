/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   RSLAM_gtsam.h
 * @brief  Reference implementation of relative SLAM with GTSAM factor graphs
 * @author Jose Luis Blanco Claraco
 * @date   Dec 21, 2018
 */
#pragma once

// mrpt includes first:
#include <mola-kernel/BackEndBase.h>
// gtsam next:
#include <gtsam/slam/BetweenFactor.h>

namespace mola
{
/** Reference implementation of relative SLAM with GTSAM factor graphs
 * See docs in: \ref mola_slam_gtsam_grp
 * \ingroup mola_slam_gtsam_grp */
class RSLAM_gtsam : public BackEndBase
{
   public:
    RSLAM_gtsam();
    ~RSLAM_gtsam() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

    // Impl. if virtual methods. See base class docs:
    ProposeKF_Output doAddKeyFrame(const ProposeKF_Input& i) override;
    AddFactor_Output doAddFactor(Factor& f) override;
    bool             doFactorExistsBetween(id_t a, id_t b) override;
    void             doAdvertiseUpdatedLocalization(
                    const AdvertiseUpdatedLocalization_Input& l) override;

   private:
};

}  // namespace mola
