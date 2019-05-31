/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   gtsam_mola_bridge.h
 * @brief  Convert datatypes between GTSAM <-> MOLA
 * @author Jose Luis Blanco Claraco
 * @date   May 29, 2019
 */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>  // Velocity3
#include <mola-kernel/Entity.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>

namespace mola
{
/** @name gtsam_mola_utils GTSAM <-> MOLA conversion functions
 * @{ */

gtsam::Pose3 toPose3(const mrpt::math::TPose3D& p);

mrpt::math::TPose3D toTPose3D(const gtsam::Pose3& p);

gtsam::Point3 toPoint3(const mrpt::math::TPoint3D& p);

mrpt::math::TTwist3D toTTwist3D(const gtsam::Velocity3& v);

void updateEntityPose(Entity& e, const gtsam::Pose3& x);

std::array<double, 3> toVelArray(const gtsam::Velocity3& v);

void updateEntityVel(Entity& e, const gtsam::Velocity3& v);

/** @} */

}  // namespace mola
