/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   gtsam_mola_bridge.cpp
 * @brief  Convert datatypes between GTSAM <-> MOLA
 * @author Jose Luis Blanco Claraco
 * @date   May 29, 2019
 */

#include <mola-slam-gtsam/gtsam_mola_bridge.h>

gtsam::Pose3 mola::toPose3(const mrpt::math::TPose3D& p)
{
    gtsam::Pose3                  ret;
    mrpt::math::CQuaternionDouble q;
    mrpt::poses::CPose3D(p).getAsQuaternion(q);
    return gtsam::Pose3(
        gtsam::Rot3::Quaternion(q.r(), q.x(), q.y(), q.z()),
        gtsam::Point3(p.x, p.y, p.z));
}

mrpt::math::TPose3D mola::toTPose3D(const gtsam::Pose3& p)
{
    const auto HM = p.matrix();
    const auto H  = mrpt::math::CMatrixDouble44(HM);
    return mrpt::poses::CPose3D(H).asTPose();
}

gtsam::Point3 mola::toPoint3(const mrpt::math::TPoint3D& p)
{
    return gtsam::Point3(p.x, p.y, p.z);
}

mrpt::math::TTwist3D mola::toTTwist3D(const gtsam::Velocity3& v)
{
    mrpt::math::TTwist3D t;
    t.vx = v.x();
    t.vy = v.y();
    t.vz = v.z();
    return t;
}

void mola::updateEntityPose(mola::Entity& e, const gtsam::Pose3& x)
{
    mola::entity_update_pose(e, toTPose3D(x));
}

std::array<double, 3> mola::toVelArray(const gtsam::Velocity3& v)
{
    return {v.x(), v.y(), v.z()};
}

void mola::updateEntityVel(mola::Entity& e, const gtsam::Velocity3& v)
{
    mola::entity_update_vel(e, toVelArray(v));
}
