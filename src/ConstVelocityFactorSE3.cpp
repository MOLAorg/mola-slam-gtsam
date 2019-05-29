/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ConstVelocityFactorSE3.cpp
 * @brief  Constant velocity factor in SE(3)
 * @author Jose Luis Blanco Claraco
 * @date   May 29, 2019
 */

#include <mola-slam-gtsam/ConstVelocityFactorSE3.h>

using namespace mola;

ConstVelocityFactorSE3::~ConstVelocityFactorSE3() = default;

gtsam::Vector ConstVelocityFactorSE3::evaluateError(
    const gtsam::Pose3& p1, const gtsam::Velocity3& v1, const gtsam::Pose3& p2,
    const gtsam::Velocity3& v2, boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2, boost::optional<gtsam::Matrix&> H3,
    boost::optional<gtsam::Matrix&> H4) const
{
    gtsam::Vector6 err;
    err.head<3>() = p1.translation() + v1 * deltaTime_ - p2.translation();
    err.tail<3>() = v2 - v1;

    if (H1)
    {
        auto& H1v = H1.value();
        H1v.setZero(6, 6);
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
        H3v.setZero(6, 6);
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

gtsam::NonlinearFactor::shared_ptr ConstVelocityFactorSE3::clone() const
{
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

void ConstVelocityFactorSE3::print(
    const std::string& s, const gtsam::KeyFormatter& keyFormatter) const
{
    std::cout << s << "ConstVelocityFactorSE3(" << keyFormatter(this->key1())
              << "," << keyFormatter(this->key2()) << ","
              << keyFormatter(this->key3()) << "," << keyFormatter(this->key4())
              << ")\n";
    gtsam::traits<double>::Print(deltaTime_, "  deltaTime: ");
    this->noiseModel_->print("  noise model: ");
}

bool ConstVelocityFactorSE3::equals(
    const gtsam::NonlinearFactor& expected, double tol) const
{
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::traits<Measure>::Equals(this->deltaTime_, e->deltaTime_, tol);
}

/** number of variables attached to this factor */
std::size_t ConstVelocityFactorSE3::size() const { return 4; }
