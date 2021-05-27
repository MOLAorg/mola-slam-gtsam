/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ConstVelocityFactorSE3.h
 * @brief  Constant velocity factor in SE(3)
 * @author Jose Luis Blanco Claraco
 * @date   May 29, 2019
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
//#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/NavState.h>  // Velocity3
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace mola
{
/**
 * Factor for constant velocity model in SE(3) between pairs Pose3+Velocity3
 */
class ConstVelocityFactorSE3
    : public gtsam::NoiseModelFactor4<
          gtsam::Pose3, gtsam::Velocity3, gtsam::Pose3, gtsam::Velocity3>
{
   private:
    using This = ConstVelocityFactorSE3;
    using Base = NoiseModelFactor4<
        gtsam::Pose3, gtsam::Velocity3, gtsam::Pose3, gtsam::Velocity3>;
    using Measure = double;

    /** Time between the states key1 & key2 */
    double deltaTime_;

   public:
    // shorthand for a smart pointer to a factor
    using shared_ptr = boost::shared_ptr<ConstVelocityFactorSE3>;

    /** default constructor - only use for serialization */
    ConstVelocityFactorSE3() {}

    /** Constructor.  */
    ConstVelocityFactorSE3(
        gtsam::Key pose1, gtsam::Key vel1, gtsam::Key pose2, gtsam::Key vel2,
        const double deltaTime, const gtsam::SharedNoiseModel& model)
        : Base(model, pose1, vel1, pose2, vel2), deltaTime_(deltaTime)
    {
    }

    virtual ~ConstVelocityFactorSE3() override;

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const override;

    /** implement functions needed for Testable */

    /** print */
    virtual void print(
        const std::string& s, const gtsam::KeyFormatter& keyFormatter =
                                  gtsam::DefaultKeyFormatter) const override;

    /** equals */
    virtual bool equals(
        const gtsam::NonlinearFactor& expected,
        double                        tol = 1e-9) const override;

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    gtsam::Vector evaluateError(
        const gtsam::Pose3& p1, const gtsam::Velocity3& v1,
        const gtsam::Pose3& p2, const gtsam::Velocity3& v2,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none,
        boost::optional<gtsam::Matrix&> H4 = boost::none) const override;

    /** number of variables attached to this factor */
    std::size_t size() const;

   private:
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& boost::serialization::make_nvp(
            "ConstVelocityFactorSE3",
            boost::serialization::base_object<Base>(*this));
        ar& BOOST_SERIALIZATION_NVP(deltaTime_);
    }

    // Alignment, see
    // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mola
