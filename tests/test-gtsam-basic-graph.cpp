/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-gtsam-basic-graph.cpp
 * @brief  Just defines a simple pose graph and optimizes it.
 *  Do NOT use MRPT in any way, just to make sure that gtsam builds and runs as
 * expected in a user program.
 *
 *  Basically, copied from the GTSAM example Pose2SLAMExample.cpp (BSD License)
 *
 * @author Jose Luis Blanco Claraco
 * @date   Aug 14, 2019
 */

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file Pose2SLAMExample.cpp
 * @brief A 2D Pose SLAM example
 * @date Oct 21, 2010
 * @author Yong Dian Jian
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

void test_gtsam_basic_graph()
{
    using namespace std;
    using namespace gtsam;

    NonlinearFactorGraph graph;
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.emplace_shared<PriorFactor<Pose2>>(1, Pose2(0, 0, 0), priorNoise);
    auto model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

    graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, Pose2(2, 0, 0), model);
    graph.emplace_shared<BetweenFactor<Pose2>>(
        2, 3, Pose2(2, 0, M_PI_2), model);
    graph.emplace_shared<BetweenFactor<Pose2>>(
        3, 4, Pose2(2, 0, M_PI_2), model);
    graph.emplace_shared<BetweenFactor<Pose2>>(
        4, 5, Pose2(2, 0, M_PI_2), model);

    graph.emplace_shared<BetweenFactor<Pose2>>(
        5, 2, Pose2(2, 0, M_PI_2), model);

    Values initialEstimate;
    initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
    initialEstimate.insert(3, Pose2(4.1, 0.1, M_PI_2));
    initialEstimate.insert(4, Pose2(4.0, 2.0, M_PI));
    initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));

    GaussNewtonParams parameters;
    parameters.relativeErrorTol = 1e-5;
    parameters.maxIterations    = 100;
    GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
    Values               result = optimizer.optimize();
    // result.print("Final Result:\n");

    Marginals marginals(graph, result);
    // cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;

    /* Expected results:
    Value 1:  (-4.36656651e-21, -8.80618068e-20, -3.71519043e-20)
    Value 2:  (2, -2.01504196e-19, -5.47345182e-20)
    Value 3:  (4, -3.42173709e-11, 1.57079633)
    Value 4:  (4, 2, 3.14159265)
    Value 5:  (2, 2, -1.57079633)
    */
    const auto& p3    = result.at(3).cast<gtsam::Pose2>();
    const auto  gt_p3 = gtsam::Pose2(4, 0, 1.57079633);
    if (std::abs(p3.x() - gt_p3.x()) > 1e-4 ||
        std::abs(p3.y() - gt_p3.y()) > 1e-4 ||
        std::abs(p3.theta() - gt_p3.theta()) > 1e-4)
        throw std::runtime_error("pose[3] didn't match expected value.");
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_gtsam_basic_graph();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
