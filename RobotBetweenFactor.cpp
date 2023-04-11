#include "RobotBetweenFactor.h"
#include <cmath>

RobotBetweenFactor::RobotBetweenFactor(gtsam::Key poseKey1, gtsam::Key poseKey2, const gtsam::Pose2& between, const gtsam::SharedNoiseModel& noiseModel)
    : gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2>(noiseModel, poseKey1, poseKey2), between_(between) {}

gtsam::Vector RobotBetweenFactor::evaluateError(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2,
                              boost::optional<gtsam::Matrix&> H1,
                              boost::optional<gtsam::Matrix&> H2) const {
    gtsam::Vector3 error;

    // Calculate the error
    double theta1 = pose1.theta();
    double c1 = cos(-theta1);
    double s1 = sin(-theta1);

    error(0) = c1 * (pose2.x() - pose1.x()) + s1 * (pose2.y() - pose1.y()) - between_.x();
    error(1) = -s1 * (pose2.x() - pose1.x()) + c1 * (pose2.y() - pose1.y()) - between_.y();
    error(2) = pose2.theta() - pose1.theta() - between_.theta();

    // Calculate the Jacobians
    if (H1) {
      *H1 = (gtsam::Matrix(3, 3) <<
            -c1, -s1, s1 * (pose2.x() - pose1.x()) - c1 * (pose2.y() - pose1.y()),
             s1, -c1, -c1 * (pose2.x() - pose1.x()) - s1 * (pose2.y() - pose1.y()),
             0, 0, -1).finished();
    }

    if (H2) {
      *H2 = (gtsam::Matrix(3, 3) <<
             c1, s1, 0,
            -s1, c1, 0,
             0, 0, 1).finished();
    }

    return error;
}
