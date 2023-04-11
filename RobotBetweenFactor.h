#ifndef ROBOTBETWEENFACTOR_H
#define ROBOTBETWEENFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>

class RobotBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2> {
public:
  RobotBetweenFactor(gtsam::Key poseKey1, gtsam::Key poseKey2, const gtsam::Pose2& between, const gtsam::SharedNoiseModel& noiseModel);

  gtsam::Vector evaluateError(const gtsam::Pose2& pose1, const gtsam::Pose2& pose2,
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
  gtsam::Pose2 between_;
};

#endif // ROBOTBETWEENFACTOR_H
