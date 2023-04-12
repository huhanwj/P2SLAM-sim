#ifndef APROBOTFACTORS_H
#define APROBOTFACTORS_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>

class AP2RobotFactor : public gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2> {
public:
  AP2RobotFactor(gtsam::Key poseKey, gtsam::Key pointKey, double bearing, const gtsam::SharedNoiseModel& noiseModel);

  gtsam::Vector evaluateError(const gtsam::Pose2& pose, const gtsam::Pose2& point,
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
  double bearing_;
};

class Robot2APFactor : public gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2> {
public:
  Robot2APFactor(gtsam::Key poseKey, gtsam::Key pointKey, double bearing, const gtsam::SharedNoiseModel& noiseModel);

  gtsam::Vector evaluateError(const gtsam::Pose2& pose, const gtsam::Pose2& point,
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
  double bearing_;
};

#endif // APROBOTFACTORS_H
