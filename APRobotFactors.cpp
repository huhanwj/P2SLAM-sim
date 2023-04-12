#include "APRobotFactors.h"
#include <cmath>
// AP2RobotFactor implementation

AP2RobotFactor::AP2RobotFactor(gtsam::Key poseKey, gtsam::Key pointKey, double bearing, const gtsam::SharedNoiseModel& noiseModel)
    : gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2>(noiseModel, poseKey, pointKey), bearing_(bearing) {}

gtsam::Vector AP2RobotFactor::evaluateError(const gtsam::Pose2& pose, const gtsam::Pose2& point,
                                            boost::optional<gtsam::Matrix&> H1,
                                            boost::optional<gtsam::Matrix&> H2) const {
    // Implement the error calculation and Jacobian calculation for AP2RobotFactor here, using bearing_ as the measurement
    double dx = point.x() - pose.x();
    double dy = point.y() - pose.y();
    double theta = point.theta();
    double c = cos(-theta);
    double s = sin(-theta);
    double predicted_bearing = atan2(c*dy - s*dx, c*dx + s*dy);

    double error = bearing_ - predicted_bearing;

    if (H1){
        double dr_dtheta = s*dx + c*dy;
        double dr_dx = c;
        double dr_dy = -s;

        double denominator = dr_dtheta * dr_dtheta + dr_dx*dr_dx+ dr_dy*dr_dy;
        *H1 = (gtsam::Matrix(1,3) <<
                dr_dy / denominator,
                -dr_dx / denominator,
                -dr_dtheta / denominator).finished();
    }

    if (H2) {
        double dr_dtheta = s*dx + c*dy;
        double dr_dx = c;
        double dr_dy = -s;

        double denominator = dr_dtheta * dr_dtheta + dr_dx*dr_dx+ dr_dy*dr_dy;
        *H2 = (gtsam::Matrix(1,3) <<
                - dr_dx / (dx*dx + dy*dy),
                dr_dy / (dx*dx + dy*dy),
                0).finished();
    }
    return gtsam::Vector1(error);
}

// Robot2APFactor implementation

Robot2APFactor::Robot2APFactor(gtsam::Key poseKey, gtsam::Key pointKey, double bearing, const gtsam::SharedNoiseModel& noiseModel)
    : gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2>(noiseModel, poseKey, pointKey), bearing_(bearing) {}

gtsam::Vector Robot2APFactor::evaluateError(const gtsam::Pose2& pose, const gtsam::Pose2& point,
                                            boost::optional<gtsam::Matrix&> H1,
                                            boost::optional<gtsam::Matrix&> H2) const {
    // Implement the error calculation and Jacobian calculation for Robot2APFactor here, using bearing_ as the measurement
    double dx = point.x() - pose.x();
    double dy = point.y() - pose.y();
    double theta = pose.theta();
    double c = cos(-theta);
    double s = sin(-theta);
    double predicted_bearing = atan2(c*dy - s*dx, c*dx + s*dy);

    double error = bearing_ - predicted_bearing;
    if (H1){
        double dr_dtheta = s*dx + c*dy;
        double dr_dx = c;
        double dr_dy = -s;

        double denominator = dr_dtheta * dr_dtheta + dr_dx*dr_dx+ dr_dy*dr_dy;
        *H1 = (gtsam::Matrix(1,3) <<
                dr_dy / denominator,
                -dr_dx / denominator,
                -dr_dtheta / denominator).finished();
    }

    if (H2) {
        double dr_dtheta = s*dx + c*dy;
        double dr_dx = c;
        double dr_dy = -s;

        double denominator = dr_dtheta * dr_dtheta + dr_dx*dr_dx+ dr_dy*dr_dy;
        *H2 = (gtsam::Matrix(1,3) <<
                - dr_dx / (dx*dx + dy*dy),
                dr_dy / (dx*dx + dy*dy),
                0).finished();
    }
    return gtsam::Vector1(error);
}
