#include "RobotBetweenFactor.h"
#include "general_functions.h"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;
using namespace std;

int main() {
    // Step 1: Read or generate IMU measurements and robot poses
    string pose_csv_file_path = "DS1_noise.csv";
    vector<gtsam::Pose2> robotPoses = load_robot_poses_from_csv(pose_csv_file_path);

    if (robotPoses.empty()) {
        cerr << "Failed to load robot poses from the CSV file" << endl;
        return 1;
    }
    string imu_csv_file_path = "DS1_gt.csv";
    vector<Pose2> imuMeasurements = load_robot_poses_from_csv(imu_csv_file_path);; // Load or generate IMU measurements

    if (robotPoses.empty()) {
        cerr << "Failed to load IMU measurements from the CSV file" << endl;
        return 1;
    }

    if (imuMeasurements.size() != robotPoses.size() - 1) {
        cerr << "The number of IMU measurements must be one less than the number of robot poses." << endl;
        return 1;
    }

    // Step 2: Define the noise model for the IMU measurements
    SharedNoiseModel imuNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

    // Step 3: Create a NonlinearFactorGraph and add RobotBetweenFactors for each IMU measurement
    NonlinearFactorGraph graph;

    for (size_t i = 0; i < imuMeasurements.size(); ++i) {
        graph.add(RobotBetweenFactor(Symbol('x', i), Symbol('x', i + 1), imuMeasurements[i], imuNoise));
    }

    // Step 4: Add a prior factor to the first pose
    Pose2 priorPose = robotPoses[0];
    SharedNoiseModel priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph.add(PriorFactor<Pose2>(Symbol('x', 0), priorPose, priorNoise));

    // Step 5: Optimize the graph using Levenberg-Marquardt
    Values initialEstimates;
    for (size_t i = 0; i < robotPoses.size(); ++i) {
        initialEstimates.insert(Symbol('x', i), robotPoses[i]);
    }

    LevenbergMarquardtOptimizer optimizer(graph, initialEstimates);
    Values optimizedValues = optimizer.optimize();

    // Print the optimized pose estimates

    vector<gtsam::Pose2> optimizedPoses;
    for (size_t i = 0; i < robotPoses.size(); ++i) {
        gtsam::Pose2 optimized_pose = optimizedValues.at<gtsam::Pose2>(gtsam::Symbol('x', i));
        cout << "Optimized pose " << i << ": x=" << optimized_pose.x() << ", y=" << optimized_pose.y() << ", theta=" << optimized_pose.theta() << endl;
        optimizedPoses.push_back(optimized_pose);
    }

    // Replace this with the desired output CSV file path
    string output_csv_file_path = "optimized_poses.csv";
    save_optimized_poses_to_csv(optimizedPoses, output_csv_file_path);

    return 0;
}
