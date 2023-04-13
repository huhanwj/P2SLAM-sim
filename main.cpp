#include "RobotBetweenFactor.h"
#include "general_functions.h"
#include "APRobotFactors.h"
#include "CSI_process.h"
#include "noise_model.h"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

using namespace gtsam;
using namespace std;

int main() {
    // Read IMU measurements, robot poses and 2-way raw CSI data
    string pose_csv_file_path = "data/DS1_noise.csv";
    vector<Pose2> robotPoses = load_robot_poses_from_csv(pose_csv_file_path);

    if (robotPoses.empty()) {
        cerr << "Failed to load robot poses from the CSV file" << endl;
        return 1;
    }
    string imu_csv_file_path = "data/DS1_imu.csv";
    vector<Pose2> imuMeasurements = load_robot_poses_from_csv(imu_csv_file_path);; // Load or generate IMU measurements

    if (imuMeasurements.empty()) {
        cerr << "Failed to load IMU measurements from the CSV file" << endl;
        return 1;
    }

    if (imuMeasurements.size() != robotPoses.size() - 1) {
        cerr << "The number of IMU measurements must be one less than the number of robot poses." << endl;
        return 1;
    }
    string channels_mat_file_path = "data/channels.mat";
    ChannelMatrix channels_ap = load_5d_matrix_from_mat(channels_mat_file_path, "channels_ap");
    ChannelMatrix channels_cli = load_5d_matrix_from_mat(channels_mat_file_path, "channels_cli");

    // Wireless environment setting
    double f0 = 5.18e9; // Central frequency for channel 36 at 5GHz
    double c = 3.0e8; // Speed of light
    int N = channels_ap[0].size(); // Number of frequency bins
    int ap_num = channels_ap[0][0].size();
    int rx_num = channels_ap[0][0][0].size();
    int tx_num = channels_ap[0][0][0][0].size();
    double d = 0.05; // Antenna spacing, set at 5cm here, not provided in P2SLAM

    // variables for direct path bearing
    vector<vector<double>> bearings_ap(robotPoses.size(), vector<double>(channels_ap[0][0].size()));
    vector<vector<double>> bearings_cli(robotPoses.size(), vector<double>(channels_cli[0][0].size()));
    // Matrix for the reconstructed CSI after bearing search
    ChannelMatrix combined_CSI(
        channels_ap.size(),
            vector<vector<vector<vector<Complex>>>>(channels_ap[0].size(),
                vector<vector<vector<Complex>>>(channels_ap[0][0].size(),
                    vector<vector<Complex>>(channels_ap[0][0][0].size(),
                        vector<Complex>(channels_ap[0][0][0][0].size())
                )      
            )
        )
    );

    // Calculate the bearings
    for (size_t i = 0; i < robotPoses.size(); ++i){
        for (size_t j = 0; j < ap_num; ++j){
            double bearing_ap = direct_bearing(channels_ap, f0, c, N, d, i, j);
            double bearing_cli = direct_bearing(channels_cli, f0, c, N, d, i, j);
            bearings_ap[i][j] = bearing_ap;
            bearings_cli[i][j] = bearing_cli;
            ChannelMatrix reconstructed_csi = reconstruct_CSI(channels_ap, i, j, f0, c, N, d, bearing_ap);
            for (size_t n = 0; n < N; ++n) {
                for (size_t rx = 0; rx < rx_num; ++rx) {
                    for (size_t tx = 0; tx < tx_num; ++tx) {
                        combined_CSI[i][n][j][rx][tx] = reconstructed_csi[0][n][0][rx][tx];
                    }
                }
            }
        }
    }
    // Define the noise model for the IMU measurements
    SharedNoiseModel imuNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

    // Define the noise model for the landmark measurements.
    // TODO: model the noise from the algorithm from ArgMax and ArgMin: Transitional probabilistic models in cognitive radio mesh networks.
    SharedNoiseModel customFactorsNoise = noiseModel::Diagonal::Sigmas(Vector1(0.01)); //need change

    // Create a NonlinearFactorGraph and add RobotBetweenFactors for each IMU measurement
    NonlinearFactorGraph graph;

    for (size_t i = 0; i < imuMeasurements.size(); ++i) {
        graph.add(RobotBetweenFactor(Symbol('x', i), Symbol('x', i + 1), imuMeasurements[i], imuNoise));
    }


    // Add the APRobotFactors for each AP pose
    for (size_t i = 0; i < robotPoses.size(); ++i){
        for (size_t j = 0;j < ap_num; ++j){
            graph.add(AP2RobotFactor(Symbol('l',j), Symbol('x',i), bearings_ap[i][j], customFactorsNoise));
            graph.add(Robot2APFactor(Symbol('x',i), Symbol('l',j), bearings_cli[i][j], customFactorsNoise));
        }
    }
    // Add a prior factor to the first pose
    Pose2 priorPose = robotPoses[0];
    SharedNoiseModel priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
    graph.add(PriorFactor<Pose2>(Symbol('x', 0), priorPose, priorNoise));

    // Optimize the graph using Levenberg-Marquardt
    Values initialEstimates;
    for (size_t i = 0; i < robotPoses.size(); ++i) {
        initialEstimates.insert(Symbol('x', i), robotPoses[i]);
    }
    for (size_t j = 0; j < ap_num; ++j){
        initialEstimates.insert(Symbol('l',j),Pose2(0,0,0));
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
