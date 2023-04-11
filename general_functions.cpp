#include "general_functions.h"

#include <fstream>
#include <sstream>
#include <iostream>

std::vector<gtsam::Pose2> load_robot_poses_from_csv(const std::string &csv_file_path) {
    std::vector<gtsam::Pose2> robotPoses;
    std::ifstream poses_file(csv_file_path);

    if (!poses_file.is_open()) {
        std::cerr << "Error opening the file: " << csv_file_path << std::endl;
        return robotPoses;
    }

    std::string line;
    while (std::getline(poses_file, line)) {
        std::stringstream line_stream(line);

        double x, y, theta;

        std::getline(line_stream, line, ',');
        x = std::stod(line);

        std::getline(line_stream, line, ',');
        y = std::stod(line);

        std::getline(line_stream, line, ',');
        theta = std::stod(line);

        robotPoses.push_back(gtsam::Pose2(x, y, theta));
    }

    poses_file.close();
    return robotPoses;
}
void save_optimized_poses_to_csv(const std::vector<gtsam::Pose2> &optimizedPoses, const std::string &csv_file_path) {
    std::ofstream output_file(csv_file_path);

    if (!output_file.is_open()) {
        std::cerr << "Error opening the output file: " << csv_file_path << std::endl;
        return;
    }

    for (size_t i = 0; i < optimizedPoses.size(); ++i) {
        const gtsam::Pose2 &optimized_pose = optimizedPoses[i];
        output_file << i << "," << optimized_pose.x() << "," << optimized_pose.y() << "," << optimized_pose.theta() << std::endl;
    }

    output_file.close();
}

