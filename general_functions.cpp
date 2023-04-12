#include "general_functions.h"

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <gtsam/geometry/Pose2.h>

std::vector<gtsam::Pose2> load_robot_poses_from_csv(const std::string& csv_file_path) {
    std::vector<gtsam::Pose2> poses;

    std::ifstream file(csv_file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << csv_file_path << std::endl;
        return poses;
    }

    // Skip the BOM if it's present at the beginning of the file
    if (file.peek() == 0xEF) {
        file.ignore(1);
        if (file.peek() == 0xBB) {
            file.ignore(1);
            if (file.peek() == 0xBF) {
                file.ignore(1);
            }
        }
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string x_str, y_str, theta_str;
        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, theta_str)) {
            try {
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                double theta = std::stod(theta_str);
                poses.emplace_back(x, y, theta);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error parsing line '" << line << "': " << e.what() << std::endl;
            }
        }
    }

    file.close();
    return poses;
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

