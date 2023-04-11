#ifndef GENERAL_FUNCTIONS_H
#define GENERAL_FUNCTIONS_H

#include <gtsam/geometry/Pose2.h>
#include <vector>
#include <string>

std::vector<gtsam::Pose2> load_robot_poses_from_csv(const std::string &csv_file_path);
void save_optimized_poses_to_csv(const std::vector<gtsam::Pose2> &optimizedPoses, const std::string &csv_file_path);


#endif // GENERAL_FUNCTIONS_H
