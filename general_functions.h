#ifndef GENERAL_FUNCTIONS_H
#define GENERAL_FUNCTIONS_H

#include <gtsam/geometry/Pose2.h>
#include <vector>
#include <string>
#include <complex>

using namespace std;

typedef complex<double> Complex;
typedef vector<vector<vector<vector<vector<Complex>>>>> ChannelMatrix;

vector<gtsam::Pose2> load_robot_poses_from_csv(const string &csv_file_path);
void save_optimized_poses_to_csv(const vector<gtsam::Pose2> &optimizedPoses, const string &csv_file_path);

ChannelMatrix load_5d_matrix_from_mat(const string &mat_file_path, const string &variable_name);

#endif // GENERAL_FUNCTIONS_H
