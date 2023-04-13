#include "general_functions.h"

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <gtsam/geometry/Pose2.h>
#include <matio.h>
#include <omp.h>

vector<gtsam::Pose2> load_robot_poses_from_csv(const string& csv_file_path) {
    vector<gtsam::Pose2> poses;

    ifstream file(csv_file_path);
    if (!file.is_open()) {
        cerr << "Failed to open CSV file: " << csv_file_path << endl;
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

    string line;
    while (getline(file, line)) {
        istringstream ss(line);
        string x_str, y_str, theta_str;
        if (getline(ss, x_str, ',') && getline(ss, y_str, ',') && getline(ss, theta_str)) {
            try {
                double x = stod(x_str);
                double y = stod(y_str);
                double theta = stod(theta_str);
                poses.emplace_back(x, y, theta);
            } catch (const invalid_argument& e) {
                cerr << "Error parsing line '" << line << "': " << e.what() << endl;
            }
        }
    }

    file.close();
    return poses;
}

void save_optimized_poses_to_csv(const vector<gtsam::Pose2> &optimizedPoses, const string &csv_file_path) {
    ofstream output_file(csv_file_path);

    if (!output_file.is_open()) {
        cerr << "Error opening the output file: " << csv_file_path << endl;
        return;
    }

    for (size_t i = 0; i < optimizedPoses.size(); ++i) {
        const gtsam::Pose2 &optimized_pose = optimizedPoses[i];
        output_file << i << "," << optimized_pose.x() << "," << optimized_pose.y() << "," << optimized_pose.theta() << endl;
    }

    output_file.close();
}
ChannelMatrix load_5d_matrix_from_mat(const string &mat_file_path, const string &variable_name){
    mat_t *matfp = Mat_Open(mat_file_path.c_str(), MAT_ACC_RDONLY);

    if (matfp == NULL){
        cerr << "Error opening MAT file " << mat_file_path << endl;
        exit(-1);
    }
    matvar_t *matvar = Mat_VarRead(matfp, variable_name.c_str());
    
    if (matvar == NULL){
        cerr << "Error reading variable " << variable_name << " from MAT file " << mat_file_path << endl;
        Mat_Close(matfp);
        exit(-1);
    }

    if (matvar->rank != 5){
        cerr << "Error: variable rank is not 5." << endl;
        Mat_VarFree(matvar);
        Mat_Close(matfp);
        exit(-1);
    }
    size_t dim0 = matvar->dims[0];
    size_t dim1 = matvar->dims[1];
    size_t dim2 = matvar->dims[2];
    size_t dim3 = matvar->dims[3];
    size_t dim4 = matvar->dims[4];

    double *data = static_cast<double *>(matvar->data);

    ChannelMatrix channel_matrix(dim0, vector<vector<vector<vector<Complex>>>>(dim1, vector<vector<vector<Complex>>>(dim2, vector<vector<Complex>>(dim3, vector<Complex>(dim4)))));
    #pragma omp parallel for collapse(5)

    for (size_t i = 0; i < dim0; ++i){
        for (size_t j = 0; j < dim1; ++j){
            for (size_t k = 0; k < dim2; ++k){
                for (size_t l = 0; l < dim3; ++l){
                    for (size_t m = 0; m < dim4; ++m){
                        size_t index = i + j*dim0 + k*dim0*dim1 + l*dim0*dim1*dim2 + m*dim0*dim1*dim2*dim3;
                        channel_matrix[i][j][k][l][m] = data[index];
                    }
                }
            }
        }
    }

    // Free the variable and close the file
    Mat_VarFree(matvar);
    Mat_Close(matfp);

    return channel_matrix;
}
