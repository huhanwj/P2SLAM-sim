#include "CSI_process.h"
#include <cmath>
#include <complex>
#include <vector>
#include <iostream>
#include <limits>

using namespace std;

Complex calculate_lambda_ij(const ChannelMatrix& H, double f0, double c, int N, double d, int i, int j, double z_i, double l_j){
    Complex lambda_ij(0,0);
    int n_datapoints = H.size();
    int n_frequency = H[0].size();
    int num_aps = H[0][0].size();
    int n_rx_ant = H[0][0][0].size();
    int n_tx_ant = H[0][0][0][0].size();

    for (int n = -N/2; n<= N/2 -1; ++n){
        for (int m = 1; m <= n_rx_ant; ++m){
            double fn = f0+ n * (c/N);
            Complex alpha_m = polar(1.0, 2.0*M_PI*f0*(m-1)*d*sin(z_i)/c);
            Complex tau_n = polar(1.0, 2.0*M_PI*fn*l_j/c);

            Complex H_nm = H[i][n + N/2][j][m-1][0]; // first antenna
            lambda_ij += H_nm * alpha_m *tau_n;
        }
    }
    return lambda_ij;
}
double direct_bearing(const ChannelMatrix& H, double f0, double c, int N, double d, int i, int j){
    int n_z_i_steps = 360;
    int n_l_j_steps = 250;
    double z_i_min = -M_PI;
    double z_i_max = M_PI;
    double l_j_min = 0;
    double l_j_max = 25;

    vector<pair<double, double>> local_maxima;

    for (int z_i_step = 0; z_i_step <= n_z_i_steps; ++z_i_step){
        double z_i = z_i_min + (z_i_max - z_i_min)* z_i_step/(n_z_i_steps);

        double max_magnitude = -1;
        double max_l_j = l_j_min;

        for (int l_j_step = 0; l_j_step <= n_l_j_steps; ++l_j_step){
            double l_j = l_j_min + (l_j_max - l_j_min) * l_j_step /(n_l_j_steps);
            Complex lambda_ij = calculate_lambda_ij(H, f0, c, N, d, i, j, z_i, l_j);
            double magnitude = abs(lambda_ij);

            if (magnitude > max_magnitude){
                max_magnitude = magnitude;
                max_l_j = l_j;
            }
        }

        local_maxima.push_back({z_i, max_l_j});
    }
    double smallest_l_j = numeric_limits<double>::max();
    double z_i_with_smallest_l_j = z_i_min;

    for (const auto& maxima: local_maxima){
        if (maxima.second < smallest_l_j){
            smallest_l_j = maxima.second;
            z_i_with_smallest_l_j = maxima.first;
        }
    }
    return z_i_with_smallest_l_j;
}

