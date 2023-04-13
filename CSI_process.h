#ifndef CSI_PROCESS_H
#define CSI_PROCESS_H

#include <complex>
#include <vector>

using namespace std;

typedef complex<double> Complex;
typedef vector<vector<vector<vector<vector<Complex>>>>> ChannelMatrix;

Complex calculate_lambda_ij(const ChannelMatrix& H, double f0, double c, int N, double d, int i, int j, double z_i, double l_j);
double direct_bearing(const ChannelMatrix& H, double f0, double c, int N, double d, int i, int j );

#endif // CSI_PROCESS_H