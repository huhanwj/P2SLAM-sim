#ifndef NOISE_MODEL_H
#define NOISE_MODEL_H

#include <complex>
#include <vector>

using namespace std;

typedef complex<double> Complex;
typedef vector<vector<vector<vector<vector<Complex>>>>> ChannelMatrix;

ChannelMatrix reconstruct_CSI(const ChannelMatrix& H, size_t i, size_t j, double f0, double c, int N, double d, double z);
//TODO: model the bearing noise from the reconstructed CSI
#endif