#include "noise_model.h"
#include <complex>
#include <cmath>
#include <vector>

using namespace std;

ChannelMatrix reconstruct_CSI(const ChannelMatrix& H, size_t i, size_t j, double f0, double c, int N, double d, double z){
    ChannelMatrix reconstructed_CSI(1, vector<vector<vector<vector<Complex>>>>(H[i].size(), vector<vector<vector<Complex>>>(1, vector<vector<Complex>>(H[i][0][j].size(), vector<Complex>(H[i][0][j][0].size())))));

    for (size_t n = 0; n < H[i].size(); ++n){
        for (size_t m = 0; m < H[i][n][j].size(); ++m){
            for(size_t tx = 0; tx < H[i][n][j][m].size(); ++tx){
                double a_i = abs(H[i][n][j][m][tx]); // Get the magnitude of the CSI
                double exponent = -2.0*M_PI*f0*(m-1)*d*sin(z)/c;
                Complex exp_term = Complex(cos(exponent), sin(exponent));
                reconstructed_CSI[0][n][0][m][tx] = a_i * exp_term;
            }
        }
    }
    return reconstructed_CSI;

    ////TODO: model the bearing noise from the reconstructed CSI

}