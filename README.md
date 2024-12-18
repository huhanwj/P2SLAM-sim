# P2SLAM-sim
A simple cpp code to implement the algorithm used in **P2SLAM: Bearing Based WiFi SLAM for Indoor Robots** using GTSAM.

**Notice: This may not work and I am not maintaining it!!!**

## Intro
* `APRobotFactors`: implementation of the "ping-pong" factors connected between the APs and robot poses.
* `CSI_process`: implementation of the 2D-FFT-based bearing estimation algorithm used to find direct path bearing.
* `RobotBetweenFactor`: implementation of the wheel odometry factors between robot poses.
* `noise_model`: implementation of the mathematic model of the proposed noise model in P2SLAM for bearing noise estimation
* `general_functions`: Data IO from/to CSV data and MAT data.
* `data` folder: P2SLAM dataset and visualization code.
