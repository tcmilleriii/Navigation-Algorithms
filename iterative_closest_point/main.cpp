#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <fstream>
#include "iterative_closest_point_helper.hpp"

// --------------------- ITERATIVE CLOSEST POINT ------------------------

// STEPS:
//  - Align the centers of mass of the two point clouds
//  - Calculate the cross covariance matrix H 
//      - H = sum(p_i - p_centroid)(q_i - q_centroid)'  <-- notice the points are grouped here, so the "closest" points defines what gets paired
//  - Calculate the singular value decomposition SVD(H) = UEV'
//  - Rotation = UV'
//  ITERATE!

// OBSERVATIONS
//  - The technique we're going to code up here assumes equivalent numbers of points for both P and Q but, potentially, we just don't know 
//    which point corresponds with which.

// INPUTS:
//  - Two corresponding point sets (clouds) P {p1, p2, ..., pn} and Q {q1, q2, ..., qn}
//  - Data will be input as a single text file with the following generalized formatting:
//      x1  y1  z1  |
//      x2  y2  z2  |
//          .       | Truth Data
//          .       |
//      xn  yn  zn  |
//      NaN NaN NaN <-- Separated by a row of NaNs
//      x1  y1  z1  |
//      x2  y2  z2  |
//          .       | Measurement Data
//          .       |
//      xn  yn  zn  |

// OUTPUTS:
//  - homogenous transformation matrix containing translation and rotation that minimize the sum of the squared error (point to point)
//  - final transform can be used to convert MEASURMENT DATA TO TRUTH DATA
// -----------------------------------------------------------------------

int main(){

    // Take in input data sets (truth and measurement data).
    std::string data_file_name = "./data/input_data/iterative_closest_point_data_17-Aug-2024.txt"; // USER CHANGES THIS PATH FOR DIFFERENT DATA SETS
    std::vector<std::vector<std::vector<double> > > input_data = inputDataHandling(data_file_name);

    // Get the truth and measurement data instantiated as PointCloud types.
    PointCloud truth_data = input_data[0];
    PointCloud measurement_data = input_data[1];

    // Define the targeted error threshold.
    double err_threshold = 0.05;

    // Execute Iterative Closest Point.
    Eigen::Matrix4d final_transform = executeICP(truth_data,measurement_data,err_threshold);

    // DEBUG STEP
    std::cout << Eigen::Matrix4d(final_transform) << std::endl;

    return 0;
}