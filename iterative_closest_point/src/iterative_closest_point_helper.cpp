#include <iostream>
#include <vector>
#include <numeric>
#include <string>
#include <map>
#include <cmath>
#include "iterative_closest_point_helper.hpp"

std::vector<double> calculateCentroid(std::vector<std::vector<double> > input_array){

    // Determine centroid of a data set.

    double x = 0;
    double y = 0;
    double z = 0;

    for (auto element : input_array){
        x += element[0];
        y += element[1];
        z += element[2];
    }
    x = x / input_array.size();
    y = y / input_array.size();
    z = z / input_array.size();

    return {x,y,z};
}


std::vector<double> calculateDistanceVector(std::vector<double> first_point, std::vector<double> second_point){

    // Determines the distance that second_point is away from first_point.
    std::vector<double> differences = std::vector<double>(3); 
    for (int i = 0; i < 3; i++){
        differences[i] = second_point[i] - first_point[i];
    }

    return differences;
}

double calculateDistanceMagnitude(std::vector<double> first_point, std::vector<double> second_point){

    // Determine magnitude of distance between two points.
    std::vector<double> differences = std::vector<double>(3); 
    for (int i = 0; i < 3; i++){
        differences[i] = second_point[i] - first_point[i];
    }
    double distance_magnitude = sqrt(pow(differences[0],2)+pow(differences[1],2)+pow(differences[2],2));

    return distance_magnitude;
}


std::vector<std::vector<double> > applyOffsetDistance(std::vector<std::vector<double> > &input_array, std::vector<double> offset_by){

    // Iterate through the input_array and modify it by migrating its points by the offest distance.
    int looper = 0;
    for (auto element : input_array){
        for (int j = 0; j < 3; j++){
            input_array[looper][j] = element[j] + offset_by[j];
        }
        looper++;
    }

    return input_array;
}

Eigen::Matrix3d EigenTools::calculateRotation(PointCloud truth_data, PointCloud measurement_data){
    
    // Initialize the cross covariance matrix.
    Eigen::Matrix<double,3,3> cross_covariance_matrix;
    cross_covariance_matrix.setZero();

    // Use the closest_points member of the PointClouds to calculate cross-covariance matrix.
    for (int i = 0; i < measurement_data._point_cloud.size(); i++){
        
        // Reinitialize a zero'd out temporary matrix that iteratively sums into the final cross-cov mat.
        Eigen::Matrix<double,3,3> temporary_matrix;
        temporary_matrix.setZero();

        // Construct centroid-to-point difference vectors.
        std::vector<double> meas_diff = calculateDistanceVector(measurement_data._centroid,measurement_data._point_cloud[i]);
        std::vector<double> truth_diff = calculateDistanceVector(truth_data._centroid,truth_data._point_cloud[measurement_data._closest_points[i]]);

        for (int j = 0; j < 3; j++){
            temporary_matrix(j,0) = truth_diff[j]*meas_diff[0];
            temporary_matrix(j,1) = truth_diff[j]*meas_diff[1];
            temporary_matrix(j,2) = truth_diff[j]*meas_diff[2];
        }

        cross_covariance_matrix = cross_covariance_matrix + temporary_matrix;
    }

    // SVD of the cross-covariance matrix.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cross_covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d rotation_matrix = svd.matrixU() * svd.matrixV().transpose(); 
    return rotation_matrix;
}

std::vector<std::vector<std::vector<double> > > inputDataHandling(std::string data_file_name){

    // Initialize storage vectors.
    std::vector<std::vector<std::vector<double> > > data_sets;
    std::vector<std::vector<double> > curr_data_storage;

    // Bring input data into c++.
    std::ifstream inputFile(data_file_name);
    std::string line;

    // Quick check for whether the data input file was found and brought in correctly.
    if (!inputFile.is_open()) { 
        std::cerr << "Error opening the file!" << std::endl; 
    } else {
        getline(inputFile,line); // First line is a burner
    }

    int finder,finder2;
    
    while (getline(inputFile, line)){

        if (line.find("NaN") == std::string::npos){
            std::vector<double> curr_line = std::vector<double>(3);

            // Find the spaces.
            finder = line.find("\t");
            finder2 = line.find("\t",finder+1);

            // Toss the data into the curr_data vector.
            curr_line[0] = std::stod(line.substr(0,finder));
            curr_line[1] = std::stod(line.substr(finder+1,finder2));
            curr_line[2] = std::stod(line.substr(finder2+1,line.size()-1));
            curr_data_storage.push_back(curr_line);
        } else {
            // At this point all truth data is snagged, so store it.
            data_sets.push_back(curr_data_storage);
            curr_data_storage = {};
        }
    }
    data_sets.push_back(curr_data_storage);

    return data_sets;
}

void findClosestPoints(PointCloud &truth_data, PointCloud &measurement_data){

    // Loop on truth data and measurement data, iteratively finding closest point.
    for (int i = 0; i < measurement_data._point_cloud.size(); i++){

        // Fine to initialize these on each loop.
        int min_dist_idx = -1; 
        double min_dist = -1;
        double curr_dist;

        for (int j = 0; j < truth_data._point_cloud.size(); j++){
            curr_dist = calculateDistanceMagnitude(measurement_data._point_cloud[i],truth_data._point_cloud[j]);
            if (min_dist_idx > -1){
                min_dist_idx = (curr_dist < min_dist) ? j : min_dist_idx;
                min_dist = (curr_dist < min_dist) ? curr_dist : min_dist;
            } else{
                min_dist_idx = j;
                min_dist = curr_dist;
            }
        }
        measurement_data._closest_points[i] = min_dist_idx;
        truth_data._closest_points[min_dist_idx] = i;
    }
}

Eigen::Matrix4d EigenTools::stockTransform(Eigen::Matrix3d rotation_matrix, std::vector<double> curr_translation){

    Eigen::Matrix4d transformation_matrix = Eigen::MatrixXd::Identity(4,4);

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            transformation_matrix(i,j) = rotation_matrix(i,j);
        }
        transformation_matrix(i,3) = curr_translation[i];
    }
    return transformation_matrix;
}

void applyTransformToData(Eigen::Matrix4d curr_trans,PointCloud &measurement_data){

    // Refresh the measurement data.
    for (int i = 0; i < measurement_data._point_cloud.size(); i++){
        Eigen::Matrix<double,4,1> curr_point = {measurement_data._point_cloud[i][0], measurement_data._point_cloud[i][1], measurement_data._point_cloud[i][2],1};
        Eigen::Matrix<double,4,1> updated_point = curr_trans * curr_point;
        for (int j = 0; j < 3; j++){
            measurement_data._point_cloud[i][j] = updated_point(j);
        }
    }

    // Refresh the centroid.
    measurement_data._centroid = calculateCentroid(measurement_data._point_cloud);

}

double calculateMaxError(PointCloud truth_data,PointCloud measurement_data){

    double max_error = 1;
    double curr_error;

    for (int i = 0; i < measurement_data._point_cloud.size(); i++){
        curr_error = calculateDistanceMagnitude(measurement_data._point_cloud[i], truth_data._point_cloud[measurement_data._closest_points[i]]);
        if (max_error == 1){
            max_error = curr_error;
        } else if (curr_error > max_error){
            max_error = curr_error;
        }
    }

    return max_error;
}


// MAIN FUNCTION CALL

Eigen::Matrix4d executeICP(PointCloud truth_data,PointCloud measurement_data, double err_threshold){

    // Initialize the homogeneous transform.
    Eigen::Matrix4d final_transform = Eigen::MatrixXd::Identity(4,4);

    // Initialize curr_translation and curr_rotation types.
    std::vector<double> curr_translation;
    std::vector<std::vector<double> > curr_rotation;

    // Initialize current error (maximum distance between a measurement point and its targeted "true" point).
    double curr_err = 1;

    while (curr_err > err_threshold){

        // STEP 1: Determine distance between centroids.
        curr_translation = calculateDistanceVector(measurement_data._centroid,truth_data._centroid);

        // STEP 2: Determine closest points (this is being done extremely brute force).
        findClosestPoints(truth_data,measurement_data);

        // STEP 3: Determine rotation matrix via SVD of cross-covariance matrix.
        Eigen::Matrix3d rotation_matrix = EigenTools::calculateRotation(truth_data, measurement_data);

        // STEP 4: Assemble the transform.
        Eigen::Matrix4d curr_trans = EigenTools::stockTransform(rotation_matrix,curr_translation);

        // STEP 5: Apply transform to measurement data.
        applyTransformToData(curr_trans,measurement_data);

        // STEP 6: Stack transform onto previous iteration's transform.
        final_transform = curr_trans * final_transform;

        // DEBUG STEP
        std::cout << Eigen::Matrix4d(final_transform) << std::endl;

        // STEP 6: Calculate current error.
        curr_err = calculateMaxError(truth_data,measurement_data);
    }

    return final_transform;
}