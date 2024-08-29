#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <fstream>
#include <Eigen/Dense>
    
// Functions
std::vector<std::vector<std::vector<double> > > inputDataHandling(std::string data_file_name);
std::vector<double> calculateCentroid(std::vector<std::vector<double> > input_array);
std::vector<double> calculateDistanceVector(std::vector<double> first_point, std::vector<double> second_point);
std::vector<std::vector<double> > applyOffsetDistance(std::vector<std::vector<double> > &input_array, std::vector<double> offset_by);
double calculateDistanceMagnitude(std::vector<double> first_point, std::vector<double> second_point);

// Will use a class to house point clouds and their centroid information.
class PointCloud{
    public:

        // Constructor.
        PointCloud(std::vector<std::vector<double> > input_data){
            _point_cloud = input_data;
            _centroid = calculateCentroid(input_data);
            for (int i = 0; i < input_data.size(); i++){
                _closest_points[i] = -1;
            }
        }

        std::vector<std::vector<double> > _point_cloud; // vector of vectors (x, y, z)
        std::vector<double> _centroid; // (x, y, z)
        std::map<int, int> _closest_points; // <index of THIS cloud, index of THAT cloud>
    private:
};

// More functions. 
Eigen::Matrix4d executeICP(PointCloud truth_data,PointCloud measurement_data, double err_threshold);
void findClosestPoints(PointCloud &truth_data, PointCloud &measurement_data);
double calculateMaxError(PointCloud truth_data,PointCloud measurement_data);

// Strange hack needed here, the calculateCrossCovariance function was being seen as ambiguous if I didn't encapsulate it in a 
// namespace to define that the call to it was DEFINITELY this function.
namespace EigenTools{
    Eigen::Matrix3d calculateRotation(PointCloud truth_data, PointCloud measurement_data);
    Eigen::Matrix4d stockTransform(Eigen::Matrix3d rotation_matrix, std::vector<double> curr_translation);
    void applyTransformToData(Eigen::Matrix4d curr_trans,PointCloud &measurement_data);
}