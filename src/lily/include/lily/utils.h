

#pragma once
#include <pcl/ModelCoefficients.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <vector>

Eigen::Matrix4d calculate_tf_matrix_from_translation_and_rotation(
    const std::vector<double>& translation,
    const std::vector<double>& rotation);

pcl::ModelCoefficients::Ptr compute_plane(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

// 获取两个法向量之间的转换刚性矩阵
Eigen::Matrix4d calculate_rotation_matrix4d_from_two_vectors(
    Eigen::Vector3d before, Eigen::Vector3d after);

std::vector<double> quaternion_to_euler_angles(const std::vector<double>& q);
Eigen::Matrix3d quaternion_to_rotation_matrix(const std::vector<double>& q);
std::vector<double> euler_angles_to_quaternion(
    const std::vector<double>& euler_angles);
std::vector<double> rotation_matrix_to_euler_angles(const Eigen::Matrix3d& R);
std::vector<double> rotation_matrix_to_quaternion(const Eigen::Matrix3d& R);
std::vector<double> transform_matrix_to_euler_angles(const Eigen::Matrix4d& T);
std::vector<double> transform_matrix_to_quaternion(const Eigen::Matrix4d& T);
std::vector<double> transform_matrix_to_translation(const Eigen::Matrix4d& T);

std::string current_date_time();

std::vector<double> round_to_3_decimal_places(const std::vector<double>& vec);

bool cloud_map_full_check(
    const std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr>
        cloud_map);
