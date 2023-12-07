/*
 * @Author: wind windzu1@gmail.com
 * @Date: 2023-12-07 10:30:41
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-12-07 11:47:44
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */
#include "lily/utils.h"

Eigen::Matrix4d calculate_tf_matrix_from_translation_and_rotation(
    const std::vector<double>& translation,
    const std::vector<double>& rotation) {
  Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();

  // calculate tf_matrix_map_ from translation and rotation(quat)
  // - translation is a vector of [x, y, z]
  // - rotation is a vector of [w, x, y, z]
  // - tf_matrix is a 4x4 matrix

  Eigen::Translation3d trans(translation[0], translation[1], translation[2]);
  Eigen::Quaterniond quat(rotation[0], rotation[1], rotation[2], rotation[3]);
  quat.normalize();  // 正规化四元数以确保其表示有效的旋转

  Eigen::Affine3d transform =
      Eigen::Translation3d(translation[0], translation[1], translation[2]) *
      quat;
  tf_matrix = transform.matrix();

  return tf_matrix;
}

pcl::ModelCoefficients::Ptr compute_plane(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(
      0.01);  // You might need to adjust this value based on your data

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
  }

  return coefficients;
}

// 获取两个法向量之间的转换刚性矩阵
Eigen::Matrix4d calculate_rotation_matrix4d_from_two_vectors(
    Eigen::Vector3d before, Eigen::Vector3d after) {
  before.normalize();
  after.normalize();

  double angle = acos(before.dot(after));
  Eigen::Vector3d p_rotate = before.cross(after);
  p_rotate.normalize();

  Eigen::Matrix4d rotation_matrix = Eigen::Matrix4d::Identity();
  rotation_matrix(0, 0) =
      cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
  // Note : 这里跟公式比多了一个括号，但是看实验结果它是对的
  rotation_matrix(0, 1) =
      p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));
  rotation_matrix(0, 2) =
      p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

  rotation_matrix(1, 0) =
      p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
  rotation_matrix(1, 1) =
      cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
  rotation_matrix(1, 2) =
      -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

  rotation_matrix(2, 0) =
      -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
  rotation_matrix(2, 1) =
      p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
  rotation_matrix(2, 2) =
      cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

  return rotation_matrix;
}

std::vector<double> quaternion_to_euler_angles(const std::vector<double>& q) {
  // 1. quaternion to rotation matrix
  // 2. rotation matrix to euler angles
  Eigen::Matrix3d rotation_matrix = quaternion_to_rotation_matrix(q);
  std::vector<double> euler_angles_vec =
      rotation_matrix_to_euler_angles(rotation_matrix);
  return euler_angles_vec;

  // Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
  // quat.normalize();
  // Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(0, 1,
  // 2); std::vector<double> euler_angles_vec(
  //     euler_angles.data(), euler_angles.data() + euler_angles.size());
  // return euler_angles_vec;
}

Eigen::Matrix3d quaternion_to_rotation_matrix(const std::vector<double>& q) {
  Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
  quat.normalize();
  Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
  return rotation_matrix;
}

std::vector<double> euler_angles_to_quaternion(
    const std::vector<double>& euler_angles) {
  Eigen::Quaterniond quat =
      Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());
  quat.normalize();

  Eigen::Vector4d quaternion =
      Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
  std::vector<double> quaternion_vec(quaternion.data(),
                                     quaternion.data() + quaternion.size());
  return quaternion_vec;
}

std::vector<double> rotation_matrix_to_euler_angles(const Eigen::Matrix3d& R) {
  double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

  double singular = sy < 1e-6;  // If

  double x, y, z;
  if (!singular) {
    x = atan2(R(2, 1), R(2, 2));
    y = atan2(-R(2, 0), sy);
    z = atan2(R(1, 0), R(0, 0));
  } else {
    x = atan2(-R(1, 2), R(1, 1));
    y = atan2(-R(2, 0), sy);
    z = 0;
  }

  std::vector<double> euler_angles = {x, y, z};

  return euler_angles;
}

std::vector<double> rotation_matrix_to_quaternion(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond quat(R);
  quat.normalize();  // 正规化四元数以确保其表示有效的旋转
  Eigen::Vector4d quaternion =
      Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
  std::vector<double> quaternion_vec(quaternion.data(),
                                     quaternion.data() + quaternion.size());
  return quaternion_vec;
}

std::vector<double> transform_matrix_to_euler_angles(
    const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  return rotation_matrix_to_euler_angles(R);
}

std::vector<double> transform_matrix_to_quaternion(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(R);
  quat.normalize();  // 正规化四元数以确保其表示有效的旋转
  Eigen::Vector4d quaternion =
      Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
  std::vector<double> quaternion_vec(quaternion.data(),
                                     quaternion.data() + quaternion.size());
  return quaternion_vec;
}

std::vector<double> transform_matrix_to_translation(const Eigen::Matrix4d& T) {
  Eigen::Vector3d translation = T.block<3, 1>(0, 3);
  std::vector<double> translation_vec(translation.data(),
                                      translation.data() + translation.size());
  return translation_vec;
}

std::string current_date_time() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
  return ss.str();
}

std::vector<double> round_to_3_decimal_places(const std::vector<double>& vec) {
  std::vector<double> round_vec = vec;
  for (int i = 0; i < vec.size(); i++) {
    round_vec[i] = std::round(vec[i] * 1000.0) / 1000.0;
  }
  return round_vec;
}

bool cloud_map_full_check(
    const std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr>
        cloud_map) {
  for (auto iter = cloud_map.begin(); iter != cloud_map.end(); iter++) {
    if (iter->second == nullptr) {
      return false;
    }
  }
  return true;
}