/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-11-18 10:29:24
 * @LastEditors: windzu windzu1@gmail.com
 * @LastEditTime: 2023-11-18 16:09:38
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */
#include "lily/icp.h"

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>

#include <limits>
Eigen::Vector3f ICP::rotationMatrixToEulerAngles(Eigen::Matrix3d& R) {
  // assert(isRotationMatrix(R));
  float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

  bool singular = sy < 1e-6;  // If

  float x, y, z;
  if (!singular) {
    x = atan2(R(2, 1), R(2, 2));
    y = atan2(-R(2, 0), sy);
    z = atan2(R(1, 0), R(0, 0));
  } else {
    x = atan2(-R(1, 2), R(1, 1));
    y = atan2(-R(2, 0), sy);
    z = 0;
  }
  return Eigen::Vector3f(x, y, z);
}
ICP::ICP() {
  all_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  all_octree_.reset(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(0.05));
  all_octree_->setInputCloud(all_cloud_);
}

void ICP::process() {
  return;
}

void ICP::SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& gcloud,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& ngcloud,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  tgt_gcloud_ = gcloud;
  tgt_ngcloud_ = ngcloud;
  tgt_cloud_ = cloud;
}

void ICP::SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& gcloud,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& ngcloud,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  src_gcloud_ = gcloud;
  src_ngcloud_ = ngcloud;
  src_cloud_ = cloud;
}

Eigen::Matrix4d ICP::GetFinalTransformation() {
  return final_transformation_;
}

Eigen::Matrix4d GetDeltaT(const float yaw) {
  Eigen::Matrix3d deltaR = Eigen::Matrix3d(
      Eigen::AngleAxisd(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));

  Eigen::Matrix4d deltaT = Eigen::Matrix4d::Identity();
  deltaT.block<3, 3>(0, 0) = deltaR;
  return deltaT;
}

bool ICP::RegistrationByICP(const Eigen::Matrix4d& init_guess,
                            Eigen::Matrix4d& transform) {
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(tgt_ngcloud_);
  double cur_yaw = 0;
  double min_error = CalculateICPError(kdtree, init_guess, cur_yaw);
  double best_yaw = cur_yaw;
  float degree_2_radian = 0.017453293;
  int iter_cnt = 0;
  double step = 5;  // Resolution is 5°
  int search_range = 10;
  while (iter_cnt < 5) {
    for (int delta = -search_range; delta < search_range; delta++) {
      double yaw = cur_yaw + delta * step * degree_2_radian;
      double error = CalculateICPError(kdtree, init_guess, yaw);
      if (error < min_error) {
        min_error = error;
        best_yaw = yaw;
      }
    }
    search_range = static_cast<int>(search_range / 2 + 0.5);
    step /= 2;
    cur_yaw = best_yaw;
    iter_cnt++;
  }
  Eigen::Matrix4d T = GetDeltaT(best_yaw);
  T = T * init_guess;
  transform = T;
  return true;
}

double ICP::CalculateICPError(const pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
                              const Eigen::Matrix4d& init_guess,
                              float cur_yaw) {
  Eigen::Matrix4d T = GetDeltaT(cur_yaw) * init_guess;
  pcl::PointCloud<pcl::PointXYZI> trans_cloud;
  // T(0, 2) = 0;
  // T(1, 2) = 0;
  // T(2, 0) = 0;
  // T(2, 1) = 0;
  // T(2, 2) = 1;
  T(2, 3) = 0;
  Eigen::Matrix<double, 3, 3> rot = T.block<3, 3>(0, 0);
  Eigen::Vector3f euler = rotationMatrixToEulerAngles(rot);
  // 计算旋转矩阵的Z分量
  Eigen::Matrix3d R;
  R << cos(euler[2]), -sin(euler[2]), 0.0, sin(euler[2]), cos(euler[2]), 0.0,
      0.0, 0.0, 1.0;
  T.block<3, 3>(0, 0) = R;  //去除刚性变换中与Z轴方向有关的旋转和平移因素
  pcl::transformPointCloud(*src_ngcloud_, trans_cloud, T);
  double dist_sum = 0;
  for (size_t j = 0; j < trans_cloud.points.size(); j++) {
    std::vector<int> indices;
    std::vector<float> distances;
    int k = 1;
    pcl::PointXYZI point = trans_cloud.points[j];
    int size = kdtree.nearestKSearch(point, k, indices, distances);
    if (distances.size() > 0) {
      dist_sum += distances[0];
    } else {
      std::cout << "no nearest neighbors found" << std::endl;
    }
  }
  return dist_sum;
}

bool ICP::RegistrationByICP2(const Eigen::Matrix4d& init_guess,
                             Eigen::Matrix4d& refined_extrinsic) {
  pcl::PointCloud<pcl::PointXYZI> trans_cloud;
  pcl::transformPointCloud(*src_cloud_, trans_cloud, init_guess);

  std::cout << "compute normals of source points cloud." << std::endl;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_before_normal(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  computeNormals(trans_cloud.makeShared(), cloud_before_normal);

  std::cout << "compute normals of target points cloud." << std::endl;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt_normal(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  computeNormals(tgt_cloud_, cloud_tgt_normal);

  pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal,
                                        pcl::PointXYZINormal>
      icp;
  icp.setInputSource(cloud_before_normal);
  icp.setInputTarget(cloud_tgt_normal);
  icp.setMaximumIterations(8);
  // icp.setMaxCorrespondenceDistance(1.0);  // 1.5m
  icp.setMaxCorrespondenceDistance(0.1);  // 1.5m
  pcl::PointCloud<pcl::PointXYZINormal> cloud_out;
  std::cout << "align start" << std::endl;
  icp.align(cloud_out);
  std::cout << "align end" << std::endl;
  Eigen::Matrix4f transform = icp.getFinalTransformation();

  refined_extrinsic = transform.cast<double>() * init_guess;
  Eigen::Matrix<double, 3, 3> rot = refined_extrinsic.block<3, 3>(0, 0);
  Eigen::Vector3f euler = rotationMatrixToEulerAngles(rot);
  // 计算旋转矩阵的Z分量
  Eigen::Matrix3d R;
  R << cos(euler[2]), -sin(euler[2]), 0.0, sin(euler[2]), cos(euler[2]), 0.0,
      0.0, 0.0, 1.0;
  refined_extrinsic.block<3, 3>(0, 0) =
      R;  //去除刚性变换中与Z轴方向有关的旋转和平移因素
  // refined_extrinsic(0, 2) = 0;
  // refined_extrinsic(1, 2) = 0;
  // refined_extrinsic(2, 0) = 0;
  // refined_extrinsic(2, 1) = 0;
  // refined_extrinsic(2, 2) = 1;
  refined_extrinsic(2, 3) = 0;
  return true;
}

void ICP::computeNormals(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pts,
                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr out_pts) {
  pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(20);
  // norm_est.setRadiusSearch(5);
  norm_est.setInputCloud(in_pts);
  norm_est.compute(*out_pts);

  std::cout << "normal point cloud number: " << out_pts->size() << std::endl;
  for (int i = 0; i < out_pts->size(); ++i) {
    (*out_pts)[i].x = (*in_pts)[i].x;
    (*out_pts)[i].y = (*in_pts)[i].y;
    (*out_pts)[i].z = (*in_pts)[i].z;
  }
}
