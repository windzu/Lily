/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-06-16 16:13:00
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-12-07 10:44:17
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */
/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "lily/ground_segmentation.h"
#include "lily/utils.h"

class AutoCalibrator {
 public:
  AutoCalibrator(int num_iter, int num_lpr, double th_seeds, double th_dist)
      : num_iter_(num_iter),
        num_lpr_(num_lpr),
        th_seeds_(th_seeds),
        th_dist_(th_dist) {
  }

  std::unordered_map<std::string, Eigen::Matrix4d> process(
      const std::unordered_map<
          std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud_map,
      const std::string& main_topic,
      const std::unordered_map<std::string, bool>& need_calibration_map,
      const std::unordered_map<std::string, Eigen::Matrix4d>& tf_matrix_map);

  bool icpn();
  void stiching();

  std::unordered_map<std::string, Eigen::Matrix4d> calib_map_;

 private:
  // 地面提取
  pcl::ModelCoefficients::Ptr ground_plane_extraction(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_cloud,
      double sensor_height);

 private:
  // const double sensor_height = 1.5;
  int num_iter_ = 50;
  int num_lpr_ = 500;
  double th_seeds_ = 0.06;
  double th_dist_ = 0.06;

  std::string main_topic_;
  std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr>
      cloud_map_;
  std::unordered_map<std::string, Eigen::Matrix4d> tf_matrix_map_;
  std::unordered_map<std::string, bool> need_calibration_map_;

  YAML::Node config_;
  std::map<std::string, Eigen::Matrix4d> extrinsic_map_;
};
