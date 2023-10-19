/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-06-16 16:13:00
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-10-19 13:50:04
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */
#pragma once
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <iostream>

class GroundPlaneFit {
 public:
  GroundPlaneFit(double sensor_height, int num_iter, int num_lpr, double th_seeds, double th_dist);

  ~GroundPlaneFit() = default;

  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> process(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);

  Eigen::MatrixXf normal_;
  Eigen::Vector3f seeds_mean;

 private:
  double sensor_height_;
  int num_iter_;
  int num_lpr_;
  double th_seeds_;
  double th_dist_;

  float th_dist_d_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr g_seeds_pc_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_ground_pc_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_not_ground_pc_;

  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<pcl::PointXYZ>& p_sorted);
};
