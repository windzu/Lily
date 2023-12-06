/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-06-16 17:34:42
 * @LastEditors: windzu windzu1@gmail.com
 * @LastEditTime: 2023-12-07 01:16:22
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */

#include "lily/lily.h"

Lily::Lily(ros::NodeHandle nh, ros::NodeHandle pnh) {
  nh_ = nh;
  pnh_ = pnh;

  pnh_.param<bool>("manual_mode", manual_mode_, "");
  if (manual_mode_) {
    manual_lily_.reset(new ManualLily(nh_, pnh_));
    manual_lily_->init();
  } else {
    auto_lily_.reset(new AutoLily(nh_, pnh_));
    auto_lily_->init();
  }

  if (!init()) {
    ROS_ERROR("init failed");
    return;
  }

  std::cout << "-------------------------" << std::endl;
  std::cout << "Collection" << std::endl;

  ros::Rate rate(10);
  while (ros::ok() && !cloud_map_full_check()) {
    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "-------------------------" << std::endl;
  std::cout << "Calibration" << std::endl;

  // debug
  // echo tf_matrix_map_
  std::cout << "before calibration:" << std::endl;
  std::cout << "-------------------------" << std::endl;
  for (auto iter = tf_matrix_map_.begin(); iter != tf_matrix_map_.end();
       iter++) {
    std::cout << iter->first << std::endl;
    std::cout << iter->second << std::endl;
  }
  std::cout << "-------------------------" << std::endl;

  // calibration
  calibrator_.reset(new Calibrator(num_iter_, num_lpr_, th_seeds_, th_dist_));
  tf_matrix_map_ = calibrator_->process(cloud_map_, main_topic_, points_map_,
                                        tf_matrix_map_);

  // debug
  std::cout << "after calibration:" << std::endl;
  std::cout << "-------------------------" << std::endl;
  for (auto iter = tf_matrix_map_.begin(); iter != tf_matrix_map_.end();
       iter++) {
    std::cout << iter->first << std::endl;
    std::cout << iter->second << std::endl;
  }
  std::cout << "-------------------------" << std::endl;

  std::cout << "-------------------------" << std::endl;
  std::cout << "Saving" << std::endl;
  // save
  save_config();
  ros::shutdown();
  return;
}

bool Lily::cloud_map_full_check() {
  for (auto iter = cloud_map_.begin(); iter != cloud_map_.end(); iter++) {
    if (iter->second == nullptr) {
      return false;
    }
  }
  return true;
}

void Lily::callback(const sensor_msgs::PointCloud2::ConstPtr& msg,
                    const std::string& topic_name) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);
  cloud_map_[topic_name] = cloud;
  return;
}
