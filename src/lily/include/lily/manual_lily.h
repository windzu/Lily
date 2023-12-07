/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-07-05 11:38:22
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-12-07 10:26:09
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */

#pragma once
#include <dynamic_reconfigure/server.h>
#include <lily/dynamicConfig.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "boost/thread.hpp"
#include "lily/utils.h"

class ManualLily {
 public:
  ManualLily(ros::NodeHandle nh, ros::NodeHandle pnh);
  bool init();
  void run();

  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg,
                const std::string& topic_name);
  void trans_and_pub();
  void dynamic_config_callback(dynamic_tf_config::dynamicConfig config);
  void flash_status_bar();
  void save_config();

 private:
  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string config_path_;
  std::string main_topic_ = "";

  int min_points_num_ = 4;  // estimate plane need at least 4 points

  YAML::Node config_;

  // variables
  std::vector<ros::Subscriber> subs_;
  std::unordered_map<std::string, ros::Publisher> pubs_map_;

  std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr>
      cloud_map_;
  std::unordered_map<std::string, Eigen::Matrix4d> tf_matrix_map_;
  std::unordered_map<std::string, std::vector<pcl::PointXYZ>> points_map_;

  // dynamic reconfigure
  boost::recursive_mutex mutex_;
  std::shared_ptr<
      dynamic_reconfigure::Server<dynamic_tf_config::dynamicConfig>>
      server_;
  dynamic_reconfigure::Server<dynamic_tf_config::dynamicConfig>::CallbackType
      server_f_;
  std::string last_topic_name_ = "";
  std::unordered_map<std::string, dynamic_tf_config::dynamicConfig>
      dynamic_config_map_;
  bool flash_status_bar_flag_ = false;
  dynamic_tf_config::dynamicConfig temp_config_;
};
