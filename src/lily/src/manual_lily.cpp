/*
 * @Author: wind windzu1@gmail.com
 * @Date: 2023-12-06 18:36:33
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-12-06 19:25:37
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */
#include "lily/manual_lily.h"

ManualLily::ManualLily(ros::NodeHandle nh, ros::NodeHandle pnh) {
  nh_ = nh;
  pnh_ = pnh;

  pnh_.param<std::string>("config_path", config_path_, "");

  init();
}

bool ManualLily::init() {
  // 1. load config
  config_ = YAML::LoadFile(config_path_);

  // 2, iter config_
  for (auto iter = config_.begin(); iter != config_.end(); iter++) {
    std::string topic = iter->first.as<std::string>();

    // translation is a vector of [x, y, z]
    // rotation is a vector of [w, x, y, z]
    // rotation_euler is a vector of [roll, pitch, yaw]
    std::vector<double> translation =
        iter->second["transform"]["translation"].as<std::vector<double>>();
    std::vector<double> rotation =
        iter->second["transform"]["rotation"].as<std::vector<double>>();
    std::vector<double> rotation_euler =
        iter->second["transform"]["rotation_euler"].as<std::vector<double>>();

    // check if update rotation_euler from rotation
    if (rotation[0] != 1 || rotation[1] != 0 || rotation[2] != 0 ||
        rotation[3] != 0) {
      rotation_euler = quaternion_to_euler_angles(rotation);
    }
    // check if update rotation from rotation_euler
    if (rotation[0] == 1 && rotation[1] == 0 && rotation[2] == 0 &&
        rotation[3] == 0) {
      if (rotation_euler[0] != 0 || rotation_euler[1] != 0 ||
          rotation_euler[2] != 0) {
        rotation = euler_angles_to_quaternion(rotation_euler);
      }
    }

    Eigen::Matrix4d tf_matrix =
        calculate_tf_matrix_from_translation_and_rotation(translation,
                                                          rotation);

    // set dynamic_config_map_
    dynamic_tf_config::dynamicConfig config;
    config.lidar_topic = topic;
    config.x = translation[0];
    config.y = translation[1];
    config.z = translation[2];
    config.roll = rotation_euler[0];
    config.pitch = rotation_euler[1];
    config.yaw = rotation_euler[2];
    dynamic_config_map_[topic] = config;

    // check if need load from pcd file
    bool load_from_file = iter->second["load_from_file"].as<bool>();
    if (load_from_file) {
      std::string file_path = iter->second["file_path"].as<std::string>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1) {
        ROS_ERROR("load pcd file %s failed", file_path.c_str());
        return false;
      }
      cloud_map_[topic] = cloud;
    } else {
      cloud_map_[topic] = nullptr;
    }

    ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
        topic, 1, boost::bind(&ManualLily::callback, this, _1, topic));
    ros::Publisher pub =
        nh_.advertise<sensor_msgs::PointCloud2>(topic + "/calibrated", 1);

    subs_.push_back(sub);
    pubs_map_[topic] = pub;
    tf_matrix_map_[topic] = tf_matrix;
  }

  return true;
}
