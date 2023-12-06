/*
 * @Author: wind windzu1@gmail.com
 * @Date: 2023-12-06 18:36:33
 * @LastEditors: windzu windzu1@gmail.com
 * @LastEditTime: 2023-12-07 01:22:06
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

void ManualLily::run() {
  // init dynamic reconfigure
  server_.reset(
      new dynamic_reconfigure::Server<dynamic_tf_config::dynamicConfig>(mutex_,
                                                                        pnh_));
  server_f_ = boost::bind(&ManualLily::dynamic_config_callback, this, _1);
  server_->setCallback(server_f_);

  ros::Rate rate(10);
  while (ros::ok()) {
    flash_status_bar();
    trans_and_pub();
    ros::spinOnce();
    rate.sleep();
  }

  save_config();
  ros::shutdown();
  return;
}

void ManualLily::callback(const sensor_msgs::PointCloud2::ConstPtr& msg,
                          const std::string& topic_name) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);
  cloud_map_[topic_name] = cloud;
  return;
}

void ManualLily::dynamic_config_callback(
    dynamic_tf_config::dynamicConfig config) {
  std::string topic = config.lidar_topic;

  // check if need save config
  if (topic == "save") {
    save_config();
    return;
  }

  // check topic if in dynamic_config_map_
  if (dynamic_config_map_.find(topic) == dynamic_config_map_.end()) {
    ROS_ERROR("topic %s not in dynamic_config_map_", topic.c_str());
    return;
  }

  // update tf_matrix_map_
  // 切换topic时，是用对应lidar的参数更新状态栏
  if (topic != last_topic_name_) {
    ROS_WARN("change lidar_topic, will update tf_matrix_map_");

    last_topic_name_ = topic;
    flash_status_bar_flag_ = true;
    temp_config_ = dynamic_config_map_[topic];
    return;
  }

  // update dynamic_config_map_
  dynamic_config_map_[topic] = config;

  if (cloud_map_[topic] != nullptr) {
    Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
    Eigen::Translation3d tl(config.x, config.y, config.z);
    Eigen::AngleAxisd rot_x(config.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(config.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(config.yaw, Eigen::Vector3d::UnitZ());
    tf_matrix = (tl * rot_z * rot_y * rot_x).matrix();
    tf_matrix_map_[topic] = tf_matrix;
  }

  return;
}

void ManualLily::flash_status_bar() {
  if (flash_status_bar_flag_) {
    server_->updateConfig(temp_config_);
    flash_status_bar_flag_ = false;
  }
  return;
}

void ManualLily::trans_and_pub() {
  for (auto iter = cloud_map_.begin(); iter != cloud_map_.end(); iter++) {
    if (iter->second != nullptr) {
      std::string topic = iter->first;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*(iter->second), *cloud, tf_matrix_map_[topic]);
      sensor_msgs::PointCloud2::Ptr pc_msg(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloud, *pc_msg);
      pc_msg->header.frame_id = "base_link";
      pubs_map_[topic].publish(pc_msg);
    }
  }
  return;
}

void ManualLily::save_config() {
  for (auto iter = dynamic_config_map_.begin();
       iter != dynamic_config_map_.end(); iter++) {
    std::string topic = iter->first;
    dynamic_tf_config::dynamicConfig dynamic_config = iter->second;

    Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();
    Eigen::Translation3f tl(dynamic_config.x, dynamic_config.y,
                            dynamic_config.z);
    Eigen::AngleAxisf rot_x(dynamic_config.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(dynamic_config.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(dynamic_config.yaw, Eigen::Vector3f::UnitZ());
    tf_matrix = (tl * rot_z * rot_y * rot_x).matrix();

    // convert transform matrix to translation and rotation
    std::vector<double> translation_vec =
        transform_matrix_to_translation(tf_matrix.cast<double>());
    std::vector<double> quaternion_vec =
        transform_matrix_to_quaternion(tf_matrix.cast<double>());
    std::vector<double> euler_angles_vec =
        transform_matrix_to_euler_angles(tf_matrix.cast<double>());

    // round to 3 decimal places
    translation_vec = round_to_3_decimal_places(translation_vec);
    quaternion_vec = round_to_3_decimal_places(quaternion_vec);
    euler_angles_vec = round_to_3_decimal_places(euler_angles_vec);

    // save to config_
    config_[topic]["transform"]["translation"] = translation_vec;
    config_[topic]["transform"]["rotation"] = quaternion_vec;
    config_[topic]["transform"]["rotation_euler"] = euler_angles_vec;
  }

  // save config
  std::string save_path = config_path_ + "_" + current_date_time();
  std::ofstream fout(save_path);
  fout << config_;
  fout.close();

  ROS_INFO("save config success");
  return;
}