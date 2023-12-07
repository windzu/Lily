/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-12-06 23:12:01
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-12-07 10:11:24
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */
#include "lily/auto_lily.h"

AutoLily::AutoLily(ros::NodeHandle nh, ros::NodeHandle pnh) {
  nh_ = nh;
  pnh_ = pnh;

  pnh_.param<std::string>("config_path", config_path_, "");

  init();
}

bool AutoLily::init() {
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
    tf_matrix_map_[topic] = tf_matrix;

    // find main topic
    // Note : auto mode need know which topic is main topic
    //       because main topic will not change yaw
    bool is_main = iter->second["is_main"].as<bool>();
    if (is_main) {
      main_topic_ = topic;
    }

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

    // load if need calibration
    need_calibration_map_[topic] = iter->second["need_calibration"].as<bool>();

    // load points from clicked_point topic
    // 1. subscribe /clicked_point topic and publish transformed cloud
    // 2. selected points from cloud mannually
    // 3. wait until enough points are selected
    // 4. calculate tf_matrix from points (estimate plane)
    points_map_[topic] = std::vector<pcl::PointXYZI>();
    if (iter->second["use_points"]) {
      ros::Subscriber sub = nh_.subscribe<geometry_msgs::PointStamped>(
          "/clicked_point", 1,
          boost::bind(&AutoLily::clicked_point_callback, this, _1, topic));

      ros::Publisher pub =
          nh_.advertise<sensor_msgs::PointCloud2>(topic + "/calibrated", 1);

      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*(cloud_map_[topic]), *transformed_cloud,
                               tf_matrix);
      sensor_msgs::PointCloud2::Ptr pc_msg(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*transformed_cloud, *pc_msg);

      pc_msg->header.frame_id = "base_link";

      // publish
      pub.publish(pc_msg);

      // ros spin
      ros::Rate rate(10);
      while (ros::ok() && points_map_[topic].size() < min_points_num_) {
        pub.publish(pc_msg);
        ros::spinOnce();
        rate.sleep();
      }

      // when enough points are selected
      // calculate tf_matrix from points
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      for (auto point : points_map_[topic]) {
        cloud->push_back(point);
      }
      pcl::ModelCoefficients::Ptr coefficients = compute_plane(cloud);
      Eigen::Vector3d real_ground_normal_vector(0, 0, 1);
      Eigen::Vector3d estimate_ground_normal_vector;
      estimate_ground_normal_vector[0] = coefficients->values[0];
      estimate_ground_normal_vector[1] = coefficients->values[1];
      estimate_ground_normal_vector[2] = coefficients->values[2];
      Eigen::Matrix4d rotation_matrix =
          calculate_rotation_matrix4d_from_two_vectors(
              estimate_ground_normal_vector, real_ground_normal_vector);
      // update tf_matrix
      tf_matrix_map_[topic] = rotation_matrix * tf_matrix;

      // set need_calibration to false because we have calibrated
      need_calibration_map_[topic] = false;
    }
  }

  return true;
}

void AutoLily::run() {
  std::cout << "-------------------------" << std::endl;
  std::cout << "Collection" << std::endl;

  ros::Rate rate(10);
  while (ros::ok() && !cloud_map_full_check(cloud_map_)) {
    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "-------------------------" << std::endl;
  std::cout << "Calibration" << std::endl;

  // calibration
  auto_calibrator_.reset(
      new AutoCalibrator(num_iter_, num_lpr_, th_seeds_, th_dist_));
  tf_matrix_map_ = auto_calibrator_->process(
      cloud_map_, main_topic_, need_calibration_map_, tf_matrix_map_);

  std::cout << "-------------------------" << std::endl;
  std::cout << "Saving Result" << std::endl;
  save_config();
  ros::shutdown();
  return;
}

void AutoLily::clicked_point_callback(
    const geometry_msgs::PointStamped::ConstPtr& msg,
    const std::string& topic_name) {
  if (points_map_.find(topic_name) == points_map_.end()) {
    ROS_ERROR("topic %s not in points_map_", topic_name.c_str());
    return;
  }

  pcl::PointXYZI point;
  point.x = msg->point.x;
  point.y = msg->point.y;
  point.z = msg->point.z;
  points_map_[topic_name].push_back(point);

  // ros info
  ROS_INFO("topic %s, point: (%f, %f, %f)", topic_name.c_str(), point.x,
           point.y, point.z);

  return;
}

void AutoLily::callback(const sensor_msgs::PointCloud2::ConstPtr& msg,
                        const std::string& topic_name) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);
  cloud_map_[topic_name] = cloud;
  return;
}

void AutoLily::save_config() {
  for (auto iter = tf_matrix_map_.begin(); iter != tf_matrix_map_.end();
       iter++) {
    std::string topic = iter->first;
    Eigen::Matrix4d tf_matrix = iter->second;

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
