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
#include <vector>

void parse_config(std::string config_path) {
  // 1. load config
  YAML::Node config = YAML::LoadFile(config_path);

  // 2, iter config_
  for (auto iter = config.begin(); iter != config.end(); iter++) {
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

    // find main topic
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

    // load points
    points_map_[topic] = std::vector<pcl::PointXYZ>();
    if (iter->second["use_points"]) {
      // subscribe /clicked_point topic
      ros::Subscriber sub = nh_.subscribe<geometry_msgs::PointStamped>(
          "/clicked_point", 1,
          boost::bind(&Lily::clicked_point_callback, this, _1, topic));

      // publish transformed cloud and selected points from cloud
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
    }

    ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
        topic, 1, boost::bind(&Lily::callback, this, _1, topic));
    ros::Publisher pub =
        nh_.advertise<sensor_msgs::PointCloud2>(topic + "/calibrated", 1);

    subs_.push_back(sub);
    pubs_map_[topic] = pub;
    tf_matrix_map_[topic] = tf_matrix;
  }

  return;
}

std::vector<double> quaternion_to_euler_angles(const std::vector<double>& q) {
  Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
  quat.normalize();  // 正规化四元数以确保其表示有效的旋转
  Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);
  std::vector<double> euler_angles_vec(
      euler_angles.data(), euler_angles.data() + euler_angles.size());
  return euler_angles_vec;
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

Eigen::Vector3d compute_plane_normal_vector(
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

  Eigen::Vector3d normal_vector;
  normal_vector[0] = coefficients->values[0];
  normal_vector[1] = coefficients->values[1];
  normal_vector[2] = coefficients->values[2];

  return normal_vector;
}
