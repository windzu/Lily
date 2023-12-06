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

    // load points from clicked_point topic
    // 1. subscribe /clicked_point topic and publish transformed cloud
    // 2. selected points from cloud mannually
    // 3. wait until enough points are selected
    // 4. calculate tf_matrix from points (estimate plane)
    points_map_[topic] = std::vector<pcl::PointXYZ>();
    if (iter->second["use_points"]) {
      // subscribe /clicked_point topic
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
    }

    tf_matrix_map_[topic] = tf_matrix;
  }

  return true;
}

void AutoLily::clicked_point_callback(
    const geometry_msgs::PointStamped::ConstPtr& msg,
    const std::string& topic_name) {
  if (points_map_.find(topic_name) == points_map_.end()) {
    ROS_ERROR("topic %s not in points_map_", topic_name.c_str());
    return;
  }

  pcl::PointXYZ point;
  point.x = msg->point.x;
  point.y = msg->point.y;
  point.z = msg->point.z;
  points_map_[topic_name].push_back(point);

  // ros info
  ROS_INFO("topic %s, point: (%f, %f, %f)", topic_name.c_str(), point.x,
           point.y, point.z);

  if (points_map_[topic_name].size() == min_points_num_) {
    // calculate tf_matrix (plane )
  }

  return;
}