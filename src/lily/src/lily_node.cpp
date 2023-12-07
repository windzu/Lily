/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-06-16 14:24:52
 * @LastEditors: windzu windzu1@gmail.com
 * @LastEditTime: 2023-10-19 02:01:10
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */

#include <ros/ros.h>

#include "lily/lily.h"

int main(int argc, char** argv) {
  ROS_WARN_STREAM("~~ ===========================================~~");
  ROS_WARN_STREAM("~~ =================   Lily  ================= ~~");
  ROS_WARN_STREAM("~~ =========================================== ~~");
  ros::init(argc, argv, "lily_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  Lily lily(nh, pnh);

  return 0;
}
