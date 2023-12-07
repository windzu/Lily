/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-07-05 11:38:22
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-12-07 10:20:03
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */

#pragma once

#include <ros/ros.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "lily/auto_lily.h"
#include "lily/manual_lily.h"

class Lily {
 public:
  Lily(ros::NodeHandle nh, ros::NodeHandle pnh);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<ManualLily> manual_lily_;
  std::unique_ptr<AutoLily> auto_lily_;
};
