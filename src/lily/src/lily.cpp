/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-06-16 17:34:42
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-12-07 10:19:47
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */

#include "lily/lily.h"

Lily::Lily(ros::NodeHandle nh, ros::NodeHandle pnh) {
  nh_ = nh;
  pnh_ = pnh;

  bool manual_mode = false;
  pnh_.param<bool>("manual_mode", manual_mode, "");
  if (manual_mode) {
    manual_lily_.reset(new ManualLily(nh_, pnh_));
    manual_lily_->init();
    manual_lily_->run();
  } else {
    auto_lily_.reset(new AutoLily(nh_, pnh_));
    auto_lily_->init();
    auto_lily_->run();
  }

  return;
}
