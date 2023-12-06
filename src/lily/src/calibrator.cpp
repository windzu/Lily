/*
 * @Author: windzu windzu1@gmail.com
 * @Date: 2023-06-16 16:13:34
 * @LastEditors: wind windzu1@gmail.com
 * @LastEditTime: 2023-12-06 18:26:17
 * @Description:
 * Copyright (c) 2023 by windzu, All Rights Reserved.
 */
#include "lily/calibrator.h"

#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <time.h>

#include <chrono>  // NOLINT
#include <iostream>
#include <thread>  // NOLINT

// 点云赋色数值
unsigned char color_map[10][3] = {{255, 255, 255},  // "white"
                                  {255, 0, 0},      // "red"
                                  {0, 255, 0},      // "green"
                                  {255, 255, 0},    // "blue"
                                  {0, 0, 255},      // "yellow"
                                  {255, 0, 255},    // "pink"
                                  {50, 255, 255},   // "light-blue"
                                  {135, 60, 0},     //
                                  {150, 240, 80},   //
                                  {80, 30, 180}};   //

std::unordered_map<std::string, Eigen::Matrix4d> Calibrator::process(
    const std::unordered_map<std::string,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloud_map,
    const std::string& main_topic,
    const std::unordered_map<std::string, std::vector<pcl::PointXYZ>>&
        points_map,
    const std::unordered_map<std::string, Eigen::Matrix4d>& tf_matrix_map) {
  main_topic_ = main_topic;
  cloud_map_ = cloud_map;
  points_map_ = points_map;
  tf_matrix_map_ = tf_matrix_map;

  // 1. 对每个雷达点云进行初始化变换
  for (auto iter = cloud_map_.begin(); iter != cloud_map_.end(); iter++) {
    auto& topic = iter->first;
    auto& cloud = iter->second;
    auto tf_matrix_iter = tf_matrix_map_.find(topic);
    if (tf_matrix_iter == tf_matrix_map_.end()) {
      std::cout << "tf_matrix_map_ find failed" << std::endl;
      return tf_matrix_map_;
    }
    Eigen::Matrix4d tf_matrix = tf_matrix_iter->second;
    pcl::transformPointCloud(*cloud, *cloud, tf_matrix);
  }

  // 2. 对每个雷达点云进行地面分割并矫正
  if (!ground_calibration()) {
    std::cout << "ground_calibration failed" << std::endl;
  }

  // TODO(windzu) : 点云匹配
  // calibrator.LoadCalibrationData(lidar_points, raw_extrinsics);  //
  // 导入初值标定参数 auto time_begin = std::chrono::steady_clock::now();
  // calibrator.Calibrate();  // 标定主函数
  // auto time_end = std::chrono::steady_clock::now();
  // 获得标定结果（刚性变换矩阵）
  // std::map<int32_t, Eigen::Matrix4d> refined_extrinsics =
  // calibrator.GetFinalTransformation();

  // stitching and save for visualization

  // debug

  // stiching();

  return tf_matrix_map_;
}

bool Calibrator::ground_calibration() {
  // 对每个雷达点云进行地面标定
  // 1. 提取地面平面参数
  // 2. 利用地面平面参数对点云进行变换，使点云z轴与地面法向量平行
  // 3. 再次提取地面平面参数，获取地面平面的高度值，将点云与地面平面对齐
  // 4. 更新标定参数
  Eigen::Vector3f z_normal(0, 0, 1);
  for (auto iter = cloud_map_.begin(); iter != cloud_map_.end(); iter++) {
    auto& topic = iter->first;
    auto& cloud = iter->second;
    auto& tf_matrix = tf_matrix_map_[topic];

    // check if points_map_ has this topic and not empty
    for (auto points_map_iter = points_map_.begin();
         points_map_iter != points_map_.end(); points_map_iter++) {
      if (points_map_iter->first == topic) {
        if (points_map_iter->second.size() != 0) {
          // use this points to calculate ground plane
          cloud->clear();
          for (auto points_iter = points_map_iter->second.begin();
               points_iter != points_map_iter->second.end(); points_iter++) {
            pcl::PointXYZI point;
            point.x = points_iter->x;
            point.y = points_iter->y;
            point.z = points_iter->z;
            cloud->push_back(point);
          }
        }
      }
    }

    double sensor_height = std::abs(tf_matrix(2, 3));

    // debug
    sensor_height = 0;

    // 第一次计算地面平面参数,为了获取地面平面法向量
    pcl::ModelCoefficients::Ptr first_coefficients;
    if (cloud->points.size() < 100) {
      first_coefficients = compute_plane_normal(cloud);
    } else {
      first_coefficients = ground_plane_extraction(cloud, sensor_height);
    }

    Eigen::Vector3f normal;
    normal[0] = first_coefficients->values[0];
    normal[1] = first_coefficients->values[1];
    normal[2] = first_coefficients->values[2];

    // 使用第一次获取的地面平面参数对点云做变换，使点云z轴与地面法向量平行
    // 矫正法向量到z轴(0,0,1)
    Eigen::Matrix4d first_ret = create_rotate_matrix(normal, z_normal);
    pcl::transformPointCloud(*cloud, *cloud, first_ret);

    // 第二次计算地面平面参数,为了获取地面平面的高度值
    Eigen::Matrix4d second_ret = Eigen::Matrix4d::Identity();
    pcl::ModelCoefficients::Ptr sceond_coefficients;
    if (cloud->points.size() < 100) {
      sceond_coefficients = compute_plane_normal(cloud);
    } else {
      sceond_coefficients = ground_plane_extraction(cloud, sensor_height);
    }
    // 获取地面与0平面的距离
    second_ret(2, 3) = sceond_coefficients->values[3];
    pcl::transformPointCloud(*cloud, *cloud, second_ret);

    tf_matrix = second_ret * first_ret * tf_matrix;
  }

  return true;
}

bool Calibrator::icpn() {
  // 对所有点云进行icp匹配，其中main_topic_为主雷达 是其他雷达的参考系

  Eigen::Vector3f z_normal(0, 0, 1);
  for (auto iter = cloud_map_.begin(); iter != cloud_map_.end(); iter++) {
    auto& topic = iter->first;
    auto& cloud = iter->second;
    auto& tf_matrix = tf_matrix_map_[topic];

    //     pcl::transformPointCloud(*cloud, *cloud, second_ret);
    //
    //     tf_matrix = second_ret * first_ret * tf_matrix;
  }

  return true;
}

// bool Calibrator::icpn() {
//   Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
//   Eigen::Matrix4d curr_transform = Eigen::Matrix4d::Identity();
//
//   int32_t master_id = 0;
//   auto master_iter = pcs_.find(master_id);
//   pcl::PointCloud<pcl::PointXYZI> master_pc = master_iter->second;
//   pcl::PointCloud<pcl::PointXYZI>::Ptr master_pc_ptr =
//   master_pc.makeShared(); PlaneParam master_gplane;
//   pcl::PointCloud<pcl::PointXYZI>::Ptr master_gcloud(new
//   pcl::PointCloud<pcl::PointXYZI>); pcl::PointCloud<pcl::PointXYZI>::Ptr
//   master_ngcloud(new pcl::PointCloud<pcl::PointXYZI>); bool ret =
//   ground_plane_extraction(master_pc_ptr, master_gcloud, master_ngcloud,
//                                      master_gplane);  //
//                                      使用ransac去除地面点

//   registrator_->SetTargetCloud(master_gcloud, master_ngcloud,
//   master_pc_ptr); registrator_->SetTargetCloud(master_pc_ptr, master_pc_ptr,
//   master_pc_ptr);  // 注册点云 Eigen::Vector3d t_mp(0, 0,
//   -master_gplane.intercept / master_gplane.normal(2));
//
//   for (auto iter = pcs_.begin(); iter != pcs_.end(); iter++) {
//     int32_t slave_id = iter->first;
//     if (slave_id == master_id) continue;
//     pcl::PointCloud<pcl::PointXYZI> slave_pc = iter->second;
//     pcl::PointCloud<pcl::PointXYZI> slave_original_pc = slave_pc;
//     if (init_extrinsics_.find(slave_id) == init_extrinsics_.end()) {
//       LOGE("cannot find the init extrinsic, id: %d\n", slave_id);
//       return;
//     }
//     Eigen::Matrix4d init_ext = init_extrinsics_[slave_id];
//     pcl::PointCloud<pcl::PointXYZI>::Ptr slave_pc_ptr =
//     slave_pc.makeShared(); pcl::PointCloud<pcl::PointXYZI>::Ptr
//     cloud_after_Condition(new pcl::PointCloud<pcl::PointXYZI>); PlaneParam
//     slave_gplane; pcl::PointCloud<pcl::PointXYZI>::Ptr slave_gcloud(new
//     pcl::PointCloud<pcl::PointXYZI>); pcl::PointCloud<pcl::PointXYZI>::Ptr
//     slave_ngcloud(new pcl::PointCloud<pcl::PointXYZI>);
//
//     // earse the points close to LiDAR
//     if (slave_id) {
//       pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_condition(
//           new pcl::ConditionAnd<pcl::PointXYZI>());
//       range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
//           new pcl::FieldComparison<pcl::PointXYZI>("x",
//           pcl::ComparisonOps::GT, -1)));
//       range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
//           new pcl::FieldComparison<pcl::PointXYZI>("x",
//           pcl::ComparisonOps::LT, 1)));  //
//       range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
//           new pcl::FieldComparison<pcl::PointXYZI>("y",
//           pcl::ComparisonOps::GT, -1.0)));
//       range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
//           new pcl::FieldComparison<pcl::PointXYZI>("y",
//           pcl::ComparisonOps::LT, 1)));
//       range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
//           new pcl::FieldComparison<pcl::PointXYZI>("z",
//           pcl::ComparisonOps::GT, -1)));
//       range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
//           new pcl::FieldComparison<pcl::PointXYZI>("z",
//           pcl::ComparisonOps::LT, 1)));
//
//       pcl::ConditionalRemoval<pcl::PointXYZI> condition;
//       condition.setCondition(range_condition);
//       condition.setInputCloud(slave_pc_ptr);
//       condition.setKeepOrganized(false);
//       condition.filter(*cloud_after_Condition);
//
//       pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
//       pcl::PointXYZI searchPoint;
//       int K = 1;
//       std::vector<int> pointIdxNKNSearch(K);
//       std::vector<float> pointNKNSquaredDistance(K);
//       std::vector<pcl::PointXYZI> DeleteData;
//       int num = 0;
//       for (auto iter = cloud_after_Condition->begin(); iter !=
//       cloud_after_Condition->end();
//            iter++) {
//         searchPoint.x = iter->x;
//         searchPoint.y = iter->y;
//         searchPoint.z = iter->z;
//         kdtree.setInputCloud(slave_pc_ptr);
//         num = kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
//         pointNKNSquaredDistance); if (num > 0) {
//           if (sqrt(pointNKNSquaredDistance[0]) < eps) {
//             auto iterB = slave_pc_ptr->begin() + pointIdxNKNSearch[0];
//             slave_pc_ptr->erase(iterB);
//             DeleteData.push_back(searchPoint);
//             if (slave_pc_ptr->size() == 0) {
//               break;
//             }
//             searchPoint.x = 0;
//             searchPoint.y = 0;
//             searchPoint.z = 0;
//             num = 0;
//             pointIdxNKNSearch.clear();
//             pointNKNSquaredDistance.clear();
//           }
//         }
//       }
//     }
//
//     ret = ground_plane_extraction(slave_pc_ptr, slave_gcloud, slave_ngcloud,
//     slave_gplane); if (!ret) {
//       LOGE("ground plane extraction failed.\n");
//       continue;
//     }
//
//     pcl::PointCloud<pcl::PointXYZI>::Ptr slave_original_pc_ptr =
//     slave_original_pc.makeShared(); PlaneParam slave_original_gplane;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr slave_original_ngcloud(
//         new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr slave_original_gcloud(new
//     pcl::PointCloud<pcl::PointXYZI>);
//
//     ret = ground_plane_extraction(slave_original_pc_ptr,
//     slave_original_gcloud,
//                                   slave_original_ngcloud,
//                                   slave_original_gplane);
//     registrator_->SetSourceCloud(slave_original_gcloud,
//     slave_original_ngcloud,
//                                  slave_original_pc_ptr);
//
//     // ground normal direction
//     Eigen::Vector3f ground_point(0, 0, (slave_gplane.intercept) /
//     (-slave_gplane.normal(2))); Eigen::Vector3f point2plane_vector; int
//     Ontheground = 0; int Undertheground = 0; for (auto iter =
//     slave_ngcloud->begin(); iter < slave_ngcloud->end() - 100; iter += 100)
//     {
//       Eigen::Vector3f samplePoint(iter->x, iter->y, iter->z);
//       point2plane_vector = samplePoint - ground_point;
//       if ((point2plane_vector(0) * slave_gplane.normal(0) +
//            point2plane_vector(1) * slave_gplane.normal(1) +
//            point2plane_vector(2) * slave_gplane.normal(2)) >= 0) {
//         Ontheground++;
//       } else {
//         Undertheground++;
//       }
//     }
//     // ground plane align
//     Eigen::Vector3d rot_axis2 =
//     slave_gplane.normal.cross(master_gplane.normal); rot_axis2.normalize();
//     double alpha2 =
//     std::acos(slave_gplane.normal.dot(master_gplane.normal));
//     Eigen::Matrix3d R_ms;
//     R_ms = Eigen::AngleAxisd(alpha2, rot_axis2);
//     Eigen::Vector3d slave_intcpt_local(0, 0, -slave_gplane.intercept /
//     slave_gplane.normal(2)); Eigen::Vector3d slave_intcpt_master = R_ms *
//     slave_intcpt_local; Eigen::Vector3d t_ms(0, 0, t_mp(2) -
//     slave_intcpt_master(2));
//
//     Eigen::Matrix4d T_ms = Eigen::Matrix4d::Identity();
//     T_ms.block<3, 1>(0, 3) = t_ms;
//     T_ms.block<3, 3>(0, 0) = R_ms;
//
//     double z_error = std::fabs(t_ms(2) - init_ext(2, 3));
//     if (z_error > 0.5) {
//       slave_gplane.normal = -slave_gplane.normal;
//       slave_gplane.intercept = -slave_gplane.intercept;
//       rot_axis2 = slave_gplane.normal.cross(master_gplane.normal);
//       rot_axis2.normalize();
//       alpha2 = std::acos(slave_gplane.normal.dot(master_gplane.normal));
//       R_ms = Eigen::AngleAxisd(alpha2, rot_axis2);
//       slave_intcpt_local = Eigen::Vector3d(0, 0, -slave_gplane.intercept /
//       slave_gplane.normal(2)); slave_intcpt_master = R_ms *
//       slave_intcpt_local; t_ms = Eigen::Vector3d(0, 0, t_mp(2) -
//       slave_intcpt_master(2)); T_ms.block<3, 1>(0, 3) = t_ms; T_ms.block<3,
//       3>(0, 0) = R_ms;
//     }
//     curr_transform = init_ext * T_ms;
//
//     std::cout << "transform: " << curr_transform << std::endl;
//     registrator_->RegistrationByICP(curr_transform, transform);
//     Eigen::Matrix4d final_opt_result;
//     registrator_->RegistrationByICP2(transform, final_opt_result);
//     // final_opt_result(0, 2)=0;final_opt_result(1, 2)=0;final_opt_result(2,
//     // 0)=0;final_opt_result(2, 1)=0;final_opt_result(2, 2)=1;
//     final_opt_result(2, 3) = 0;
//     Eigen::Matrix<double, 3, 3> rot = final_opt_result.block<3, 3>(0, 0);
//     Eigen::Vector3f euler = rotation_matrix_to_euler_angles(rot);
//     // 计算旋转矩阵的Z分量
//     Eigen::Matrix3d R;
//     R << cos(euler[2]), -sin(euler[2]), 0.0, sin(euler[2]), cos(euler[2]),
//     0.0, 0.0, 0.0, 1.0; final_opt_result.block<3, 3>(0, 0) = R;  //
//     去除刚性变换中与Z轴方向有关的旋转和平移因素
//     refined_extrinsics_.insert(std::make_pair(slave_id, final_opt_result));
//   }
//
//   return true;
// }

void Calibrator::stiching() {
  // stitching and save for visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto iter = cloud_map_.begin(); iter != cloud_map_.end(); iter++) {
    auto topic = iter->first;
    auto cloud = iter->second;
    int color_id = 0;
    // auto extrinsic_iter = extrinsic_map_.find(topic);
    // if (extrinsic_iter == extrinsic_map_.end()) {
    //   YERROR << "extrinsic_map_ find failed";
    //   return false;
    // }
    // Eigen::Matrix4d extrinsic = extrinsic_iter->second;
    // pcl::transformPointCloud(*cloud, *cloud, extrinsic);

    // 点云赋色，方便更直观的看到标定效果
    for (auto src : cloud->points) {
      pcl::PointXYZRGB point;
      point.x = src.x;
      point.y = src.y;
      point.z = src.z;
      point.r = color_map[color_id % 7][0];
      point.g = color_map[color_id % 7][1];
      point.b = color_map[color_id % 7][2];
      all_cloud->push_back(point);
    }
    color_id++;
  }
  // save pcd
  pcl::io::savePCDFileASCII("all_cloud.pcd", *all_cloud);
}

// 旋转矩阵转欧拉角
Eigen::Vector3d Calibrator::rotation_matrix_to_euler_angles(
    const Eigen::Matrix3d& R) {
  // assert(isRotationMatrix(R));
  double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

  bool singular = sy < 1e-6;  // If

  double x, y, z;
  if (!singular) {
    x = atan2(R(2, 1), R(2, 2));
    y = atan2(-R(2, 0), sy);
    z = atan2(R(1, 0), R(0, 0));
  } else {
    x = atan2(-R(1, 2), R(1, 1));
    y = atan2(-R(2, 0), sy);
    z = 0;
  }
  return Eigen::Vector3d(x, y, z);
}

// 获取两个法向量之间的转换刚性矩阵
Eigen::Matrix4d Calibrator::create_rotate_matrix(Eigen::Vector3f before,
                                                 Eigen::Vector3f after) {
  before.normalize();
  after.normalize();

  float angle = acos(before.dot(after));
  Eigen::Vector3f p_rotate = before.cross(after);
  p_rotate.normalize();

  Eigen::Matrix4d rotationMatrix = Eigen::Matrix4d::Identity();
  rotationMatrix(0, 0) =
      cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
  rotationMatrix(0, 1) =
      p_rotate[0] * p_rotate[1] *
      (1 - cos(angle) -
       p_rotate[2] *
           sin(angle));  // 这里跟公式比多了一个括号，但是看实验结果它是对的
  rotationMatrix(0, 2) =
      p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

  rotationMatrix(1, 0) =
      p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
  rotationMatrix(1, 1) =
      cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
  rotationMatrix(1, 2) =
      -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

  rotationMatrix(2, 0) =
      -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
  rotationMatrix(2, 1) =
      p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
  rotationMatrix(2, 2) =
      cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

  return rotationMatrix;
}

// 使用gpf算法提取地面并获取法向量
pcl::ModelCoefficients::Ptr Calibrator::ground_plane_extraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_cloud,
    double sensor_height) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  tmp->points.resize(in_cloud->size());
  for (size_t i = 0; i < tmp->points.size(); i++) {
    double radius = sqrt(in_cloud->points[i].x * in_cloud->points[i].x +
                         in_cloud->points[i].y * in_cloud->points[i].y);
    if (radius <= 7.0) {
      tmp->points[i].x = in_cloud->points[i].x;
      tmp->points[i].y = in_cloud->points[i].y;
      tmp->points[i].z = in_cloud->points[i].z;
    }
  }

  // GroundPlaneFit* gpf = new GroundPlaneFit(sensor_height, num_iter_,
  // num_lpr_, th_seeds_, th_dist_);
  GroundPlaneFit gpf(sensor_height, num_iter_, num_lpr_, th_seeds_, th_dist_);

  gpf.process(tmp);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  coefficients->values.push_back(gpf.normal_(0, 0));
  coefficients->values.push_back(gpf.normal_(1, 0));
  coefficients->values.push_back(gpf.normal_(2, 0));
  coefficients->values.push_back(-gpf.seeds_mean(2));

  return coefficients;
}

pcl::ModelCoefficients::Ptr Calibrator::compute_plane_normal(
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

  return coefficients;
}

// std::vector<Eigen::Matrix4d> Calibrator::process(
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& top_cloud,
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& front_cloud,
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& left_cloud,
//     pcl::PointCloud<pcl::PointXYZI>::Ptr& right_cloud) {
//   std::vector<Eigen::Matrix4d> calib_list_tmp;  // 用于获取标定结果
//   calib_list_tmp.push_back(top_ret);
//   int count = 0;
//   for (auto iter = refined_extrinsics.begin(); iter !=
//   refined_extrinsics.end();
//        iter++) {  // 依次获取标定结果
//     int32_t slave_id = iter->first;
//     Eigen::Matrix4d transform = iter->second;  // 获取点云的刚性变换矩阵
//
//     auto slave_iter = lidar_points.find(slave_id);
//     pcl::PointCloud<pcl::PointXYZI> slave_pc = slave_iter->second;
//
//     pcl::PointCloud<pcl::PointXYZI> trans_cloud;
//     Eigen::Matrix<double, 3, 3> rot = transform.block<3, 3>(0, 0);
//     Eigen::Vector3f euler = rotation_matrix_to_euler_angles(rot);
//     // 计算旋转矩阵的Z分量
//     Eigen::Matrix3d R;
//     R << cos(euler[2]), -sin(euler[2]), 0.0, sin(euler[2]), cos(euler[2]),
//     0.0, 0.0, 0.0, 1.0; transform.block<3, 3>(0, 0) = R;  //
//     去除刚性变换中与Z轴方向有关的旋转和平移因素 transform(2, 3) = 0;
//     pcl::transformPointCloud(slave_pc, trans_cloud, transform);  // 点云转换
//     calib_list_tmp.push_back(transform);
//     // trans_cloud=slave_pc;
//     for (auto src : trans_cloud.points) {  // 补盲雷达点云赋色
//       pcl::PointXYZRGB point;
//       point.x = src.x;
//       point.y = src.y;
//       point.z = src.z;
//       point.r = color_map[slave_id % 7][0];
//       point.g = color_map[slave_id % 7][1];
//       point.b = color_map[slave_id % 7][2];
//       all_cloud->push_back(point);
//     }
//     count++;
//   }
//   all_cloud->height = 1;
//   all_cloud->width = all_cloud->points.size();
//
//   // 将ICPN配准的结果更新到标定变换矩阵中
//   calib_list[1] = calib_list_tmp[1] * calib_list[1];
//   calib_list[2] = calib_list_tmp[2] * calib_list[2];
//   calib_list[3] = calib_list_tmp[3] * calib_list[3];
//
//   // top雷达与车体主轴的相对平移不变，只需更新旋转矩阵
//   double top_x = sensor_param_config_.sensor_units(0).tf_config().tf_x();
//   double top_y = sensor_param_config_.sensor_units(0).tf_config().tf_y();
//   double top_z = sensor_param_config_.sensor_units(0).tf_config().tf_z();
//   double trans_x = 0, trans_y = 0, trans_z = 0;
//
//   // 修改sensor_param.pb.txt中的数值
//   for (int i = 0; i < calib_list.size(); i++) {
//     Eigen::Matrix<double, 3, 3> rot = calib_list[i].block<3, 3>(0, 0);
//     Eigen::Vector3f euler = rotation_matrix_to_euler_angles(rot);
//     if (i == 0) {
//       trans_x = top_x - calib_list[i](0, 3);
//       trans_y = top_y - calib_list[i](1, 3);
//       trans_z = top_z - calib_list[i](2, 3);
//
//       // 仅保存小数点后三位
//       trans_x = std::round(trans_x * 1000.0) / 1000.0;
//       trans_y = std::round(trans_y * 1000.0) / 1000.0;
//       trans_z = std::round(trans_z * 1000.0) / 1000.0;
//
//       sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_x(top_x);
//       sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_y(top_y);
//       sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_z(top_z);
//     } else {
//       double tf_x = calib_list[i](0, 3) + trans_x;
//       double tf_y = calib_list[i](1, 3) + trans_y;
//       double tf_z = calib_list[i](2, 3) + trans_z;
//
//       // 仅保存小数点后三位
//       tf_x = std::round(tf_x * 1000.0) / 1000.0;
//       tf_y = std::round(tf_y * 1000.0) / 1000.0;
//       tf_z = std::round(tf_z * 1000.0) / 1000.0;
//
//       sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_x(tf_x);
//       sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_y(tf_y);
//       sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_z(tf_z);
//     }
//
//     double tf_roll = euler[0];
//     double tf_pitch = euler[1];
//     double tf_yaw = euler[2];
//
//     // 仅保存小数点后三位
//     tf_roll = std::round(tf_roll * 1000.0) / 1000.0;
//     tf_pitch = std::round(tf_pitch * 1000.0) / 1000.0;
//     tf_yaw = std::round(tf_yaw * 1000.0) / 1000.0;
//
//     sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_roll(tf_roll);
//     sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_pitch(tf_pitch);
//     sensor_param_config_.mutable_sensor_units(i)->mutable_tf_config()->set_tf_yaw(tf_yaw);
//   }
//
//   // 保存sensor_param.pb.txt中的数值
//   yczx::autodriver::proto_util::SetProtoToASCIIFile(sensor_param_config_,
//   kSensorParamConfigFile); return calib_list;
// }
