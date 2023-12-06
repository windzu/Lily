#include "lily/ground_segmentation.h"

bool point_cmp(pcl::PointXYZ a, pcl::PointXYZ b) {
  return a.z < b.z;
}

GroundPlaneFit::GroundPlaneFit(double sensor_height, int num_iter, int num_lpr,
                               double th_seeds, double th_dist) {
  sensor_height_ = sensor_height;
  num_iter_ = num_iter;
  num_lpr_ = num_lpr;
  th_seeds_ = th_seeds;
  th_dist_ = th_dist;

  g_seeds_pc_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  g_ground_pc_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  g_not_ground_pc_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
          pcl::PointCloud<pcl::PointXYZ>::Ptr>
GroundPlaneFit::process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
  // 1.Msg to pointcloud
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *input;
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn_org = *input;

  // 2.Sort on Z-axis value.
  std::sort(laserCloudIn.points.begin(), laserCloudIn.points.end(), point_cmp);

  // 3.Error point removal
  // As there are some error mirror reflection under the ground,
  // here regardless point under 2* sensor_height
  // Sort point according to height, here uses z-axis in default
  pcl::PointCloud<pcl::PointXYZ>::iterator it = laserCloudIn.points.begin();
  for (int i = 0; i < laserCloudIn.points.size(); i++) {
    if (laserCloudIn.points[i].z < -1.5 * sensor_height_) {
      it++;
    } else {
      break;
    }
  }
  laserCloudIn.points.erase(laserCloudIn.points.begin(), it);

  // 4. Extract init ground seeds.
  extract_initial_seeds_(laserCloudIn);
  g_ground_pc_ = g_seeds_pc_;

  // 5. Ground plane fitter mainloop
  for (int i = 0; i < num_iter_; i++) {
    estimate_plane_();
    g_ground_pc_->clear();
    g_not_ground_pc_->clear();

    // pointcloud to matrix
    Eigen::MatrixXf points(laserCloudIn_org.points.size(), 3);
    int j = 0;
    for (auto p : laserCloudIn_org.points) {
      points.row(j++) << p.x, p.y, p.z;
    }
    // ground plane model
    Eigen::VectorXf result = points * normal_;
    // threshold filter
    for (int r = 0; r < result.rows(); r++) {
      if (result[r] < th_dist_d_) {
        g_ground_pc_->points.push_back(laserCloudIn_org[r]);
      } else {
        g_not_ground_pc_->points.push_back(laserCloudIn_org[r]);
      }
    }
  }
  return std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr>(g_not_ground_pc_,
                                                        g_ground_pc_);
}

void GroundPlaneFit::estimate_plane_(void) {
  // Create covarian matrix in single pass.
  // TODO: compare the efficiency.
  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;
  pcl::computeMeanAndCovarianceMatrix(*g_ground_pc_, cov, pc_mean);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(
      cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));
  // mean ground seeds value
  seeds_mean = pc_mean.head<3>();
  // Model parameter for ground plane fitting
  // The ground plane model is: ax+by+cz+d=0
  // Here normal:=[a,b,c], d=d
  // th_dist_d_ = threshold_dist - d
  // according to normal.T*[x,y,z] = -d
  float d = -(normal_.transpose() * seeds_mean)(0, 0);
  // set distance threhold to `th_dist - d`
  th_dist_d_ = th_dist_ - d;

  // return the equation parameters
}

void GroundPlaneFit::extract_initial_seeds_(
    const pcl::PointCloud<pcl::PointXYZ>& p_sorted) {
  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;
  // Calculate the mean height value.
  for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
    sum += p_sorted.points[i].z;
    cnt++;
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0
  g_seeds_pc_->clear();
  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + th_seeds_) {
      g_seeds_pc_->points.push_back(p_sorted.points[i]);
    }
  }
  // return seeds points
}
