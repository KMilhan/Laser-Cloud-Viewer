#ifndef KITTI_UTILS_HPP
#define KITTI_UTILS_HPP

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

pcl::PointCloud<pcl::PointXYZI>::Ptr loadKittiBin(const std::string &file_path);

std::vector<Eigen::Vector2f> projectToImagePlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                 const Eigen::Matrix<float, 3, 4> &projection);

#endif // KITTI_UTILS_HPP
