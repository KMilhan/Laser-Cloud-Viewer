#include "kitti_utils.hpp"
#include <fstream>
#include <iostream>

pcl::PointCloud<pcl::PointXYZI>::Ptr loadKittiBin(const std::string &file_path) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::ifstream input(file_path.c_str(), std::ios::binary);
    if (!input.is_open()) {
        std::cerr << "Could not open KITTI file: " << file_path << std::endl;
        return cloud;
    }

    pcl::PointXYZI point;
    while (input.read(reinterpret_cast<char *>(&point), sizeof(pcl::PointXYZI))) {
        cloud->points.push_back(point);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    input.close();
    return cloud;
}

std::vector<Eigen::Vector2f> projectToImagePlane(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        const Eigen::Matrix<float, 3, 4> &projection) {
    std::vector<Eigen::Vector2f> pixels;
    for (const auto &pt : cloud->points) {
        Eigen::Vector4f vec(pt.x, pt.y, pt.z, 1.0f);
        Eigen::Vector3f proj = projection * vec;
        if (proj(2) <= 0.f)
            continue;
        pixels.emplace_back(proj(0) / proj(2), proj(1) / proj(2));
    }
    return pixels;
}
