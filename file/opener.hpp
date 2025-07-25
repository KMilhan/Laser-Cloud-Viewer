#pragma once
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>

#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <omp.h>

#include "processing/cloud_processing.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
open_point_cloud_xyz_to_xyzrgb(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_from_txt(std::string file_path);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_open_pcd_file(std::string file_path);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_open_file(std::string file_path);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> open_test_files();
