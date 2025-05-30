#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/file_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <omp.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_from_txt(std::string file_path);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_open_pcd_file(std::string file_path);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_open_file(std::string file_path);

std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> open_test_files();
