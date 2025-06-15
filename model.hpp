#include "file/opener.hpp"
#include "processing/cloud_processing.hpp"
#include "simple_vis/simple_vis.hpp"
#include "typedefs.h"

class model {
  public:
    model();

    int open_files(std::vector<std::string> file_names);

    int save_files(std::string file_name);

    int save_files(std::string file_name,
                   std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector);

    int save_files(std::string file_name,
                   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector);

    int noise_cancel();

    int noise_cancel(int meanK, float stddev);

    int registration();

    int registration(float min, float max, float leaf_size, int iteration);

    int segmentation();

    int segmentation(float voxel_leaf_size, int max_iteration, int distnace_thresh,
                     double cluster_tolerance, int min_cluster_size, int max_cluster_size);

    int clean_cloud_vector();

    int visualize_cloud();

    ~model();

    /* data */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rgb_cloud_vector;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> xyz_cloud_vector;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cloud;

  private:
    void save_file(std::string file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    void save_file(std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void visualize_rgb_cloud_vector();

    void visualize_xyz_cloud_vector();

    int clean_xyz_cloud_vector();

    int clean_rgb_cloud_vector();
};