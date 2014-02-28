#include "../typedefs.h"

static float cut_min = 100.0f;
static float cut_max = 8000.0f;
static float reg_leaf_size = 100.0f;
static int noise_meanK = 50;
static float noise_stddev = 1.0f ;
static int reg_iteration = 30;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_xyzrgb_to_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_xyz_to_xyzrgb(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cut_off_floor (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);


pcl::PointCloud<pcl::PointXYZ>::Ptr point_downsample (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr point_noise_removal (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_noise_removal (pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr point_prepare_icp (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_get_reg_cloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_get_reg_from_vector (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_vector);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_get_reg_from_vector (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_vector, float min, float max, float leaf_size, int iteration);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_fine_voxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud);
