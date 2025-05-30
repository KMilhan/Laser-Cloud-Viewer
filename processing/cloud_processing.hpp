#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <Eigen/Dense>
#include <vector>

extern float cut_min;
extern float cut_max;
extern float reg_leaf_size;
extern int noise_meanK;
extern float noise_stddev;
extern int reg_iteration;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_xyzrgb_to_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_xyz_to_xyzrgb(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr point_cut_off_floor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);


pcl::PointCloud<pcl::PointXYZ>::Ptr point_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr point_noise_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_noise_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr point_prepare_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_get_reg_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud,
                                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_get_reg_from_vector(std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_vector);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_get_reg_from_vector(std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_vector, float min, float max,
                          float leaf_size, int iteration);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_fine_voxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud);
