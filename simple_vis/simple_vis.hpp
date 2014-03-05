#include "../typedefs.h"


boost::shared_ptr <pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

boost::shared_ptr <pcl::visualization::PCLVisualizer>
rgbDualVis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
           Eigen::Matrix4f transform);

boost::shared_ptr <pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

boost::shared_ptr <pcl::visualization::PCLVisualizer>
simpleDualVis(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
              Eigen::Matrix4f transform);
