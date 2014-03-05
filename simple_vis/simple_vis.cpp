#include "simple_vis.hpp"

boost::shared_ptr <pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer("3D Laser RGB Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "basic cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "basic cloud");
    viewer->addCoordinateSystem(100.0);
    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr <pcl::visualization::PCLVisualizer>
rgbDualVis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
           Eigen::Matrix4f transform) {
    Eigen::Matrix4f targetToSource = transform.inverse();
    std::cout << "Entered dual viewer... transforming..." << std::endl;
    pcl::transformPointCloud(*target_cloud, *target_cloud, targetToSource);

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer("3D Laser RGB Viewer"));

    std::cout << "Initializing Viewer" << std::endl;
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGB> rgbs(source_cloud);
    pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGB> rgbt(target_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(source_cloud, rgbs, "source cloud");
    viewer->addPointCloud<pcl::PointXYZRGB>(target_cloud, rgbt, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    viewer->addCoordinateSystem(100.0);
    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr <pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer("3D Laser Simple Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "basic cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "basic cloud");
    viewer->addCoordinateSystem(100.0);
    viewer->initCameraParameters();
    return (viewer);
}

boost::shared_ptr <pcl::visualization::PCLVisualizer>
simpleDualVis(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
              Eigen::Matrix4f transform) {

    Eigen::Matrix4f targetToSource = transform.inverse();
    std::cout << "Entered dual viewer... transforming..." << std::endl;
    pcl::transformPointCloud(*target_cloud, *target_cloud, targetToSource);

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr <pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer("3D Laser Simple Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(source_cloud, "source cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    viewer->addCoordinateSystem(100.0);
    viewer->initCameraParameters();
    return (viewer);
}
