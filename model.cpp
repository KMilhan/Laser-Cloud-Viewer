#include "model.hpp"
#include "omp.h"

model::model() {
}

model::~model() {
    this->clean_cloud_vector();
}

int model::clean_cloud_vector() {
    rgb_cloud_vector.clear();
    xyz_cloud_vector.clear();
    return 0;
}

int model::clean_xyz_cloud_vector() {
    xyz_cloud_vector.clear();
    return 0;
}

int model::clean_rgb_cloud_vector() {
    rgb_cloud_vector.clear();
    return 0;
}

int model::visualize_cloud() {
    if (rgb_cloud_vector.size() != 0)
        visualize_rgb_cloud_vector();
    else if (xyz_cloud_vector.size() != 0)
        visualize_xyz_cloud_vector();
    else
        return -1;
    return 0;
}

void model::visualize_rgb_cloud_vector() {
    ///////VISUALIZER INITIALIZE
    pcl::visualization::PCLVisualizer *viewer(new pcl::visualization::PCLVisualizer("3D Laser RGB Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    for (int i = 0; i < rgb_cloud_vector.size(); i++) {
        std::stringstream ss;
        ss << i;
        pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGB> rgb_color(rgb_cloud_vector.at(i));
        viewer->addPointCloud<pcl::PointXYZRGB>(rgb_cloud_vector.at(i), rgb_color, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
    }

    viewer->addCoordinateSystem(100.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }
    free(viewer);
}

void model::visualize_xyz_cloud_vector() {
    ///////VISUALIZER INITIALIZE
    pcl::visualization::PCLVisualizer *viewer(new pcl::visualization::PCLVisualizer("3D Laser RGB Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    for (int i = 0; i < xyz_cloud_vector.size(); i++) {
        std::stringstream ss;
        ss << i;
        pcl::visualization::PointCloudColorHandlerRandom <pcl::PointXYZ> random_color(xyz_cloud_vector.at(i));
        viewer->addPointCloud<pcl::PointXYZ>(xyz_cloud_vector.at(i), random_color, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
    }

    viewer->addCoordinateSystem(100.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    free(viewer);
}

int model::save_files(std::string file_name) {
    if (xyz_cloud_vector.size() != 0) {
        this->save_files(file_name, xyz_cloud_vector);
        return 0;
    } else if (rgb_cloud_vector.size() != 0) {
        this->save_files(file_name, rgb_cloud_vector);
        return 0;
    }
    std::cout << "File Saving Error: No clouds in your memory." << std::endl;
    return -1;
}

int model::save_files(std::string file_name, std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector) {
    int index = 0;
    std::string name_seed = file_name;
    for (int i = 0; i < cloud_vector.size(); i++) {
        std::string current_file_name = name_seed + boost::lexical_cast<std::string>(index);
        save_file(current_file_name, cloud_vector.at(i));
        std::cout << "File Saved: " << current_file_name << std::endl;
        index++;
    }
    return index;
}

int model::save_files(std::string file_name, std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector) {
    int index = 0;
    std::string name_seed = file_name;
    for (int i = 0; i < cloud_vector.size(); i++) {
        std::string current_file_name = name_seed + boost::lexical_cast<std::string>(index);
        save_file(current_file_name, cloud_vector.at(i));
        std::cout << "File Saved: " << current_file_name << std::endl;
        index++;
    }
    return index;

}

void model::save_file(std::string file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    try {
        pcl::io::savePCDFileASCII(file_name + ".pcd", *cloud);
    }
    catch (...) {
        std::cout << "File Save Error" << std::endl;
    }

}

void model::save_file(std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    try {
        pcl::io::savePCDFileASCII(file_name + ".pcd", *cloud);
    }
    catch (...) {
        std::cout << "File Save Error" << std::endl;
    }

}

int model::open_files(std::vector <std::string> file_names) {
    this->clean_cloud_vector();
#pragma omp parallel for
    for (int i = 0; i < file_names.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
        if ((cloud = point_cloud_open_file(file_names.at(i))) == NULL) {
            std::cout << "Error in file opening: " << file_names.at(i) << std::endl;
            continue;
        }
#pragma omp critical(dataupdate)
        this->rgb_cloud_vector.push_back(cloud);
    }
    return rgb_cloud_vector.size(); //Return the number of opened clouds

}

int model::registration() {
    this->rgb_cloud = point_get_reg_from_vector(rgb_cloud_vector);
    this->clean_rgb_cloud_vector();
    rgb_cloud_vector.push_back(rgb_cloud);

    if (this->rgb_cloud != NULL) //Return error code
        return 0;
    else
        return -1;
}

int model::registration(float min, float max, float leaf_size, int iteration) {
    this->rgb_cloud = point_get_reg_from_vector(rgb_cloud_vector, min, max, leaf_size, iteration);
    this->clean_rgb_cloud_vector();
    rgb_cloud_vector.push_back(rgb_cloud);

    if (rgb_cloud != NULL) //Return error code
        return 0;
    else
        return -1;
}

int model::noise_cancel() {
    return this->noise_cancel(50, 1.0f);
}

int model::noise_cancel(int meanK = 50, float stddev = 1.0f) {
    int count = 0;
#pragma omp parallel for
    for (int i = 0; i < rgb_cloud_vector.size(); i++) {
#pragma omp critical(dataupdate)
        rgb_cloud_vector.at(i) = point_noise_removal(rgb_cloud_vector.at(i));
#pragma omp critical(dataupdate)
        count++;
    }

#pragma omp parallel for
    for (int i = 0; i < xyz_cloud_vector.size(); i++) {
#pragma omp critical(dataupdate)
        rgb_cloud_vector.at(i) = point_noise_removal(rgb_cloud_vector.at(i));
#pragma omp critical(dataupdate)
        count++;
    }
    if (count > 0)
        return count;
    else
        return -1;

}

int model::segmentation() {
    std::cout << "Default Segmentation Value : (100.f, 256, 200, 200.0, 100, 25000);" << std::endl;
    return segmentation(100.f, 256, 200, 200.0, 100, 25000);
}

int model::segmentation(float voxel_leaf_size = 10.f, int max_iteration = 256, int distance_thresh = 200,
                        double cluster_tolerance = 200.0, int min_cluster_size = 100, int max_cluster_size = 25000) {
    this->clean_xyz_cloud_vector();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>), cloud_f(
            new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud;

    if (this->rgb_cloud != NULL)
        rgb_cloud = this->rgb_cloud;
    else if (this->rgb_cloud_vector.size() != 0)
        rgb_cloud = this->rgb_cloud_vector.back();
    else
        return -1;

    cloud = point_cloud_xyzrgb_to_xyz(rgb_cloud);

    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid <pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points."
              << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation <pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iteration);
    seg.setDistanceThreshold(distance_thresh);

    int i = 0, nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices <pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points."
                  << std::endl;
        std::cout << "point size: " << cloud_filtered->points.size() << std::endl;
        std::cout << "do until: " << 0.3 * nr_points << std::endl;
        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector <pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction <pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance); // 2cm
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    std::cout << "ec set." << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        std::cout << "file saving" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud <pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
                  << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        j++;
        ////ADD cloud_cluster to current rgb.viewer
        xyz_cloud_vector.push_back(cloud_cluster);
    }
    this->clean_rgb_cloud_vector();

    return 0;

}

