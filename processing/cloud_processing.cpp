#include "cloud_processing.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_get_reg_from_vector(std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_vector, float min, float max,
                          float leaf_size, int iteration) {
    cut_min = min;
    cut_max = max;
    reg_leaf_size = leaf_size;
    reg_iteration = iteration;
    return point_get_reg_from_vector(source_vector);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
point_cloud_xyzrgb_to_xyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    //assign return cloud
#pragma omp parallel for
    for (int i = 0; i < rgb_cloud->points.size(); i++) {
        pcl::PointXYZ point;
        //temporal point
        point.x = rgb_cloud->points[i].x;
        point.y = rgb_cloud->points[i].y;
        point.z = rgb_cloud->points[i].z;
#pragma omp critical(dataupdate)
        {
            xyz_cloud->points.push_back(point);
        }
    }

    xyz_cloud->width = (int) rgb_cloud->points.size();
    xyz_cloud->height = 1;
    return xyz_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_cloud_xyz_to_xyzrgb(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    //assign return cloud
#pragma omp parallel for
    for (int i = 0; i < xyz_cloud->points.size(); i++) {
        pcl::PointXYZRGB point;
        //temporal point
        point.x = rgb_cloud->points[i].x;
        point.y = rgb_cloud->points[i].y;
        point.z = rgb_cloud->points[i].z;
        point.r = 255;
        point.g = 255;
        point.b = 255;
#pragma omp critical(dataupdate)
        {
            rgb_cloud->points.push_back(point);
        }
    }

    rgb_cloud->width = (int) xyz_cloud->points.size();
    rgb_cloud->height = 1;
    return rgb_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
point_cut_off_floor(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud(new pcl::PointCloud <pcl::PointXYZ>);
    //assign return cloud
    size_t rejected = 0;
#pragma omp parallel for
    for (int i = 0; i < input_cloud->points.size(); i++) {
        if (input_cloud->points[i].z < cut_min || input_cloud->points[i].z > cut_max) {
#pragma omp atomic
            rejected++;
            continue;
        }

        pcl::PointXYZ point;
        //temporal point
        point.x = input_cloud->points[i].x;
        point.y = input_cloud->points[i].y;
        point.z = input_cloud->points[i].z;
#pragma omp critical(dataupdate)
        {
            return_cloud->points.push_back(point);
        }
    }

    return_cloud->width = (int) input_cloud->points.size() - (int) rejected;
    return_cloud->height = 1;
    std::cout << return_cloud->points.size() << " Points from input cloud" << std::endl;
    return return_cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
point_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    //Downsampler
    pcl::VoxelGrid <pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setLeafSize(reg_leaf_size, reg_leaf_size, reg_leaf_size);
    sor.filter(*input_cloud);
    return input_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
point_noise_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    int original = input_cloud->points.size();
    pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(noise_meanK);
    sor.setStddevMulThresh(noise_stddev);
    sor.filter(*input_cloud);
    std::cout << "Removed " << original - input_cloud->points.size() << " Points as a noise" << std::endl;;
    return input_cloud;

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_noise_removal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) {
    int original = input_cloud->points.size();
    pcl::StatisticalOutlierRemoval <pcl::PointXYZRGB> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(noise_meanK);
    sor.setStddevMulThresh(noise_stddev);
    sor.filter(*input_cloud);
    std::cout << "Removed " << original - input_cloud->points.size() << " Points as a noise" << std::endl;;
    return input_cloud;

}


pcl::PointCloud<pcl::PointXYZ>::Ptr
point_prepare_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    input_cloud = point_cut_off_floor(input_cloud);
    input_cloud = point_downsample(input_cloud);
    // Without Noise Removal was better in test.
    std::cout << "Filtered Original cloud into " << input_cloud->points.size() << " Points." << std::endl;
    return input_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_get_reg_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_source, reduced_target;
    reduced_source = point_cloud_xyzrgb_to_xyz(
            source_cloud); //Here, we create new cloud. So you can handle reduced clouds regardless of orignial ones
    reduced_target = point_cloud_xyzrgb_to_xyz(target_cloud);
    reduced_source = point_prepare_icp(reduced_source);
    reduced_target = point_prepare_icp(reduced_target);
    //Prepare clouds to be icp-ed
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f target_to_source_transformation;
    //Transformation Matrix
    pcl::IterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> icp;
    //ICP Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud <pcl::PointXYZ>);
    std::vector<unsigned int> score;
    for (int i = 0; i < reg_iteration; i++) {
        icp.setInputSource(reduced_source);
        icp.setInputTarget(reduced_target);
        //set ICP
        icp.align(*reduced_source);
        transform *= icp.getFinalTransformation();
        unsigned int current_score = (unsigned int) icp.getFitnessScore();
        if (i > 1 && score.back() - current_score < 50) {
            std::cout << "ran" << i << "times, with score : " << icp.getFitnessScore() << std::endl;
            break;
        } else
            score.push_back(current_score);
    }
    target_to_source_transformation = transform.inverse();
    //"We got transformation matrix, now transforming target into source's frame"
    pcl::transformPointCloud(*target_cloud, *target_cloud, target_to_source_transformation);
    //"Transformation done. Returning transformed cloud"
    *source_cloud += *target_cloud;
    //Merge them
    return source_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_get_reg_from_vector(std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_vector) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr return_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);

    if (source_vector.size() < 1)
        return return_cloud;
    if (source_vector.size() == 1)
        return source_vector.at(0);
    // for a few exceptions..

    return_cloud = source_vector.at(0);
    for (int i = 0; i < source_vector.size() - 1; i++)
        return_cloud = point_get_reg_cloud(return_cloud, source_vector.at(i + 1));

    return return_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_cloud_fine_voxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud) {
    std::cout << "[Fine voxel filter] Origianl Clouds :" << source_cloud->points.size() << std::endl;


    std::cout << "[Fine voxel filter] Filtered Cloud :" << source_cloud->points.size() << std::endl;
    return source_cloud;
}


