#include "opener.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
open_point_cloud_xyz_to_xyzrgb(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // assign return cloud
#pragma omp parallel for
    for (int i = 0; i < xyz_cloud->points.size(); i++) {
        pcl::PointXYZRGB point;
        // copy coordinates from source cloud
        point.x = xyz_cloud->points[i].x;
        point.y = xyz_cloud->points[i].y;
        point.z = xyz_cloud->points[i].z;
        point.r = 255;
        point.g = 255;
        point.b = 255;
#pragma omp critical(dataupdate)
        {
            rgb_cloud->points.push_back(point);
        }
    }

    rgb_cloud->width = (int)xyz_cloud->points.size();
    rgb_cloud->height = 1;
    return rgb_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_from_txt(std::string file_path) {
    std::string data(file_path); // open the file
    std::ifstream in(data.c_str());
    if (!in.is_open())
        std::cout << "File Error";

    std::vector<std::string> txtFileStorage; // Contains unsorted txt file line

    typedef boost::tokenizer<boost::escaped_list_separator<char>> tokenizer;
    // Make a boost tokenizer

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temporal_point_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    // Make a pointer to cloud

    std::string line;
    // get lines

    while (getline(in, line)) {
        tokenizer tok(line);
        // Token line
        txtFileStorage.assign(tok.begin(), tok.end());
        // Copy to vector

        if (txtFileStorage.size() != 6 && txtFileStorage.size() != 3)
            continue;
        // Check XYZRGB or XYZ File Type

        pcl::PointXYZRGB point;
        // temporal point
        float x = boost::lexical_cast<float>(txtFileStorage.at(0));
        float y = boost::lexical_cast<float>(txtFileStorage.at(1));
        float z = boost::lexical_cast<float>(txtFileStorage.at(2));
        uint8_t r = 255; // For given XYZ data, set all color into WHITE
        uint8_t g = 255;
        uint8_t b = 255;
        if (txtFileStorage.size() != 3) {
            uint8_t r = boost::lexical_cast<short int>(txtFileStorage.at(3));
            uint8_t g = boost::lexical_cast<short int>(txtFileStorage.at(4));
            uint8_t b = boost::lexical_cast<short int>(txtFileStorage.at(5));

            // cast strings into float and int
        }

        point.x = x;
        point.y = y;
        point.z = z;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 |
                        static_cast<uint32_t>(b));
        // Calc RGB
        point.rgb = *reinterpret_cast<float *>(&rgb);
        temporal_point_cloud_ptr->points.push_back(point);
        // Assign point
    }
    temporal_point_cloud_ptr->width = (int)temporal_point_cloud_ptr->points.size();
    temporal_point_cloud_ptr->height = 1;
    std::cout << temporal_point_cloud_ptr->points.size() << " Points from file:" << data
              << std::endl;

    return temporal_point_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_open_pcd_file(std::string file_path) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud) != 0) {
        std::cout << "Cloud reading failed. may be xyz cloud " << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *xyz_cloud) != 0) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr converted_cloud(
                new pcl::PointCloud<pcl::PointXYZRGB>);
            converted_cloud = open_point_cloud_xyz_to_xyzrgb(xyz_cloud);
            cloud = converted_cloud;
        }
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_open_file(std::string file_path) {
    if (file_path.substr(file_path.find_last_of(".") + 1) == "txt")
        return point_cloud_from_txt(file_path);
    else if (file_path.substr(file_path.find_last_of(".") + 1) == "pcd")
        return point_cloud_open_pcd_file(file_path);

    std::cout << "unsupported file" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    return cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> open_test_files() {

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> return_vector;
    std::vector<std::string> file_path_vector;
    file_path_vector.push_back("./data/example.txt");

    std::cout << "Opening [" << file_path_vector.size() << "] Files " << std::endl;

#pragma omp parallel for // Parse files concurrently
    for (int i = 0; i < file_path_vector.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if ((cloud = point_cloud_open_file(file_path_vector.at(i))) == NULL)
            continue;
#pragma omp critical(dataupdate)
        return_vector.push_back(cloud);
    }

    return return_vector;
}
