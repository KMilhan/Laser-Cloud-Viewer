#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "processing/cloud_processing.hpp"
#include "kitti/scale_converter.hpp"
#include <fstream>
#include <filesystem>

TEST_CASE("point_cut_off_floor removes outliers") {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p1; p1.x = 0; p1.y = 0; p1.z = 50;
    pcl::PointXYZ p2; p2.x = 0; p2.y = 0; p2.z = 150;
    pcl::PointXYZ p3; p3.x = 0; p3.y = 0; p3.z = 1000;
    cloud->push_back(p1);
    cloud->push_back(p2);
    cloud->push_back(p3);

    auto filtered = point_cut_off_floor(cloud);
    CHECK(filtered->size() == 2);
    bool has150 = false, has1000 = false;
    for (const auto& p : filtered->points) {
        if (p.z == doctest::Approx(150.f)) has150 = true;
        if (p.z == doctest::Approx(1000.f)) has1000 = true;
    }
    CHECK(has150);
    CHECK(has1000);
}

TEST_CASE("convert_scale_to_kitti writes label") {
    namespace fs = std::filesystem;
    const char* json_path = "tests/sample_scale.json";
    const char* out_dir = "tests/tmp_out";
    fs::create_directories(out_dir);
    std::ofstream js(json_path);
    js << "{\n"
          "  \"image\": \"000000.png\",\n"
          "  \"annotations\": [\n"
          "    {\n"
          "      \"label\": \"Car\",\n"
          "      \"bbox\": {\"left\": 0, \"top\": 1, \"width\": 2, \"height\": 3},\n"
          "      \"dimensions\": {\"height\": 1, \"width\": 2, \"length\": 3},\n"
          "      \"location\": {\"x\": 4, \"y\": 5, \"z\": 6},\n"
          "      \"rotation_y\": 0.5\n"
          "    }\n"
          "  ]\n"
          "}\n";
    js.close();

    CHECK(convert_scale_to_kitti(json_path, out_dir));

    std::ifstream label(std::string(out_dir) + "/label_2/000000.txt");
    CHECK(label.is_open());
    std::string line; std::getline(label, line);
    CHECK(line.find("Car") != std::string::npos);
    label.close();

    fs::remove_all(out_dir);
    fs::remove(json_path);
}
