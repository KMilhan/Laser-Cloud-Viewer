#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "processing/cloud_processing.hpp"
#include "file/opener.hpp"
#include "kitti/kitti_utils.hpp"

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

TEST_CASE("point_cloud_xyzrgb_to_xyz converts correctly") {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB p; p.x = 1; p.y = 2; p.z = 3; p.r = 100; p.g = 150; p.b = 200;
    rgb->push_back(p);
    auto xyz = point_cloud_xyzrgb_to_xyz(rgb);
    REQUIRE(xyz->size() == 1);
    CHECK(xyz->points[0].x == doctest::Approx(1.f));
    CHECK(xyz->points[0].y == doctest::Approx(2.f));
    CHECK(xyz->points[0].z == doctest::Approx(3.f));
}

TEST_CASE("point_cloud_xyz_to_xyzrgb copies coordinates and assigns white") {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p; p.x = -1; p.y = -2; p.z = -3;
    xyz->push_back(p);
    auto rgb = point_cloud_xyz_to_xyzrgb(xyz);
    REQUIRE(rgb->size() == 1);
    CHECK(rgb->points[0].x == doctest::Approx(-1.f));
    CHECK(rgb->points[0].y == doctest::Approx(-2.f));
    CHECK(rgb->points[0].z == doctest::Approx(-3.f));
    CHECK(rgb->points[0].r == 255);
    CHECK(rgb->points[0].g == 255);
    CHECK(rgb->points[0].b == 255);
}

TEST_CASE("point_downsample reduces duplicates") {
    reg_leaf_size = 1.0f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(0,0,0));
    cloud->push_back(pcl::PointXYZ(0.2f,0.2f,0.2f));
    cloud->push_back(pcl::PointXYZ(2,2,2));
    auto filtered = point_downsample(cloud);
    CHECK(filtered->size() == 2);
}

TEST_CASE("point_noise_removal eliminates outliers") {
    noise_meanK = 1;
    noise_stddev = 0.01f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(0,0,0));
    cloud->push_back(pcl::PointXYZ(0.1f,0.1f,0.1f));
    cloud->push_back(pcl::PointXYZ(100,100,100));
    auto filtered = point_noise_removal(cloud);
    CHECK(filtered->size() == 2);
}

TEST_CASE("point_noise_removal works on RGB clouds") {
    noise_meanK = 1;
    noise_stddev = 0.01f;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB p1; p1.x = 0; p1.y = 0; p1.z = 0;
    pcl::PointXYZRGB p2; p2.x = 0.1f; p2.y = 0.1f; p2.z = 0.1f;
    pcl::PointXYZRGB p3; p3.x = 100; p3.y = 100; p3.z = 100;
    cloud->push_back(p1);
    cloud->push_back(p2);
    cloud->push_back(p3);
    auto filtered = point_noise_removal(cloud);
    CHECK(filtered->size() == 2);
}

TEST_CASE("kitti binary loads") {
    auto cloud = loadKittiBin("../data/kitti_sample.bin");
    CHECK(cloud->size() > 0);
}

TEST_CASE("pcd scaling works") {
    auto rgb = point_cloud_open_pcd_file("../data/sample.pcd");
    auto scale = load_scale("../data/scale.txt");
    auto scaled = point_scale(rgb, scale);
    REQUIRE(rgb->size() == scaled->size());
    if (!rgb->empty()) {
        CHECK(scaled->points[0].x == doctest::Approx(rgb->points[0].x * scale[0]));
    }
}
