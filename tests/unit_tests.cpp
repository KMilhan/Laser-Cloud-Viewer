#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "file/opener.hpp"
#include "kitti/kitti_utils.hpp"
#include "processing/cloud_processing.hpp"
#include <filesystem>

TEST_CASE("point_cut_off_floor removes outliers") {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 50;
    pcl::PointXYZ p2;
    p2.x = 0;
    p2.y = 0;
    p2.z = 150;
    pcl::PointXYZ p3;
    p3.x = 0;
    p3.y = 0;
    p3.z = 1000;
    cloud->push_back(p1);
    cloud->push_back(p2);
    cloud->push_back(p3);

    auto filtered = point_cut_off_floor(cloud);
    CHECK(filtered->size() == 2);
    bool has150 = false, has1000 = false;
    for (const auto &p : filtered->points) {
        if (p.z == doctest::Approx(150.f))
            has150 = true;
        if (p.z == doctest::Approx(1000.f))
            has1000 = true;
    }
    CHECK(has150);
    CHECK(has1000);
}

TEST_CASE("point_cloud_xyzrgb_to_xyz converts correctly") {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.r = 100;
    p.g = 150;
    p.b = 200;
    rgb->push_back(p);
    auto xyz = point_cloud_xyzrgb_to_xyz(rgb);
    REQUIRE(xyz->size() == 1);
    CHECK(xyz->points[0].x == doctest::Approx(1.f));
    CHECK(xyz->points[0].y == doctest::Approx(2.f));
    CHECK(xyz->points[0].z == doctest::Approx(3.f));
}

TEST_CASE("point_cloud_xyz_to_xyzrgb copies coordinates and assigns white") {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p;
    p.x = -1;
    p.y = -2;
    p.z = -3;
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

TEST_CASE("open_point_cloud_xyz_to_xyzrgb wraps XYZ input") {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    xyz->push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));
    auto rgb = open_point_cloud_xyz_to_xyzrgb(xyz);
    REQUIRE(rgb->size() == 1);
    CHECK(rgb->points[0].x == doctest::Approx(1.f));
    CHECK(rgb->points[0].y == doctest::Approx(2.f));
    CHECK(rgb->points[0].z == doctest::Approx(3.f));
    CHECK(rgb->points[0].r == 255);
    CHECK(rgb->points[0].g == 255);
    CHECK(rgb->points[0].b == 255);
}

TEST_CASE("point_downsample reduces duplicates") {
    reg_leaf_size = 1.0f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(0, 0, 0));
    cloud->push_back(pcl::PointXYZ(0.2f, 0.2f, 0.2f));
    cloud->push_back(pcl::PointXYZ(2, 2, 2));
    auto filtered = point_downsample(cloud);
    CHECK(filtered->size() == 2);
}

TEST_CASE("point_noise_removal eliminates outliers") {
    noise_meanK = 1;
    noise_stddev = 0.01f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(0, 0, 0));
    cloud->push_back(pcl::PointXYZ(0.1f, 0.1f, 0.1f));
    cloud->push_back(pcl::PointXYZ(100, 100, 100));
    auto filtered = point_noise_removal(cloud);
    CHECK(filtered->size() == 2);
}

TEST_CASE("point_noise_removal works on RGB clouds") {
    noise_meanK = 1;
    noise_stddev = 0.01f;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    pcl::PointXYZRGB p2;
    p2.x = 0.1f;
    p2.y = 0.1f;
    p2.z = 0.1f;
    pcl::PointXYZRGB p3;
    p3.x = 100;
    p3.y = 100;
    p3.z = 100;
    cloud->push_back(p1);
    cloud->push_back(p2);
    cloud->push_back(p3);
    auto filtered = point_noise_removal(cloud);
    CHECK(filtered->size() == 2);
}

TEST_CASE("kitti binary loads") {
    const std::string file = "../data/kitti_sample.bin";
    if (!std::filesystem::exists(file)) {
        doctest::skip("sample file missing");
    } else {
        auto cloud = loadKittiBin(file);
        CHECK(cloud->size() > 0);
    }
}

TEST_CASE("pcd scaling works") {
    const std::string pcd = "../data/sample.pcd";
    const std::string scale_file = "../data/scale.txt";
    if (!std::filesystem::exists(pcd) || !std::filesystem::exists(scale_file)) {
        doctest::skip("sample files missing");
    } else {
        auto rgb = point_cloud_open_pcd_file(pcd);
        auto scale = load_scale(scale_file);
        auto scaled = point_scale(rgb, scale);
        REQUIRE(rgb->size() == scaled->size());
        if (!rgb->empty()) {
            CHECK(scaled->points[0].x == doctest::Approx(rgb->points[0].x * scale[0]));
        }
    }
}
TEST_CASE("multiple clouds project to multiple image spaces") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZI>);

    cloudA->push_back({1.f, 2.f, 1.f, 0.f});
    cloudA->push_back({2.f, 1.f, 1.f, 0.f});
    cloudB->push_back({3.f, 4.f, 2.f, 0.f});

    Eigen::Matrix<float, 3, 4> projA = Eigen::Matrix<float, 3, 4>::Zero();
    projA(0, 0) = 1.f;
    projA(1, 1) = 1.f;
    projA(2, 2) = 1.f;

    Eigen::Matrix<float, 3, 4> projB = projA;
    projB(0, 3) = 2.f; // translate x

    auto pixA = projectToImagePlane(cloudA, projA);
    auto pixB = projectToImagePlane(cloudB, projB);

    REQUIRE(pixA.size() == 2);
    REQUIRE(pixB.size() == 1);

    CHECK(pixA[0].x() == doctest::Approx(1.f));
    CHECK(pixA[0].y() == doctest::Approx(2.f));
    CHECK(pixA[1].x() == doctest::Approx(2.f));
    CHECK(pixA[1].y() == doctest::Approx(1.f));
    CHECK(pixB[0].x() == doctest::Approx(2.5f));
    CHECK(pixB[0].y() == doctest::Approx(2.f));
}
