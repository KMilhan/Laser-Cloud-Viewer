#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "processing/cloud_processing.hpp"

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
