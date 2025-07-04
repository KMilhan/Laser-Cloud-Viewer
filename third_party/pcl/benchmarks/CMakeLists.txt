set(SUBSYS_NAME benchmarks)
set(SUBSYS_DESC "Point cloud library benchmarks")
set(SUBSYS_DEPS common filters features search kdtree io)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW) # FindBoost module removed
endif()


PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" OFF)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
if(NOT build)
  return()
endif()

find_package(benchmark REQUIRED)
get_target_property(BenchmarkBuildType benchmark::benchmark TYPE)

add_custom_target(run_benchmarks)

PCL_ADD_BENCHMARK(features_normal_3d FILES features/normal_3d.cpp
                  LINK_WITH pcl_io pcl_search pcl_features
                  ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd"
                            "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

PCL_ADD_BENCHMARK(filters_voxel_grid FILES filters/voxel_grid.cpp
                  LINK_WITH pcl_io pcl_filters
                  ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd"
                            "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

PCL_ADD_BENCHMARK(filters_radius_outlier_removal FILES filters/radius_outlier_removal.cpp
                  LINK_WITH pcl_io pcl_filters
                  ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd"
                            "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

PCL_ADD_BENCHMARK(search_radius_search FILES search/radius_search.cpp
                  LINK_WITH pcl_io pcl_search
                  ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd"
                            "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

