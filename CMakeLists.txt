cmake_minimum_required(VERSION 3.10)
project(LASER_Cloud_Viewer LANGUAGES C CXX)

option(ENABLE_TESTS "Build unit tests with doctest" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.3 REQUIRED)
find_package(MPI REQUIRED)
find_package(FLTK REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(JsonCpp CONFIG REQUIRED)


if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Using GCC")
    add_compile_options(-O3 -Wall -Wextra)
endif()
if (OpenMP_CXX_FOUND)
    add_compile_options(${OpenMP_CXX_FLAGS})
endif()

##PCL
include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
##PCL

set(to_be_compiled
        model.cpp
        file/opener.cpp
        simple_vis/simple_vis.cpp
        processing/cloud_processing.cpp
        kitti/kitti_utils.cpp
        kitti/scale_converter.cpp
        )


add_executable(example_app test.cpp ${to_be_compiled}) ## test executable, "laserViewer"
add_executable(test_open visualize.cpp ${to_be_compiled}) ## visualizer
add_executable(test_register register.cpp ${to_be_compiled}) ## visualizer
add_executable(test_noise_cancel noise_remover.cpp ${to_be_compiled}) ## 
add_executable(test_segment segemt.cpp ${to_be_compiled}) ## 
add_executable(test_save save.cpp ${to_be_compiled}) ##
add_executable(Laser_Cloud_Viewer ui.cxx ${to_be_compiled})
add_executable(scale_to_kitti_cli scale_to_kitti_cli.cpp ${to_be_compiled})

target_link_libraries(example_app ${PCL_LIBRARIES} Boost::thread)
target_link_libraries(test_open ${PCL_LIBRARIES} Boost::thread)
target_link_libraries(test_register ${PCL_LIBRARIES} Boost::thread)
target_link_libraries(test_noise_cancel ${PCL_LIBRARIES} Boost::thread)
target_link_libraries(test_segment ${PCL_LIBRARIES} Boost::thread)
target_link_libraries(test_save ${PCL_LIBRARIES} Boost::thread)
target_link_libraries(Laser_Cloud_Viewer ${PCL_LIBRARIES} ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES} Boost::thread)
target_link_libraries(scale_to_kitti_cli JsonCpp::JsonCpp)

if(ENABLE_TESTS)
    enable_testing()
    add_executable(unit_tests tests/unit_tests.cpp ${to_be_compiled})
    target_include_directories(unit_tests PRIVATE third_party)
    target_link_libraries(unit_tests ${PCL_LIBRARIES} Boost::thread JsonCpp::JsonCpp)
    add_test(NAME unit_tests COMMAND unit_tests)
endif()
