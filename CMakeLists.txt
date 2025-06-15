cmake_minimum_required(VERSION 3.16)

# Adopt modern CMake policies when available
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW) # find_package uses <PACKAGENAME>_ROOT
endif()
if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW) # Prefer GLVND for OpenGL
endif()
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW) # FindBoost module removed
endif()

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


if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Using GCC")
    add_compile_options(-O3 -Wall -Wextra)
endif()
if (OpenMP_CXX_FOUND)
    add_compile_options(${OpenMP_CXX_FLAGS})
endif()

## PCL include directories are provided by the imported targets

set(LCV_SOURCES
    model.cpp
    file/opener.cpp
    simple_vis/simple_vis.cpp
    processing/cloud_processing.cpp
    kitti/kitti_utils.cpp
)

add_library(lcv STATIC ${LCV_SOURCES})
target_compile_features(lcv PUBLIC cxx_std_17)
target_include_directories(lcv PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lcv PUBLIC ${PCL_LIBRARIES} Boost::thread)


add_executable(test_open visualize.cpp)
add_executable(test_register register.cpp)
add_executable(test_noise_cancel noise_remover.cpp)
add_executable(test_segment segemt.cpp)
add_executable(test_save save.cpp)
add_executable(Laser_Cloud_Viewer ui.cxx)

target_link_libraries(test_open lcv)
target_link_libraries(test_register lcv)
target_link_libraries(test_noise_cancel lcv)
target_link_libraries(test_segment lcv)
target_link_libraries(test_save lcv)
target_link_libraries(Laser_Cloud_Viewer lcv)

target_link_libraries(Laser_Cloud_Viewer ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES})

if(ENABLE_TESTS)
    enable_testing()
    add_executable(unit_tests tests/unit_tests.cpp)
    target_include_directories(unit_tests PRIVATE third_party)
    target_link_libraries(unit_tests lcv)
    add_test(NAME unit_tests COMMAND unit_tests)
endif()
