cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(LASER_Cloud_Viewer)

find_package(PCL 1.3 REQUIRED)
find_package(FLTK REQUIRED)
find_package(OpenGL REQUIRED)


if (CMAKE_COMPILER_IS_GNUCXX)
    message("Found GNUCXX!!!")
    set(CMAKE_C_FLAGS “-O3 -Wall -Werror -std=c++11”) ## Optimize Parameters for GCC
    set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary for Optimization
endif ()

if (OPENMP_FOUND)
    message("Found OPENMP!!!") ##
    find_package(OpenMP REQUIRED) ## Only in case you have it, find it
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

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
        )


add_executable(test test.cpp ${to_be_compiled}) ## test executable, "laserViewer"
add_executable(test_open visualize.cpp ${to_be_compiled}) ## visualizer
add_executable(test_register register.cpp ${to_be_compiled}) ## visualizer
add_executable(test_noise_cancel noise_remover.cpp ${to_be_compiled}) ## 
add_executable(test_segment segemt.cpp ${to_be_compiled}) ## 
add_executable(test_save save.cpp ${to_be_compiled}) ## 
add_executable(Laser_Cloud_Viewer ui.cxx ${to_be_compiled})

target_link_libraries(test ${PCL_LIBRARIES})
target_link_libraries(test_open ${PCL_LIBRARIES})
target_link_libraries(test_register ${PCL_LIBRARIES})
target_link_libraries(test_noise_cancel ${PCL_LIBRARIES})
target_link_libraries(test_segment ${PCL_LIBRARIES})
target_link_libraries(test_save ${PCL_LIBRARIES})
target_link_libraries(Laser_Cloud_Viewer ${PCL_LIBRARIES} ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES})
