# ------------------------------------------------------------------------------------
# Helper to use PCL from outside project
#
# target_link_libraries(my_fabulous_target PCL_XXX_LIBRARIES) where XXX is the
# upper cased xxx from :
# 
# - common
# - kdtree
# - octree
# - search
# - sample_consensus
# - filters
# - 2d
# - geometry
# - io
# - features
# - ml
# - segmentation
# - visualization
# - surface
# - registration
# - keypoints
# - tracking
# - recognition
# - stereo
# - outofcore
# - people
#
# PCL_INCLUDE_DIRS is filled with PCL and available 3rdparty headers
# PCL_LIBRARY_DIRS is filled with PCL components libraries install directory and
# 3rdparty libraries paths
#
#                                   www.pointclouds.org
#------------------------------------------------------------------------------------

# Set default policy behavior similar to minimum requirement version
cmake_policy(VERSION 3.16.3)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/Modules")

### ---[ some useful macros
macro(pcl_report_not_found _reason)
  unset(PCL_FOUND)
  unset(PCL_LIBRARIES)
  unset(PCL_COMPONENTS)
  unset(PCL_INCLUDE_DIRS)
  unset(PCL_LIBRARY_DIRS)
  unset(PCL_DEFINITIONS)
  if(PCL_FIND_REQUIRED)
    message(FATAL_ERROR ${_reason})
  elseif(NOT PCL_FIND_QUIETLY)
    message(WARNING ${_reason})
  endif()
  return()
endmacro()

macro(pcl_message)
  if(NOT PCL_FIND_QUIETLY)
    message(${ARGN})
  endif()
endmacro()

# Remove duplicate libraries
macro(pcl_remove_duplicate_libraries _unfiltered_libraries _filtered_libraries)
  set(${_filtered_libraries})
  set(_debug_libraries)
  set(_optimized_libraries)
  set(_other_libraries)
  set(_waiting_for_debug 0)
  set(_waiting_for_optimized 0)
  set(_library_position -1)
  foreach(library ${${_unfiltered_libraries}})
    if("${library}" STREQUAL "debug")
      set(_waiting_for_debug 1)
    elseif("${library}" STREQUAL "optimized")
      set(_waiting_for_optimized 1)
    elseif(_waiting_for_debug)
      list(FIND _debug_libraries "${library}" library_position)
      if(library_position EQUAL -1)
        list(APPEND ${_filtered_libraries} debug ${library})
        list(APPEND _debug_libraries ${library})
      endif()
      set(_waiting_for_debug 0)
    elseif(_waiting_for_optimized)
      list(FIND _optimized_libraries "${library}" library_position)
      if(library_position EQUAL -1)
        list(APPEND ${_filtered_libraries} optimized ${library})
        list(APPEND _optimized_libraries ${library})
      endif()
      set(_waiting_for_optimized 0)
    else()
      list(FIND _other_libraries "${library}" library_position)
      if(library_position EQUAL -1)
        list(APPEND ${_filtered_libraries} ${library})
        list(APPEND _other_libraries ${library})
      endif()
    endif()
  endforeach()
endmacro()

### ---[ 3rd party libraries
macro(find_boost)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(BOOST_ROOT "${PCL_ROOT}/3rdParty/Boost")
  elseif(NOT BOOST_INCLUDEDIR)
    set(BOOST_INCLUDEDIR "/usr/include")
  endif()

  find_package(Boost 1.71.0 ${QUIET_} COMPONENTS system iostreams filesystem serialization CONFIG)

  set(BOOST_FOUND ${Boost_FOUND})
  set(BOOST_INCLUDE_DIRS "${Boost_INCLUDE_DIR}")
  set(BOOST_LIBRARY_DIRS "${Boost_LIBRARY_DIRS}")
  set(BOOST_LIBRARIES ${Boost_LIBRARIES})
  if(WIN32 AND NOT MINGW AND NOT "${BOOST_DEFINITIONS}" MATCHES "BOOST_ALL_NO_LIB")
    string(APPEND BOOST_DEFINITIONS -DBOOST_ALL_NO_LIB)
  endif()
endmacro()

macro(find_eigen3)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(Eigen3_DIR "${PCL_ROOT}/3rdParty/Eigen3/share/eigen3/cmake/")
  endif()
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
  if(NOT EIGEN3_FOUND AND Eigen3_FOUND)
    set(EIGEN3_FOUND ${Eigen3_FOUND})
  endif()
  # In very new Eigen versions, EIGEN3_INCLUDE_DIR(S) is not defined any more, only the target:
  if(TARGET Eigen3::Eigen)
    set(EIGEN3_LIBRARIES Eigen3::Eigen)
  endif()
endmacro()

#remove this as soon as qhull is shipped with FindQhull.cmake
macro(find_qhull)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(QHULL_ROOT "${PCL_ROOT}/3rdParty/Qhull")
  elseif(NOT QHULL_ROOT)
    get_filename_component(QHULL_ROOT "" PATH)
  endif()

  set(PCL_QHULL_REQUIRED_TYPE DONTCARE)
  find_package(Qhull)
endmacro()

#remove this as soon as libopenni is shipped with FindOpenni.cmake
macro(find_openni)
  if(PCL_FIND_QUIETLY)
    set(OpenNI_FIND_QUIETLY TRUE)
  endif()

  if(NOT OPENNI_ROOT AND ("" STREQUAL "TRUE"))
    set(OPENNI_INCLUDE_DIRS_HINT "")
    get_filename_component(OPENNI_LIBRARY_HINT "OPENNI_LIBRARY-NOTFOUND" PATH)
  endif()

  find_package(OpenNI)
endmacro()

#remove this as soon as libopenni2 is shipped with FindOpenni2.cmake
macro(find_openni2)
  if(PCL_FIND_QUIETLY)
    set(OpenNI2_FIND_QUIETLY TRUE)
  endif()

  if(NOT OPENNI2_ROOT AND ("" STREQUAL "TRUE"))
    set(OPENNI2_INCLUDE_DIRS_HINT "")
    get_filename_component(OPENNI2_LIBRARY_HINT "OPENNI2_LIBRARY-NOTFOUND" PATH)
  endif()

  find_package(OpenNI2)
endmacro()

#remove this as soon as the Ensenso SDK is shipped with FindEnsenso.cmake
macro(find_ensenso)
  if(PCL_FIND_QUIETLY)
    set(ensenso_FIND_QUIETLY TRUE)
  endif()

  if(NOT ENSENSO_ROOT AND ("" STREQUAL "TRUE"))
    get_filename_component(ENSENSO_ABI_HINT "ENSENSO_INCLUDE_DIR-NOTFOUND" PATH)
  endif()

  find_package(Ensenso)
endmacro()

#remove this as soon as the davidSDK is shipped with FinddavidSDK.cmake
macro(find_davidSDK)
  if(PCL_FIND_QUIETLY)
    set(DAVIDSDK_FIND_QUIETLY TRUE)
  endif()

  if(NOT davidSDK_ROOT AND ("" STREQUAL "TRUE"))
    get_filename_component(DAVIDSDK_ABI_HINT DAVIDSDK_INCLUDE_DIR-NOTFOUND PATH)
  endif()

  find_package(davidSDK)
endmacro()

macro(find_dssdk)
  if(PCL_FIND_QUIETLY)
    set(DSSDK_FIND_QUIETLY TRUE)
  endif()
  if(NOT DSSDK_DIR AND ("" STREQUAL "TRUE"))
    get_filename_component(DSSDK_DIR_HINT "" PATH)
  endif()

  find_package(DSSDK)
endmacro()

macro(find_rssdk)
  if(PCL_FIND_QUIETLY)
    set(RSSDK_FIND_QUIETLY TRUE)
  endif()
  if(NOT RSSDK_DIR AND ("" STREQUAL "TRUE"))
    get_filename_component(RSSDK_DIR_HINT "" PATH)
  endif()

  find_package(RSSDK)
endmacro()

macro(find_rssdk2)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(realsense2_DIR "${PCL_ROOT}/3rdParty/librealsense2/lib/cmake/realsense2" CACHE PATH "The directory containing realsense2Config.cmake")
  elseif(NOT realsense2_DIR)
    get_filename_component(realsense2_DIR "" PATH)
    set(realsense2_DIR "${realsense2_DIR}/lib/cmake/realsense2" CACHE PATH "The directory containing realsense2Config.cmake")
  endif()
  find_package(RSSDK2)
endmacro()

#remove this as soon as flann is shipped with FindFlann.cmake
macro(find_flann)
  if(PCL_ALL_IN_ONE_INSTALLER)
    set(FLANN_ROOT "${PCL_ROOT}/3rdParty/Flann")
  elseif(NOT FLANN_ROOT)
    set(FLANN_ROOT "/usr")
  endif()

  set(PCL_FLANN_REQUIRED_TYPE DONTCARE)
  find_package(FLANN)
endmacro()

macro(find_VTK)
  if(PCL_ALL_IN_ONE_INSTALLER AND NOT ANDROID)
    if(EXISTS "${PCL_ROOT}/3rdParty/VTK/lib/cmake")
      set(VTK_DIR "${PCL_ROOT}/3rdParty/VTK/lib/cmake/vtk-9.4" CACHE PATH "The directory containing VTKConfig.cmake")
    else()
      set(VTK_DIR "${PCL_ROOT}/3rdParty/VTK/lib/vtk-9.4" CACHE PATH "The directory containing VTKConfig.cmake")
    endif()
  elseif(NOT VTK_DIR AND NOT ANDROID)
    set(VTK_DIR "/usr/lib/cmake/vtk" CACHE PATH "The directory containing VTKConfig.cmake")
  endif()
  if(NOT ANDROID)
    find_package(VTK ${QUIET_} COMPONENTS ${PCL_VTK_COMPONENTS})
  endif()
endmacro()

macro(find_libusb)
  find_package(libusb)
endmacro()

macro(find_glew)
  find_package(GLEW)
endmacro()

# Finds each component external libraries if any
# The functioning is as following
# try to find _lib
# |--> _lib found ==> include the headers,
# |                   link to its library directories or include _lib_USE_FILE
# `--> _lib not found
#                   |--> _lib is optional ==> disable it (thanks to the guardians)
#                   |                         and warn
#                   `--> _lib is required
#                                       |--> component is required explicitly ==> error
#                                       `--> component is induced ==> warn and remove it
#                                                                     from the list

function(find_external_library _component _lib _is_optional)
  if("${_lib}" STREQUAL "boost")
    find_boost()
  elseif("${_lib}" STREQUAL "eigen3")
    find_eigen3()
  elseif("${_lib}" STREQUAL "flann")
    find_flann()
  elseif("${_lib}" STREQUAL "qhull")
    find_qhull()
  elseif("${_lib}" STREQUAL "openni")
    find_openni()
  elseif("${_lib}" STREQUAL "openni2")
    find_openni2()
  elseif("${_lib}" STREQUAL "ensenso")
    find_ensenso()
  elseif("${_lib}" STREQUAL "davidSDK")
    find_davidSDK()
  elseif("${_lib}" STREQUAL "dssdk")
    find_dssdk()
  elseif("${_lib}" STREQUAL "rssdk")
    find_rssdk()
  elseif("${_lib}" STREQUAL "rssdk2")
    find_rssdk2()
  elseif("${_lib}" STREQUAL "vtk")
    find_VTK()
  elseif("${_lib}" STREQUAL "libusb")
    find_libusb()
  elseif("${_lib}" STREQUAL "glew")
    find_glew()
  elseif("${_lib}" STREQUAL "opengl")
    find_package(OpenGL)
  elseif("${_lib}" STREQUAL "pcap")
    find_package(Pcap)
  elseif("${_lib}" STREQUAL "png")
    find_package(PNG)
  elseif("${_lib}" STREQUAL "OpenMP")
    find_package(OpenMP COMPONENTS CXX)
    # the previous find_package call sets OpenMP_CXX_LIBRARIES, but not OPENMP_LIBRARIES, which is used further down
    # we can link to the CMake target OpenMP::OpenMP_CXX by setting the following:
    set(OPENMP_LIBRARIES OpenMP::OpenMP_CXX)
  else()
    message(WARNING "${_lib} is not handled by find_external_library")
  endif()

  string(TOUPPER "${_component}" COMPONENT)
  string(TOUPPER "${_lib}" LIB)
  string(REGEX REPLACE "[.-]" "_" LIB "${LIB}")
  if(${LIB}_FOUND)
    list(APPEND PCL_${COMPONENT}_INCLUDE_DIRS ${${LIB}_INCLUDE_DIRS})
    set(PCL_${COMPONENT}_INCLUDE_DIRS ${PCL_${COMPONENT}_INCLUDE_DIRS} PARENT_SCOPE)

    if(${LIB} MATCHES "VTK")
      if(${${LIB}_VERSION_MAJOR} GREATER_EQUAL 9)
        set(ISVTK9ORGREATER TRUE)
      endif()
    endif()

    if(${LIB}_USE_FILE AND NOT ISVTK9ORGREATER )
      include(${${LIB}_USE_FILE})
    else()
      list(APPEND PCL_${COMPONENT}_LIBRARY_DIRS "${${LIB}_LIBRARY_DIRS}")
      set(PCL_${COMPONENT}_LIBRARY_DIRS ${PCL_${COMPONENT}_LIBRARY_DIRS} PARENT_SCOPE)
    endif()
    if(${LIB}_LIBRARIES)
      list(APPEND PCL_${COMPONENT}_LINK_LIBRARIES "${${LIB}_LIBRARIES}")
      set(PCL_${COMPONENT}_LINK_LIBRARIES ${PCL_${COMPONENT}_LINK_LIBRARIES} PARENT_SCOPE)
      set(PCL_${LIB}_LIBRARIES ${${LIB}_LIBRARIES} PARENT_SCOPE) # Later appended to PCL_LIBRARIES
    endif()
    if(${LIB}_DEFINITIONS AND NOT ${LIB} STREQUAL "VTK")
      list(APPEND PCL_${COMPONENT}_DEFINITIONS ${${LIB}_DEFINITIONS})
      set(PCL_${COMPONENT}_DEFINITIONS ${PCL_${COMPONENT}_DEFINITIONS} PARENT_SCOPE)
    endif()
  else()
    if("${_is_optional}" STREQUAL "OPTIONAL")
      list(APPEND PCL_${COMPONENT}_DEFINITIONS "-DDISABLE_${LIB}")
      pcl_message("** WARNING ** ${_component} features related to ${_lib} will be disabled")
    elseif("${_is_optional}" STREQUAL "REQUIRED")
      if((NOT PCL_FIND_ALL) OR (PCL_FIND_ALL EQUAL 1))
        pcl_report_not_found("${_component} is required but ${_lib} was not found")
      elseif(PCL_FIND_ALL EQUAL 0)
        # raise error and remove _component from PCL_TO_FIND_COMPONENTS
        string(TOUPPER "${_component}" COMPONENT)
        pcl_message("** WARNING ** ${_component} will be disabled cause ${_lib} was not found")
        list(REMOVE_ITEM PCL_TO_FIND_COMPONENTS ${_component})
      endif()
    endif()
  endif()
endfunction()

macro(pcl_check_external_dependency _component)
endmacro()

#flatten dependencies recursivity is great \o/
macro(compute_dependencies TO_FIND_COMPONENTS)
  foreach(component ${${TO_FIND_COMPONENTS}})
    set(pcl_component pcl_${component})
    if(${pcl_component}_int_dep AND (NOT PCL_FIND_ALL))
      foreach(dependency ${${pcl_component}_int_dep})
        list(FIND ${TO_FIND_COMPONENTS} ${component} pos)
        list(FIND ${TO_FIND_COMPONENTS} ${dependency} found)
        if(found EQUAL -1)
          set(pcl_dependency pcl_${dependency})
          if(${pcl_dependency}_int_dep)
            list(INSERT ${TO_FIND_COMPONENTS} ${pos} ${dependency})
            if(pcl_${dependency}_ext_dep)
              list(APPEND pcl_${component}_ext_dep ${pcl_${dependency}_ext_dep})
            endif()
            if(pcl_${dependency}_opt_dep)
              list(APPEND pcl_${component}_opt_dep ${pcl_${dependency}_opt_dep})
            endif()
            compute_dependencies(${TO_FIND_COMPONENTS})
          else()
            list(INSERT ${TO_FIND_COMPONENTS} 0 ${dependency})
          endif()
        endif()
      endforeach()
    endif()
  endforeach()
endmacro()

### ---[ Find PCL

if(PCL_FIND_QUIETLY)
  set(QUIET_ QUIET)
else()
  set(QUIET_)
endif()

find_package(PkgConfig QUIET)

file(TO_CMAKE_PATH "${PCL_DIR}" PCL_DIR)
if(WIN32 AND NOT MINGW)
# PCLConfig.cmake is installed to PCL_ROOT/cmake
  get_filename_component(PCL_ROOT "${PCL_DIR}" PATH)
  if(EXISTS "${PCL_ROOT}/3rdParty")
    set(PCL_ALL_IN_ONE_INSTALLER ON)
  endif()
else()
# PCLConfig.cmake is installed to PCL_ROOT/share/pcl-x.y
  get_filename_component(PCL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
endif()

# check whether PCLConfig.cmake is found into a PCL installation or in a build tree
if(EXISTS "${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}/pcl/pcl_config.h")
  # Found a PCL installation
  # pcl_message("Found a PCL installation")
  set(PCL_CONF_INCLUDE_DIR "${PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
  set(PCL_LIBRARY_DIRS "${PCL_ROOT}/lib")
elseif(EXISTS "${PCL_ROOT}/include/pcl/pcl_config.h")
  # Found a non-standard (likely ANDROID) PCL installation
  # pcl_message("Found a PCL installation")
  set(PCL_CONF_INCLUDE_DIR "${PCL_ROOT}/include")
  set(PCL_LIBRARY_DIRS "${PCL_ROOT}/lib")
elseif(EXISTS "${PCL_DIR}/include/pcl/pcl_config.h")
  # Found PCLConfig.cmake in a build tree of PCL
  # pcl_message("PCL found into a build tree.")
  set(PCL_CONF_INCLUDE_DIR "${PCL_DIR}/include") # for pcl_config.h
  set(PCL_LIBRARY_DIRS "${PCL_DIR}/lib")
else()
  pcl_report_not_found("PCL can not be found on this machine")
endif()

set(PCL_INCLUDE_DIRS "${PCL_CONF_INCLUDE_DIR}")

#set a suffix for debug libraries
set(PCL_DEBUG_SUFFIX "")
set(PCL_RELEASE_SUFFIX "")
set(PCL_RELWITHDEBINFO_SUFFIX "")
set(PCL_MINSIZEREL_SUFFIX "")

set(PCL_SHARED_LIBS "ON")

#set SSE flags used compiling PCL
list(APPEND PCL_DEFINITIONS )
list(APPEND PCL_COMPILE_OPTIONS -msse4.2 -mfpmath=sse -march=native)

#set AVX flags used compiling PCL
list(APPEND PCL_COMPILE_OPTIONS -mavx2)

set(pcl_all_components  common kdtree octree search sample_consensus filters 2d geometry io features ml segmentation visualization surface registration keypoints tracking recognition stereo outofcore people)
# insert "io_ply" before "io"
list(FIND pcl_all_components "io" pcl_pos_io)
list(INSERT pcl_all_components ${pcl_pos_io} "io_ply")
unset(pcl_pos_io)
list(LENGTH pcl_all_components PCL_NB_COMPONENTS)

#list each component dependencies IN PCL
set(pcl_kdtree_int_dep common )
set(pcl_octree_int_dep common )
set(pcl_search_int_dep common kdtree octree )
set(pcl_sample_consensus_int_dep common search )
set(pcl_filters_int_dep common sample_consensus search kdtree octree )
set(pcl_2d_int_dep common filters )
set(pcl_geometry_int_dep common )
set(pcl_io_int_dep common octree )
set(pcl_features_int_dep common search kdtree octree filters 2d )
set(pcl_ml_int_dep common )
set(pcl_segmentation_int_dep common geometry search sample_consensus kdtree octree features filters ml )
set(pcl_visualization_int_dep common io kdtree geometry search octree )
set(pcl_surface_int_dep common search kdtree octree )
set(pcl_registration_int_dep common octree kdtree search sample_consensus features filters )
set(pcl_keypoints_int_dep common search kdtree octree features filters )
set(pcl_tracking_int_dep common search kdtree filters octree )
set(pcl_recognition_int_dep common io search kdtree octree features filters registration sample_consensus ml )
set(pcl_stereo_int_dep common io )
set(pcl_outofcore_int_dep common io filters octree visualization )
set(pcl_people_int_dep common kdtree search sample_consensus filters io visualization geometry segmentation octree )


#list each component external dependencies (ext means mandatory and opt means optional)
set(pcl_common_ext_dep eigen3 boost )
set(pcl_kdtree_ext_dep flann )
set(pcl_search_ext_dep flann )
set(pcl_io_ext_dep boost eigen3 )
set(pcl_visualization_ext_dep vtk )


set(pcl_filters_opt_dep OpenMP )
set(pcl_io_opt_dep pcap png vtk libusb OpenMP )
set(pcl_features_opt_dep OpenMP )
set(pcl_segmentation_opt_dep OpenMP )
set(pcl_visualization_opt_dep )
set(pcl_surface_opt_dep qhull vtk OpenMP )
set(pcl_registration_opt_dep OpenMP )
set(pcl_keypoints_opt_dep OpenMP )
set(pcl_tracking_opt_dep OpenMP )


# io_ply subcomponent
list(APPEND pcl_io_int_dep io_ply)
set(pcl_io_ply_int_dep common)
set(pcl_io_ply_ext_dep boost)

# VTK components required by PCL
set(PCL_VTK_COMPONENTS "ChartsCore;CommonColor;CommonComputationalGeometry;CommonCore;CommonDataModel;CommonExecutionModel;CommonMath;CommonMisc;CommonTransforms;FiltersCore;FiltersExtraction;FiltersGeneral;FiltersGeometry;FiltersModeling;FiltersSources;ImagingCore;ImagingSources;InteractionImage;InteractionStyle;InteractionWidgets;IOCore;IOGeometry;IOImage;IOLegacy;IOPLY;RenderingAnnotation;RenderingCore;RenderingContext2D;RenderingLOD;RenderingFreeType;ViewsCore;ViewsContext2D;RenderingOpenGL2;RenderingContextOpenGL2;GUISupportQt")

set(pcl_header_only_components 2d cuda_common geometry gpu_tracking modeler in_hand_scanner point_cloud_editor cloud_composer)

include(FindPackageHandleStandardArgs)

#check if user provided a list of components
#if no components at all or full list is given set PCL_FIND_ALL
if(PCL_FIND_COMPONENTS)
  list(LENGTH PCL_FIND_COMPONENTS PCL_FIND_COMPONENTS_LENGTH)
  if(PCL_FIND_COMPONENTS_LENGTH EQUAL PCL_NB_COMPONENTS)
    set(PCL_TO_FIND_COMPONENTS ${pcl_all_components})
    set(PCL_FIND_ALL 1)
  else()
    set(PCL_TO_FIND_COMPONENTS ${PCL_FIND_COMPONENTS})
  endif()
else()
  set(PCL_TO_FIND_COMPONENTS ${pcl_all_components})
  set(PCL_FIND_ALL 1)
endif()

compute_dependencies(PCL_TO_FIND_COMPONENTS)

# We do not need to find components that have been found already, e.g. during previous invocation
# of find_package(PCL). Filter them out.
foreach(component ${PCL_TO_FIND_COMPONENTS})
  string(TOUPPER "${component}" COMPONENT)
  if(NOT PCL_${COMPONENT}_FOUND)
    list(APPEND _PCL_TO_FIND_COMPONENTS ${component})
  endif()
endforeach()
set(PCL_TO_FIND_COMPONENTS ${_PCL_TO_FIND_COMPONENTS})
unset(_PCL_TO_FIND_COMPONENTS)

if(NOT PCL_TO_FIND_COMPONENTS)
  return()
endif()

# compute external dependencies per component
foreach(component ${PCL_TO_FIND_COMPONENTS})
    foreach(opt ${pcl_${component}_opt_dep})
      find_external_library(${component} ${opt} OPTIONAL)
    endforeach()
    foreach(ext ${pcl_${component}_ext_dep})
      find_external_library(${component} ${ext} REQUIRED)
    endforeach()
endforeach()

foreach(component ${PCL_TO_FIND_COMPONENTS})
  set(pcl_component pcl_${component})
  string(TOUPPER "${component}" COMPONENT)

  pcl_message(STATUS "looking for PCL_${COMPONENT}")

  string(REGEX REPLACE "^cuda_(.*)$" "\\1" cuda_component "${component}")
  string(REGEX REPLACE "^gpu_(.*)$" "\\1" gpu_component "${component}")
  string(REGEX REPLACE "^io_(.*)$" "\\1" io_component "${component}")

  find_path(PCL_${COMPONENT}_INCLUDE_DIR
    NAMES pcl/${component}
          pcl/apps/${component}
          pcl/cuda/${cuda_component} pcl/cuda/${component}
          pcl/gpu/${gpu_component} pcl/gpu/${component}
          pcl/io/${io_component}
    HINTS ${PCL_INCLUDE_DIRS}
    PATH_SUFFIXES
          ${component}/include
          apps/${component}/include
          cuda/${cuda_component}/include
          gpu/${gpu_component}/include
          io/${io_component}/include
    DOC "path to ${component} headers"
    NO_DEFAULT_PATH)
  mark_as_advanced(PCL_${COMPONENT}_INCLUDE_DIR)

  if(PCL_${COMPONENT}_INCLUDE_DIR)
    list(APPEND PCL_${COMPONENT}_INCLUDE_DIRS "${PCL_${COMPONENT}_INCLUDE_DIR}")
  else()
    #pcl_message("No include directory found for pcl_${component}.")
  endif()

  set(FPHSA_NAME_MISMATCHED 1) # Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html
  # Skip find_library for header only modules
  list(FIND pcl_header_only_components ${component} _is_header_only)
  if(_is_header_only EQUAL -1)
    find_library(PCL_${COMPONENT}_LIBRARY
      NAMES ${pcl_component}${PCL_RELEASE_SUFFIX} ${pcl_component}${PCL_RELWITHDEBINFO_SUFFIX} ${pcl_component}${PCL_MINSIZEREL_SUFFIX}
      HINTS ${PCL_LIBRARY_DIRS}
      DOC "path to ${pcl_component} library"
      NO_DEFAULT_PATH)
    get_filename_component(${component}_library_path
      ${PCL_${COMPONENT}_LIBRARY}
      PATH)
    mark_as_advanced(PCL_${COMPONENT}_LIBRARY)

    find_library(PCL_${COMPONENT}_LIBRARY_DEBUG ${pcl_component}${PCL_DEBUG_SUFFIX}
      HINTS ${PCL_LIBRARY_DIRS}
      DOC "path to ${pcl_component} library debug"
      NO_DEFAULT_PATH)
    mark_as_advanced(PCL_${COMPONENT}_LIBRARY_DEBUG)

    if(PCL_${COMPONENT}_LIBRARY_DEBUG)
      get_filename_component(${component}_library_path_debug
        ${PCL_${COMPONENT}_LIBRARY_DEBUG}
        PATH)
    endif()

    # Restrict this to Windows users
    if(NOT PCL_${COMPONENT}_LIBRARY AND WIN32)
      # might be debug only
      set(PCL_${COMPONENT}_LIBRARY ${PCL_${COMPONENT}_LIBRARY_DEBUG})
    endif()

    find_package_handle_standard_args(PCL_${COMPONENT} DEFAULT_MSG
      PCL_${COMPONENT}_LIBRARY PCL_${COMPONENT}_INCLUDE_DIR)
  else()
    find_package_handle_standard_args(PCL_${COMPONENT} DEFAULT_MSG
      PCL_${COMPONENT}_INCLUDE_DIR)
  endif()
  unset(FPHSA_NAME_MISMATCHED)

  if(PCL_${COMPONENT}_FOUND)
    if(NOT "${PCL_${COMPONENT}_INCLUDE_DIRS}" STREQUAL "")
      set(_filtered "")
      foreach(_inc ${PCL_${COMPONENT}_INCLUDE_DIRS})
        if(EXISTS ${_inc})
          list(APPEND _filtered "${_inc}")
        endif()
      endforeach()
      list(REMOVE_DUPLICATES _filtered)
      set(PCL_${COMPONENT}_INCLUDE_DIRS ${_filtered})
      list(APPEND PCL_INCLUDE_DIRS ${_filtered})
    endif()
    mark_as_advanced(PCL_${COMPONENT}_INCLUDE_DIRS)
    if(_is_header_only EQUAL -1)
      list(APPEND PCL_DEFINITIONS ${PCL_${COMPONENT}_DEFINITIONS})
      list(APPEND PCL_LIBRARY_DIRS ${component_library_path})
      if(PCL_${COMPONENT}_LIBRARY_DEBUG)
        list(APPEND PCL_LIBRARY_DIRS ${component_library_path_debug})
      endif()
      list(APPEND PCL_COMPONENTS ${pcl_component})
      mark_as_advanced(PCL_${COMPONENT}_LIBRARY PCL_${COMPONENT}_LIBRARY_DEBUG)
    endif()
    # Append internal dependencies
    foreach(int_dep ${pcl_${component}_int_dep})
      string(TOUPPER "${int_dep}" INT_DEP)
      if(PCL_${INT_DEP}_FOUND)
        list(APPEND PCL_${COMPONENT}_INCLUDE_DIRS ${PCL_${INT_DEP}_INCLUDE_DIRS})
        if(PCL_${INT_DEP}_LIBRARIES)
          list(APPEND PCL_${COMPONENT}_LINK_LIBRARIES "${PCL_${INT_DEP}_LIBRARIES}")
        endif()
      endif()
    endforeach()
    if(_is_header_only EQUAL -1)
      add_library(${pcl_component} SHARED IMPORTED)
      if(PCL_${COMPONENT}_LIBRARY_DEBUG)
        set_target_properties(${pcl_component}
          PROPERTIES
            IMPORTED_CONFIGURATIONS "RELEASE;DEBUG"
            IMPORTED_LOCATION_RELEASE "${PCL_${COMPONENT}_LIBRARY}"
            IMPORTED_LOCATION_DEBUG "${PCL_${COMPONENT}_LIBRARY_DEBUG}"
            IMPORTED_IMPLIB_RELEASE "${PCL_${COMPONENT}_LIBRARY}"
            IMPORTED_IMPLIB_DEBUG "${PCL_${COMPONENT}_LIBRARY_DEBUG}"
        )
      else()
        set_target_properties(${pcl_component}
          PROPERTIES
            IMPORTED_LOCATION "${PCL_${COMPONENT}_LIBRARY}"
            IMPORTED_IMPLIB "${PCL_${COMPONENT}_LIBRARY}"
        )
      endif()
    else() # header-only
      add_library(${pcl_component} INTERFACE IMPORTED)
    endif()

    foreach(def ${PCL_DEFINITIONS})
      string(REPLACE " " ";" def2 ${def})
      string(REGEX REPLACE "^-D" "" def3 "${def2}")
      list(APPEND definitions ${def3})
    endforeach()
    if(CMAKE_VERSION VERSION_LESS 3.11)
      set_target_properties(${pcl_component}
        PROPERTIES
          INTERFACE_COMPILE_DEFINITIONS "${definitions}"
          INTERFACE_COMPILE_OPTIONS "$<$<COMPILE_LANGUAGE:CXX>:${PCL_COMPILE_OPTIONS}>"
          INTERFACE_COMPILE_FEATURES "cxx_std_17"
          INTERFACE_INCLUDE_DIRECTORIES "${PCL_${COMPONENT}_INCLUDE_DIRS};${PCL_CONF_INCLUDE_DIR}"
          INTERFACE_LINK_LIBRARIES "${PCL_${COMPONENT}_LINK_LIBRARIES}"
      )
    else()
      set_target_properties(${pcl_component}
        PROPERTIES
          INTERFACE_COMPILE_DEFINITIONS "${definitions}"
          INTERFACE_COMPILE_OPTIONS "$<$<COMPILE_LANGUAGE:CXX>:${PCL_COMPILE_OPTIONS}>"
          INTERFACE_COMPILE_FEATURES "cxx_std_17"
          INTERFACE_INCLUDE_DIRECTORIES "${PCL_${COMPONENT}_INCLUDE_DIRS};${PCL_CONF_INCLUDE_DIR}"
      )
      # If possible, we use target_link_libraries to avoid problems with link-type keywords,
      # see https://github.com/PointCloudLibrary/pcl/issues/2989
      # target_link_libraries on imported libraries is supported only since CMake 3.11
      target_link_libraries(${pcl_component} INTERFACE ${PCL_${COMPONENT}_LINK_LIBRARIES})
    endif()
    set(PCL_${COMPONENT}_LIBRARIES ${pcl_component})
  endif()
endforeach()

if(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")
  list(REMOVE_DUPLICATES PCL_INCLUDE_DIRS)
endif()

if(NOT "${PCL_LIBRARY_DIRS}" STREQUAL "")
  list(REMOVE_DUPLICATES PCL_LIBRARY_DIRS)
endif()

if(NOT "${PCL_DEFINITIONS}" STREQUAL "")
  list(REMOVE_DUPLICATES PCL_DEFINITIONS)
endif()

pcl_remove_duplicate_libraries(PCL_COMPONENTS PCL_LIBRARIES)

# Add 3rd party libraries, as user code might include our .HPP implementations
list(APPEND PCL_LIBRARIES ${PCL_BOOST_LIBRARIES} ${PCL_OPENNI_LIBRARIES} ${PCL_OPENNI2_LIBRARIES} ${PCL_ENSENSO_LIBRARIES} ${PCL_davidSDK_LIBRARIES} ${PCL_DSSDK_LIBRARIES} ${PCL_RSSDK_LIBRARIES} ${PCL_RSSDK2_LIBRARIES} ${PCL_VTK_LIBRARIES})
if (TARGET FLANN::FLANN)
  list(APPEND PCL_LIBRARIES FLANN::FLANN)
endif()

if(TARGET QHULL::QHULL)
    list(APPEND PCL_LIBRARIES QHULL::QHULL)
endif()

find_package_handle_standard_args(PCL DEFAULT_MSG PCL_LIBRARIES PCL_INCLUDE_DIRS)
mark_as_advanced(PCL_LIBRARIES PCL_INCLUDE_DIRS PCL_LIBRARY_DIRS)
