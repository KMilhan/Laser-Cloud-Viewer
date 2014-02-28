#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>


#include <omp.h>



#include <boost/thread/thread.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_representation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/common/common_headers.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/console/parse.h>



#include <pcl/registration/icp.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/surfel_smoothing.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/file_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
    
     /* Define some custom types to make the rest of our code easier to read */
    
    // Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
    
    // Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
    typedef pcl::Normal NormalT;
    typedef pcl::PointCloud<NormalT> SurfaceNormals;
    typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
    typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;
    
    // Define "LocalDescriptors" to be a pcl::PointCloud of pcl::FPFHSignature33 points
    typedef pcl::FPFHSignature33 LocalDescriptorT;
    typedef pcl::PointCloud<LocalDescriptorT> LocalDescriptors;
    typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
    typedef pcl::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;
    
   // Define "GlobalDescriptors" to be a pcl::PointCloud of pcl::VFHSignature308 points
   typedef pcl::VFHSignature308 GlobalDescriptorT;
    typedef pcl::PointCloud<GlobalDescriptorT> GlobalDescriptors;
   typedef pcl::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
   typedef pcl::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;
   

