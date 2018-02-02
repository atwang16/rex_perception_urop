/************************************************************/
/*    NAME: Austin Wang                                     */
/*    ORGN: MIT                                             */
/*    FILE: ClusterPointCloud.h                             */
/*    DATE: 01-22-2018                                      */
/************************************************************/

#ifndef ClusterPointCloud_HEADER
#define ClusterPointCloud_HEADER

#include <vector>
#include <array>
#include "PacketBundleDecoder.h"
#include "MOOS/libMOOS/MOOSLib.h"
#include <pcl/common/projection_matrix.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

typedef pcl::PointXYZ PointType;

class ClusterPointCloud : public CMOOSApp {
 public:
  ClusterPointCloud();

  ~ClusterPointCloud();

 protected: // Standard MOOSApp functions to overload  
  bool OnNewMail(MOOSMSG_LIST &NewMail);

  bool Iterate();

  bool OnConnectToServer();

  bool OnStartUp();

 protected:
  pcl::PointCloud<PointType>::Ptr format_input();
  bool generate_clusters();
  std::vector<std::string> format_output();

 protected:  // Configuration variables
  std::string                                    m_corrections_file;
  bool                                           m_intensity;
  bool                                           m_laser_id;
  bool                                           m_azimuth;
  bool                                           m_distance;
  bool                                           m_ms_from_top_of_hour;

 private: // State variables
  void RegisterVariables();
  std::vector<double>                            m_frame;
  std::string*                                   m_latest_bundle;
  unsigned int*                                  m_latest_bundle_length;
  PacketBundleDecoder                            m_bundle_decoder;
  PacketBundleDecoder::HDLFrame                  m_latest_frame_b;
  pcl::PointCloud<PointType>::Ptr                m_input_cloud;
  std::vector<std::string>                       m_output_points;
  std::vector< std::array<double, 2> >           m_centroids;
  std::vector< pcl::PointCloud<PointType>::Ptr > m_convex_hulls;
  bool                                           m_corrections_given;
  PacketDecoder                                  m_decoder;

 protected: // Constants
  static const float REX_PLANE_THRESH_X = 3.0;
  static const float REX_PLANE_THRESH_Y = 3.0;
  static const float REX_LIDAR_HEIGHT = 5.0;
  static const float REX_VEHICLE_HEIGHT = 8.0;
  static const float VOXEL_GRID_SIZE = 0.1;
  static const float OUTLIER_RADIUS = 2.0;
  static const int   OUTLIER_MIN_NEIGHBORS = 4;
  static const float CLUSTER_TOLERANCE = 4.0;
  static const int   CLUSTER_MIN_SIZE = 5;
  static const int   CLUSTER_MAX_SIZE = 25000;


 private: // Configuration variables

 private: // State variables
  int m_iterations;
};

#endif 
