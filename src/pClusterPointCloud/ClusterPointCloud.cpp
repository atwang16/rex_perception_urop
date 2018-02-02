/************************************************************/
/*    NAME: Austin Wang                                     */
/*    ORGN: MIT                                             */
/*    FILE: ClusterPointCloud.cpp                           */
/*    DATE: 2018                                            */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ClusterPointCloud.h"

using namespace std;
using namespace pcl;

//template <typename T>
//std::vector<T> operator+(const std::vector<T> &A, const std::vector<T> &B)
//{
//  std::vector<T> AB;
//  AB.reserve( A.size() + B.size() );                // preallocate memory
//  AB.insert( AB.end(), A.begin(), A.end() );        // add A;
//  AB.insert( AB.end(), B.begin(), B.end() );        // add B;
//  return AB;
//}
//
//template <typename T>
//std::vector<T> &operator+=(std::vector<T> &A, const std::vector<T> &B)
//{
//  A.reserve( A.size() + B.size() );                // preallocate memory without erase original data
//  A.insert( A.end(), B.begin(), B.end() );         // add B;
//  return A;                                        // here A could be named AB
//}
//
//template <typename T, typename T2>
//std::vector<T> &operator+=(std::vector<T> &A, const std::vector<T2> &B)
//{
//  A.reserve( A.size() + B.size() );                // preallocate memory without erase original data
//  A.insert( A.end(), B.begin(), B.end() );         // add B;
//  return A;                                        // here A could be named AB
//}

//---------------------------------------------------------
// Constructor

ClusterPointCloud::ClusterPointCloud()
{
  m_iterations = 0;
  m_latest_bundle = new std::string();
  m_latest_bundle_length = new unsigned int();
}

//---------------------------------------------------------
// Destructor

ClusterPointCloud::~ClusterPointCloud()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool ClusterPointCloud::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    if(p->IsName("VELODYNE_PACKET_BUNDLE")) {
      m_latest_bundle = new string((char*)p->GetBinaryData(), p->GetBinaryDataSize());
      *m_latest_bundle_length = p->GetBinaryDataSize();

      m_bundle_decoder.DecodeBundle(m_latest_bundle, m_latest_bundle_length);
      if (m_bundle_decoder.GetLatestFrame(&m_latest_frame_b)) {
        m_input_cloud = format_input();
        generate_clusters();
        m_output_points = format_output();
        for(int i = 0; i < m_output_points.size(); i++)
        {
          Notify("VELODYNE_CONVEX_HULLS", m_output_points[i]);
        }

//        m_frame = m_latest_frame_b.x + m_latest_frame_b.y + m_latest_frame_b.z;
//        if (m_intensity) m_frame += m_latest_frame_b.intensity;
//        if (m_laser_id) m_frame += m_latest_frame_b.laser_id;
//        if (m_azimuth) m_frame += m_latest_frame_b.azimuth;
//        if (m_distance) m_frame += m_latest_frame_b.distance;
//        if (m_ms_from_top_of_hour) m_frame += m_latest_frame_b.ms_from_top_of_hour;
//        m_frame.clear();
      }

      delete m_latest_bundle;
    }

#if 0 // Keep these around just for template
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ClusterPointCloud::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
   // m_MissionReader.GetConfigurationParam("Name", <string>);
   // m_Comms.Register("VARNAME", 0);
	
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ClusterPointCloud::Iterate()
{
  m_iterations++;
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ClusterPointCloud::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "FOO") {
        //handled
      }
      else if(param == "BAR") {
        //handled
      }
    }
  }

  SetIterateMode(REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL);  //must use aync (V10) comms, otherwise we can't read packets quickly enough

  if (!m_MissionReader.GetConfigurationParam("CORRECTIONS_FILE", m_corrections_file)) {
    cerr << "CORRECTIONS_FILE not specified! Assuming none..." << endl;
    m_corrections_given = false;
  } else {
    m_corrections_given = true;
  }

  if (m_corrections_given) {
    m_decoder.SetCorrectionsFile(m_corrections_file);
    m_bundle_decoder.SetCorrectionsFile(m_corrections_file);
  }

  if (!m_MissionReader.GetConfigurationParam("INTENSITY", m_intensity)) {
    cerr << "INTENSITY not specified! Assuming False..." << endl;
    m_intensity = false;
  }

  if (!m_MissionReader.GetConfigurationParam("LASER_ID", m_laser_id)) {
    cerr << "LASER_ID not specified! Assuming False..." << endl;
    m_laser_id = false;
  }

  if (!m_MissionReader.GetConfigurationParam("AZIMUTH", m_azimuth)) {
    cerr << "AZIMUTH not specified! Assuming False..." << endl;
    m_azimuth = false;
  }

  if (!m_MissionReader.GetConfigurationParam("DISTANCE", m_distance)) {
    cerr << "DISTANCE not specified! Assuming False..." << endl;
    m_distance = false;
  }

  if (!m_MissionReader.GetConfigurationParam("MS_FROM_TOP_OF_HOUR", m_ms_from_top_of_hour)) {
    cerr << "MS_FROM_TOP_OF_HOUR not specified! Assuming False..." << endl;
    m_ms_from_top_of_hour = false;
  }

  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void ClusterPointCloud::RegisterVariables()
{
  Register("VELODYNE_PACKET_BUNDLE", 0);
}

//---------------------------------------------------------
// Procedure: format_input

pcl::PointCloud<PointType>::Ptr ClusterPointCloud::format_input() {
  PointCloud<PointType>::Ptr cloud (new PointCloud<PointType>);
  for(int i = 0; i < m_latest_frame_b.x.size(); ++i)
  {
    PointType pt;
    pt.x = (float) m_latest_frame_b.x[i];
    pt.y = (float) m_latest_frame_b.y[i];
    pt.z = (float) m_latest_frame_b.z[i];
    cloud->push_back(pt);
  }
  return cloud;
}

//---------------------------------------------------------
// Procedure: generate_clusters

bool ClusterPointCloud::generate_clusters() {
  m_convex_hulls.clear();
  m_centroids.clear();
  PointCloud<PointType>::Ptr cloud_filtered (new PointCloud<PointType>);

  // Filter out all points within threshold box from vehicle (origin)
  ConditionOr<PointType>::Ptr range_cond (new
                                                  ConditionOr<PointType> ());
  range_cond->addComparison (FieldComparison<PointType>::ConstPtr
                                     (new FieldComparison<PointType> ("x", ComparisonOps::GT, REX_PLANE_THRESH_X)));
  range_cond->addComparison (FieldComparison<PointType>::ConstPtr
                                     (new FieldComparison<PointType> ("x", ComparisonOps::LT, -REX_PLANE_THRESH_X)));
  range_cond->addComparison (FieldComparison<PointType>::ConstPtr
                                     (new FieldComparison<PointType> ("y", ComparisonOps::GT, REX_PLANE_THRESH_Y)));
  range_cond->addComparison (FieldComparison<PointType>::ConstPtr
                                     (new FieldComparison<PointType> ("y", ComparisonOps::LT, -REX_PLANE_THRESH_Y)));

  ConditionalRemoval<PointType> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(m_input_cloud);
  condrem.filter(*cloud_filtered);

  // Filter out points that are higher than vehicle
  PassThrough<PointType> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(-REX_LIDAR_HEIGHT, REX_VEHICLE_HEIGHT - REX_LIDAR_HEIGHT);
  pass.filter(*cloud_filtered);

  // Project all points down to XY plane
  ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  ProjectInliers<PointType> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_filtered);

  // Downsample the dataset using voxel grids
  VoxelGrid<PointType> vg;
  vg.setInputCloud (cloud_filtered);
  vg.setLeafSize (VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE);
  vg.filter (*cloud_filtered);

  // Filter out all points which don't have many neighbors within a particular radius
  RadiusOutlierRemoval<PointType> outrem;
  outrem.setInputCloud(cloud_filtered);
  outrem.setRadiusSearch(OUTLIER_RADIUS);
  outrem.setMinNeighborsInRadius(OUTLIER_MIN_NEIGHBORS);
  outrem.filter (*cloud_filtered);

  // Creating the KdTree object for the search method of the extraction
  search::KdTree<PointType>::Ptr tree (new search::KdTree<PointType>);
  tree->setInputCloud (cloud_filtered);

  vector<PointIndices> cluster_indices;
  EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance (CLUSTER_TOLERANCE);
  ec.setMinClusterSize (CLUSTER_MIN_SIZE);
  ec.setMaxClusterSize (CLUSTER_MAX_SIZE);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // iterate through all points in each cluster
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
    // set colors and compute centroids
    PointCloud<PointType>::Ptr cluster_points (new PointCloud<PointType>);
    array<double, 2> centroid = {0, 0};
    for (int index : it->indices) {
      cluster_points->push_back(cloud_filtered->points[index]);
      centroid[0] += cloud_filtered->points[index].x;
      centroid[1] += cloud_filtered->points[index].y;
    }
    if(cluster_points->width * cluster_points->height > 0) {
      centroid[0] /= cluster_points->width * cluster_points->height;
      centroid[1] /= cluster_points->width * cluster_points->height;

      PointCloud<PointType>::Ptr cloud_hull (new PointCloud<PointType>);
      ConvexHull<PointType> chull;
      chull.setInputCloud (cluster_points);
      chull.reconstruct (*cloud_hull);
      m_convex_hulls.push_back(cloud_hull);
      m_centroids.push_back(centroid);
    }
  }
  return(true);
}

vector<string> ClusterPointCloud::format_output() {
  vector<string> output;
  for(const auto pc : m_convex_hulls)
  {
    string pc_out = "{";
    for(auto it = pc->begin(); it != pc->end(); it++)
    {
      pc_out += to_string(it->x) + "," + to_string(it->y);
      if(it != pc->end())
      {
        pc_out += ":";
      }
    }
    pc_out += "}";
    output.push_back(pc_out);
  }
  return(output);
}