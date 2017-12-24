#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

using namespace std;
using namespace pcl;

// Point Type
// PointXYZ, PointXYZI, PointXYZRGBA
typedef PointXYZRGBA PointType;

int main(int argc, char *argv[]) {
	// Command-Line Argument Parsing
    if( console::find_switch( argc, argv, "-help" ) ){
        cout << "usage: " << argv[0]
            << " [-calibrationFile]"
            << " [-pcap <*.pcap>]"
            << " [-help]"
            << endl;
        return 0;
    }
    string pcap;
    // parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
    console::parse_argument( argc, argv, "-pcap", pcap );

    // Point Cloud
    PointCloud<PointType>::ConstPtr cloud (new PointCloud<PointType>);
    vector<PointCloud<PointType>::Ptr> cluster_polygon;
    vector<vector<double> > cluster_colors;

    // PCL Visualizer
    boost::shared_ptr<visualization::PCLVisualizer> viewer( new visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const PointCloud<PointType>::ConstPtr& )> function =
        [ &cloud, &cluster_polygon, &mutex ]( const PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex ); // for multi-threading

            /* Point Cloud Processing */
            cluster_polygon.clear();
            PointCloud<PointType>::Ptr cloud_filtered (new PointCloud<PointType>);
            cloud = ptr;

            // Filter out all points within threshold box from vehicle (origin)
            double thresh = 3.0;
            ConditionOr<PointType>::Ptr range_cond (new
              ConditionOr<PointType> ());
            range_cond->addComparison (FieldComparison<PointType>::ConstPtr (new
              FieldComparison<PointType> ("x", ComparisonOps::GT, thresh)));
            range_cond->addComparison (FieldComparison<PointType>::ConstPtr (new
              FieldComparison<PointType> ("x", ComparisonOps::LT, -thresh)));
            range_cond->addComparison (FieldComparison<PointType>::ConstPtr (new
              FieldComparison<PointType> ("y", ComparisonOps::GT, thresh)));
            range_cond->addComparison (FieldComparison<PointType>::ConstPtr (new
              FieldComparison<PointType> ("y", ComparisonOps::LT, -thresh)));

            ConditionalRemoval<PointType> condrem;
            condrem.setCondition(range_cond);
            condrem.setInputCloud(cloud);
            condrem.filter(*cloud_filtered);
            cloud = cloud_filtered;

            // Filter out points that are higher than vehicle
            double height_above_water = 4.0;
            double height_above_lidar = 3.0;
            PassThrough<PointType> pass;
            pass.setInputCloud (cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits(-height_above_water, height_above_lidar);
            pass.filter(*cloud_filtered);
            cloud = cloud_filtered;

            // Project all points down to XY plane
            ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
            coefficients->values.resize (4);
            coefficients->values[0] = coefficients->values[1] = 0;
            coefficients->values[2] = 1.0;
            coefficients->values[3] = 0;

            ProjectInliers<PointType> proj;
            proj.setModelType (SACMODEL_PLANE);
            proj.setInputCloud (cloud);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_filtered);
            cloud = cloud_filtered;

            // Downsample the dataset using voxel grids
            float leaf_size = 0.1f;
            VoxelGrid<PointType> vg;
            vg.setInputCloud (cloud);
            vg.setLeafSize (leaf_size, leaf_size, leaf_size);
            vg.filter (*cloud_filtered);
            cloud = cloud_filtered;

            // Filter out all points which don't have many neighbors within a particular radius
            float radius = 2.0;
            int min_neighbors = 4;
            RadiusOutlierRemoval<PointType> outrem;
            outrem.setInputCloud(cloud);
            outrem.setRadiusSearch(radius);
            outrem.setMinNeighborsInRadius(min_neighbors);
            outrem.filter (*cloud_filtered);
            cloud = cloud_filtered;

            // Creating the KdTree object for the search method of the extraction
            search::KdTree<PointType>::Ptr tree (new search::KdTree<PointType>);
            tree->setInputCloud (cloud_filtered);

            float cluster_tolerance = 4.0f;
            int min_cluster_size = 10;
            int max_cluster_size = 25000;
            vector<PointIndices> cluster_indices;
            EuclideanClusterExtraction<PointType> ec;
            ec.setClusterTolerance (cluster_tolerance);
            ec.setMinClusterSize (min_cluster_size);
            ec.setMaxClusterSize (max_cluster_size);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            // iterate through all points in each cluster
            for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
                // set colors and compute centroids
                PointCloud<PointType>::Ptr cluster_points (new PointCloud<PointType>);
                for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
                    cluster_points->push_back(cloud_filtered->points[*pit]);
                }
                PointCloud<PointType>::Ptr cloud_hull (new PointCloud<PointType>);
                ConvexHull<PointType> chull;
                chull.setInputCloud (cluster_points);
                chull.reconstruct (*cloud_hull);

                cluster_polygon.push_back(cloud_hull);
            }
            cloud = cloud_filtered;
        };

    // VLP Grabber
    boost::shared_ptr<HDLGrabber> grabber;
    if( !pcap.empty() ){
        cout << "Capture from PCAP..." << endl;
        grabber = boost::shared_ptr<HDLGrabber>( new HDLGrabber( "", pcap ) );
    }

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

	while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );

        if(lock.owns_lock()) {
            // visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> rgb(cloud);
            // if( !viewer->updatePointCloud( cloud, rgb, "cloud" ) ){
            //     viewer->addPointCloud( cloud, rgb, "cloud" );
            // }

            for (int i = 0; i < cluster_polygon.size(); i++) {
                PointCloud<PointType>::Ptr chull = cluster_polygon[i];
                double r = 0, g = 0, b = 0;
                if(i < cluster_colors.size()) { // color already chosen for cluster
                    r = cluster_colors[i][0];
                    g = cluster_colors[i][1];
                    b = cluster_colors[i][2];
                }
                else { // generate a random color
                    r = (double)rand() / RAND_MAX; // normalize between 0.0 and 1.0
                    g = (double)rand() / RAND_MAX;
                    b = (double)rand() / RAND_MAX;
                    vector<double> rgb = {r, g, b};
                    cluster_colors.push_back(rgb);
                }
                viewer->addPolygon<PointType> (chull, r, g, b, "polygon", 0);
                // pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(chull, (uint32_t)(r * 255), (uint32_t)(g * 255), (uint32_t)(b * 255));
                // viewer->addPointCloud<PointType> (chull, single_color, "cloud");
            }
        }
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }

    return 0;
}