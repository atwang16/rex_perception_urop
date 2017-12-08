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
    vector<PointCloud<PointType>::Ptr> cloud_clusters;
    vector<vector<float> > cluster_centers;
    vector<uint32_t> cluster_colors;

    // PCL Visualizer
    boost::shared_ptr<visualization::PCLVisualizer> viewer( new visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const PointCloud<PointType>::ConstPtr& )> function =
        [ &cloud, &cluster_colors, &cluster_centers, &mutex ]( const PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );

            /* Point Cloud Processing */
            cluster_centers.clear();
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
                vector<float> center{0.0, 0.0};
                int size_of_cluster = 0;
                uint32_t rgb = 0;
                if(cluster_colors.size() > it - cluster_indices.begin()) { // color already chosen for cluster
                    rgb = cluster_colors[it - cluster_indices.begin()];
                }
                else { // generate a random color
                    uint8_t r = (uint8_t)((float)(rand()) / RAND_MAX * 255),
                            g = (uint8_t)((float)(rand()) / RAND_MAX * 255),
                            b = (uint8_t)((float)(rand()) / RAND_MAX * 255);
                    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                    cluster_colors.push_back(rgb);
                }

                // set colors and compute centroids
                for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
                    cloud_filtered->points[*pit].rgba = rgb;
                    center[0] += cloud_filtered->points[*pit].x;
                    center[1] += cloud_filtered->points[*pit].y;
                    size_of_cluster++;
                }
                center[0] /= size_of_cluster;
                center[1] /= size_of_cluster;
                cluster_centers.push_back(center);
                cout << center[0] << ", " << center[1] << endl;
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
            visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> rgb(cloud);
            if( !viewer->updatePointCloud( cloud, rgb, "cloud" ) ){
                viewer->addPointCloud( cloud, rgb, "cloud" );
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