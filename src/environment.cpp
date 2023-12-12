/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
//#include "quiz/ransac/ransac2d.cpp"  // circular-dependency-issues
#include <cmath>
#include <unordered_set>
#include <Eigen/Dense>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_clusters = true;
    bool render_box = true;
    bool render_obst_cloud = false;
    bool render_road_cloud = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar = lidar->scan();

    // RenderRays to render points and shapes
    //renderRays(viewer, lidar->position, cloud_lidar);

    // Render point cloud to view the point cloud data
    //std::string first_cloud;
    //Color cloud_color(1, 1, 1);
    //renderPointCloud(viewer, cloud_lidar, "cloud_lidar");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* cloud_processor = new ProcessPointClouds<pcl::PointXYZ>();
    //ProcessPointClouds<pcl::PointXYZI>* cloud_processor_intensity = new ProcessPointClouds<pcl::PointXYZI>();

    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_cloud = cloud_processor->SegmentPlane(cloud_lidar, 100, 0.2);

    if (render_obst_cloud) {
        renderPointCloud(viewer, segmented_cloud.first, "obstacle_cloud", Color(1, 0, 0));
    }

    if (render_road_cloud) {
        renderPointCloud(viewer, segmented_cloud.second, "road_cloud", Color(0, 1, 0));
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = cloud_processor->Clustering(segmented_cloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (render_clusters) {
            std::cout << "cluster size ";
            cloud_processor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstacle_cloud" + std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box) {
            Box box = cloud_processor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
    renderPointCloud(viewer, segmented_cloud.second, "road_cloud");
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* city_processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& city_cloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display real pcds -----
    // ----------------------------------------------
    // RENDER OPTIONS
    bool render_full_cloud = false;
    bool render_filtered_cloud = false;
    bool render_obst_cloud = true;
    bool render_plane_cloud = true;
    bool render_clusters = true;
    bool render_boxes = true;

    //ProcessPointClouds<pcl::PointXYZI>* city_processor = new (ProcessPointClouds<pcl::PointXYZI>);

    // Path set relative to environment.exe in out/build/x64-Debug
    //pcl::PointCloud<pcl::PointXYZI>::Ptr city_cloud = city_processor->loadPcd("../../../src/sensors/data/pcd/data_1/0000000001.pcd");

    if (render_full_cloud) {
        renderPointCloud(viewer, city_cloud, "ip_cloud_cityblock");
    }

    // Filtering the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Use of 4D vectors because its easier to multiply them with a 4x4 projection matrix
    // Look ahead (+x): 10; back (-x): 10; left (+y): 8; right (-y): 8; up (+z): 5; down (-z): 7; 
    filtered_cloud = city_processor->FilterCloud(city_cloud, 0.3, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(30, 7, 2, 1));
    
    if (render_full_cloud) {
        renderPointCloud(viewer, filtered_cloud, "filtered_city_cloud");
    }

    // Segmenting the filtered cloud
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> seg_cloud = city_processor->SegmentPlane(filtered_cloud, 25, 0.3);

    std::unordered_set<int> inliers = city_processor->RansacPlane(filtered_cloud, 25, 0.18);

    pcl::PointIndices::Ptr inliers_pts(new pcl::PointIndices());

    for (int index : inliers) {
        inliers_pts->indices.push_back(index);
    }

    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = city_processor->SeparateClouds(inliers_pts, filtered_cloud);

    if (render_obst_cloud) {
        renderPointCloud(viewer, segmented_cloud.first, "obstacle_cloud_city", Color(1, 0, 0));
    }

    if (render_plane_cloud) {
        renderPointCloud(viewer, segmented_cloud.second, "plane_cloud_city", Color(0, 1, 0));
    }

    // Clustering the segmented and road clouds
    // 
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> city_clusters = city_processor->Clustering(seg_cloud.first, 0.6, 30, 600);

    KdTree* tree = new KdTree;

    for (int i = 0; i < segmented_cloud.first->size(); i++)
        tree->insert(segmented_cloud.first->points[i], i);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> city_clusters = city_processor->euclideanCluster(segmented_cloud.first, tree, 0.45);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : city_clusters)
    {
        if (render_clusters) {
            //std::cout << "cluster size ";
            //city_processor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstacle_cloud_city" + std::to_string(clusterId), colors[clusterId%3]);
        }
        if (render_boxes) {
            Box box = city_processor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    //cityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI>* city_pt_processor = new (ProcessPointClouds<pcl::PointXYZI>);
    // List of pcd files to operate on
    std::vector<boost::filesystem::path> stream = city_pt_processor->streamPcd("../../../src/sensors/data/pcd/data_1");
    // File iterator
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr city_cloud_I;


    while (!viewer->wasStopped ())
    {
        // Clear everything in the viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection on it
        city_cloud_I = city_pt_processor->loadPcd((*streamIterator).string());
        cityBlock(viewer, city_pt_processor, city_cloud_I);

        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    } 
}
