/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include <cstdint>
#include <unordered_set>


// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	typename pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor(1, 1, 1);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 0, 0, 0, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i].x;
  		point.y = points[i].y;
  		point.z = points[i].z;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, typename pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint32_t depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point.x, window.y_min, 0),pcl::PointXYZ(node->point.x, window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point.x;
			upperWindow.x_min = node->point.x;
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point.y, 0),pcl::PointXYZ(window.x_max, node->point.y, 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point.y;
			upperWindow.y_min = node->point.y;
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

//// Check if point is processed or not using its count in the set (0: unprocessed)
//bool isPointProcessed(const std::unordered_set<int>& processedIndices, int index) {
//    return processedIndices.count(index) > 0;
//}
//
//// Mark the point at index as processed
//void processPoint(std::unordered_set<int>& processedIndices, int index) {
//    processedIndices.insert(index);
//}
//
//// Proximity recursion to get the nearvy points for the current cluster
//void clusterHelper(int index, KdTree* tree, const std::vector<std::vector<float>> points, std::unordered_set<int>& processedIndices, std::vector<int>& cluster, float distanceTol) {
//    if (!isPointProcessed(processedIndices, index)) {
//        processPoint(processedIndices, index);
//        cluster.push_back(index);
//
//        std::vector<int> nearby_points = tree->search(points[index], distanceTol);
//
//        for (int id: nearby_points) {
//            if (!isPointProcessed(processedIndices, id)) {
//                clusterHelper(id, tree, points, processedIndices, cluster, distanceTol);
//            }
//        }
//    }
//}
//
//std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
//{
//
//	// TODO: Fill out this function to return list of indices for each cluster
//	std::vector<std::vector<int>> clusters;
//
//    std::unordered_set<int> processedIds;
//
//    for (int index = 0; index < points.size(); index++) {
//        if (!isPointProcessed(processedIds, index)) {
//            std::vector<int> current_cluster;
//            clusterHelper(index, tree, points, processedIds, current_cluster, distanceTol);
//            clusters.push_back(current_cluster);
//        }
//    }
// 
//	return clusters;
//}


// Check if point is processed or not using its count in the set (0: unprocessed)
bool isPointProcessed(const pcl::PointIndices& processedIndices, int index) {
    return std::count(processedIndices.indices.begin(), processedIndices.indices.end(), index) > 0;
}

// Mark the point at index as processed
void processPoint(pcl::PointIndices& processedIndices, int index) {
    processedIndices.indices.push_back(index);
}

// Proximity recursion to get the nearvy points for the current cluster
void clusterHelper(int index, KdTree* tree, typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices& processedIndices, typename pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster, float distanceTol) {
    if (!isPointProcessed(processedIndices, index)) {
        processPoint(processedIndices, index);
        cluster->points.push_back(cloud->points[index]);
        //cluster.push_back(index);

        std::vector<int> nearby_points = tree->search(cloud->points[index], distanceTol);

        for (int id : nearby_points) {
            if (!isPointProcessed(processedIndices, id)) {
                clusterHelper(id, tree, cloud, processedIndices, cluster, distanceTol);
            }
        }
    }
}

std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, KdTree* tree, float distanceTol) {

    // TODO: Fill out this function to return list of indices for each cluster
    // Time manual Clustering implementation
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    pcl::PointIndices processedIds;

    for (int index = 0; index < cloud->points.size(); index++) {
        if (!isPointProcessed(processedIds, index)) {
            typename pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            clusterHelper(index, tree, cloud, processedIds, current_cluster, distanceTol);
            //current_cluster->points.push_back(cloud->points[index]);
            current_cluster->width = current_cluster->size();
            current_cluster->height = 1;
            current_cluster->is_dense = true;

            clusters.push_back(current_cluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Euclidean Clustering Implementation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return clusters;
}


int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min = -10;
  	window.z_max =  10;
    typename pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	std::vector<std::vector<float>> points = { {-6.2,7,1}, {-6.3,8.4,2.1}, {-5.2,7.1,1.8}, {-5.7,6.3,4}, {-4.2,6.5,3.3}, {-6.9,8.1,5.8}, {-5.1,6,1.7}, {-8.2,6.5,2.9} };

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    for (int i = 0; i < points.size(); i++) {
        tree->insert(points[i], i);
    } 

  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);
  
  	std::cout << "Test Search" << std::endl;
  	std::vector<int> nearby = tree->search({ -8.1,6.5,2.9 },3.0);
    for (int index : nearby) {
        std::cout << index << ",";
    }
  	std::cout << std::endl;

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = euclideanCluster(points, tree, 1.5);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
        //std::cout << "Cluster no: " << clusterId << "\n" << std::endl;
        for (int indice : cluster) {
            //std::cout << " and point: " << " (" << points[indice].x << ", " << points[indice].y << ", " << points[indice].z << ")" << "\n" << std::endl;
            clusterCloud->points.push_back(pcl::PointXYZ(points[indice].x, points[indice].y, points[indice].z));
        }
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
