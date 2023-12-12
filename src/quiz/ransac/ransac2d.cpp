/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud = pointProcessor.loadPcd("../../../../sensors/data/pcd/data_1/0000000000.pcd");

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    filtered_cloud = pointProcessor.FilterCloud(cloud, 0.3, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(30, 7, 2, 1));

    return filtered_cloud;
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    std::cerr << "PointCloud size: " << cloud->width * cloud->height << " data points. \n" << std::endl;

    /*std::cerr << "\n Cloud Points: \n" << std::endl;
    for (auto pt = cloud->begin(); pt != cloud->end(); ++pt) {
        const auto& point = *pt;
        std::cerr << "    " << point.x << " "
                            << point.y << " "
                            << point.z << std::endl;
    }*/

	// For max iterations
    int max_inliers = 0;
    std::vector<int> best_pts;

    while (maxIterations--) {
	    // Randomly sample subset and fit line
        std::unordered_set<int> current_inlier_ids;
        std::vector<int> rand_pts;

        for (int i = 1; i < 3; ++i) {
            rand_pts.push_back(rand() % (cloud->points.size()));
            //std::cout << "\nRandom idx and point: " << (rand() % (cloud->points.size())) << std::endl;  // 2nd call?
            // Add a short delay to ensure a different seed - Risky replace when alternative found
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        float x1 = cloud->points[rand_pts[0]].x;
        float y1 = cloud->points[rand_pts[0]].y;
        float x2 = cloud->points[rand_pts[1]].x;
        float y2 = cloud->points[rand_pts[1]].y;

        //std::cout << "\n Selected random cloud points: " << " (" << x1 << ", " << y1 << ")" << " and " << "(" << x2 << ", " << y2 << ")" << std::endl;

        float a = (y1 - y2);
        float b = (x2 - x1);
        float c = (x1*y2 - x2*y1);

        float distance;

	    // Measure distance between every point and fitted line
        for (int index = 0; index < cloud->points.size(); index++) {

            pcl::PointXYZ point = cloud->points[index];
            distance = 0.0;

            distance = std::abs((a * point.x) + (b * point.y) + c) / std::hypot(a, b);  // hypot : sqrt(A^2 + B^2)

            //std::cout << "\n Distance between point: " << " (" << point.x << ", " << point.y << ")" << " and line: " << a << "x + " << b << "y + " << c << " = 0 is " << distance << std::endl;

            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol) {
                current_inlier_ids.insert(index);
            }
        }

        // Return indicies of inliers from fitted line with most inliers
        if (max_inliers < static_cast<int>(current_inlier_ids.size())) {
            max_inliers = static_cast<int>(current_inlier_ids.size());
            inliersResult = current_inlier_ids;
            best_pts = rand_pts;
        }
    }

    std::cerr << "Best Plane Model Points: " << " (" << cloud->points[best_pts[0]].x << ", " << cloud->points[best_pts[0]].y << ")" << " and "
                                       << "(" << cloud->points[best_pts[1]].x << ", " << cloud->points[best_pts[1]].y << ")" << " with Max inliers: " << "(" << max_inliers << ")" << std::endl;

	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    //std::cerr << "3D PointCloud size: " << cloud->width * cloud->height << " data points. \n" << std::endl;

    // For max iterations
    int max_inliers = 0;
    std::vector<int> best_pts;

    while (maxIterations--) {
        // Randomly sample subset and fit plane
        std::unordered_set<int> current_inlier_ids;  // pcl::PointIndices??
        std::vector<int> rand_pts;

        for (int i = 1; i < 4; ++i) {
            rand_pts.push_back(rand() % (cloud->points.size()));
        }

        float x1 = cloud->points[rand_pts[0]].x;
        float y1 = cloud->points[rand_pts[0]].y;
        float z1 = cloud->points[rand_pts[0]].z;

        float x2 = cloud->points[rand_pts[1]].x;
        float y2 = cloud->points[rand_pts[1]].y;
        float z2 = cloud->points[rand_pts[1]].z;

        float x3 = cloud->points[rand_pts[2]].x;
        float y3 = cloud->points[rand_pts[2]].y;
        float z3 = cloud->points[rand_pts[2]].z;

        //std::cout << "\n Selected random cloud points: " << " (" << x1 << ", " << y1 << ", " << z1 << ")" << " and " << "(" << x2 << ", " << y2 << ", " << z2 << ")" << std::endl;

        //// Define the two plane vectors
        //std::vector<std::vector<float>> v1;
        //std::vector<std::vector<float>> v2;

        //v1.push_back(x2 - x1);
        //v1.push_back(y2 - y1);
        //v1.push_back(z2 - z1);
        //v2.push_back(x3 - x1);
        //v2.push_back(y3 - y1);
        //v2.push_back(z3 - z1);

        float i;
        float j;
        float k;  // normal vector: v1 X v2 = <i, j, k>

        i = ((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1));
        j = ((z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1));
        k = ((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1));

        float a = i;
        float b = j;
        float c = k;
        float d = -((i * x1) + (j * y1) + (k * z1));

        float distance;

        // Measure distance between every point and fitted plane
        for (int index = 0; index < cloud->points.size(); index++) {

            pcl::PointXYZI point = cloud->points[index];
            distance = 0.0;

            distance = std::abs((a * point.x) + (b * point.y) + (c * point.z) + d) / std::hypot(std::hypot(a, b), c);  // hypot : sqrt(A^2 + B^2 + C^2)

            //std::cout << "\n Distance between point: " << " (" << point.x << ", " << point.y << ")" << " and plane: " << a << "x + " << b << "y + " << c << "z + " << d << " = 0 is " << distance << std::endl;

            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol) {
                current_inlier_ids.insert(index);
            }
        }

        // Return indicies of inliers from fitted line with most inliers
        if (max_inliers < static_cast<int>(current_inlier_ids.size())) {
            max_inliers = static_cast<int>(current_inlier_ids.size());
            inliersResult = current_inlier_ids;
            best_pts = rand_pts;
        }
    }

    /*std::cerr << "Best Plane Model Points: " << " (" << cloud->points[best_pts[0]].x << ", " << cloud->points[best_pts[0]].y << ", " << cloud->points[best_pts[0]].z << ")" << " and "
                                             << " (" << cloud->points[best_pts[1]].x << ", " << cloud->points[best_pts[1]].y << ", " << cloud->points[best_pts[1]].z << ")" << " and "
                                             << " (" << cloud->points[best_pts[2]].x << ", " << cloud->points[best_pts[2]].y << ", " << cloud->points[best_pts[2]].z << ")" << std::endl;*/
    
    return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();  // line
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData3D();  // plane
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);

    // Time manual RANSAC implementation
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers = RansacPlane(cloud, 25, 0.15);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC Implementation took " << elapsedTime.count() << " milliseconds" << std::endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
