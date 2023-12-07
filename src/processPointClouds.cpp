// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    /*std::cerr << "\nPoint cloud before filtering: " << cloud->width * cloud->height << " data points ("
              << pcl::getFieldsList(*cloud) << ")." << std::endl;*/

    // Filtering object
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(filterRes, filterRes, filterRes);
    vox.filter(*filtered_cloud);

    typename pcl::PointCloud<PointT>::Ptr filtered_cloud_roi(new pcl::PointCloud<PointT>());

    // ROI with cropbox
    pcl::CropBox<PointT> crop(true);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setInputCloud(filtered_cloud);
    crop.filter(*filtered_cloud_roi);

    // Remove ego car roof points
    std::vector<int> indices_roof;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(filtered_cloud_roi);
    roof.filter(indices_roof);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (int point : indices_roof) {
        inliers->indices.push_back(point);
    }

    // Extract Filter object to get the roof
    pcl::ExtractIndices<PointT> extract;

    // Extract inliers and assign remaining pts to obstacle cloud
    extract.setInputCloud(filtered_cloud_roi);
    extract.setIndices(inliers);
    extract.setNegative(true);  // negative True -> all except inliers / remove these indices
    extract.filter(*filtered_cloud_roi);  // cloud without the indices

    /*std::cerr << "\nPoint cloud after filtering: " << filtered_cloud_roi->width * filtered_cloud_roi->height << " data points ("
        << pcl::getFieldsList(*filtered_cloud_roi) << ")." << std::endl;*/

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered_cloud_roi;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    // Non-plane and Plane points
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud (new pcl::PointCloud<PointT> ()), road_cloud( new pcl::PointCloud<PointT> ());

    // Add inliers to the road plane cloud
    for (auto pt = inliers->indices.begin(); pt != inliers->indices.end(); ++pt) {
        road_cloud->points.push_back(cloud->points[*pt]);
    }

    // Extract Filter object
    pcl::ExtractIndices<PointT> extract;

    // Extract inliers and assign remaining pts to obstacle cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // negative True -> all except inliers
    extract.filter(*obstacle_cloud);

    //// Verify sizes
    //std::cerr << "\nEntire PointCloud size: " << cloud->width * cloud->height << " data points. \n" << std::endl;
    //std::cerr << "Obstacle PointCloud size: " << obstacle_cloud->width * obstacle_cloud->height << " data points. \n" << std::endl;
    //std::cerr << "Road PointCloud size: " << road_cloud->width * road_cloud->height << " data points. \n" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, road_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    //std::cerr << "PointCloud size: " << cloud->width * cloud->height << " data points. \n" << std::endl;

    //// View first 20 points
    //int count = 0;
    //for (auto pt = cloud->begin(); pt != cloud->end() && count < 20; ++pt, ++count) {
    //    const auto& point = *pt;
    //    std::cerr << "    " << point.x << " "
    //                        << point.y << " "
    //                        << point.z << std::endl;
    //}
    //count = 0;

    // Model coefficietns and inliers objects
    pcl::ModelCoefficients::Ptr coefficients{ new pcl::ModelCoefficients };
    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
    
    // SAC object
    pcl::SACSegmentation<PointT> seg;

    // Optional param
    seg.setOptimizeCoefficients(true);

    // Mandatory params
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment model plane using RANSAC
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);  // coeffs not used

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not fit a plane to the dataset. \n");
    }

    //// View first 20 inliers
    //std::cerr << "\n Inliers: \n" << std::endl;
    //for (auto in_id = inliers->indices.begin(); in_id != inliers->indices.end() && count < 20; in_id++, count++) {
    //    const auto& inlier_id = *in_id;
    //    std::cerr << "    " << cloud->points[inlier_id].x << " "
    //                        << cloud->points[inlier_id].y << " "
    //                        << cloud->points[inlier_id].z << std::endl;
    //}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "\nplane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    // Separated Obstacle point cloud and Road point cloud
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Kd-tree object
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // Kd-tree search params
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> euclid;
    euclid.setClusterTolerance(clusterTolerance);  // x cm
    euclid.setMinClusterSize(minSize);
    euclid.setMaxClusterSize(maxSize);
    euclid.setSearchMethod(tree);
    euclid.setInputCloud(cloud);
    // Get cluster indices: cluster_indices[0] will have all points in the first cluster
    euclid.extract(cluster_indices);

    // Assign the cluster_indices to separate cloud clusters to fill them with the resp points
    for (pcl::PointIndices cluster: cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr curr_cluster(new pcl::PointCloud<PointT>);
        for (int index: cluster.indices) {
            curr_cluster->points.push_back(cloud->points[index]);
        }
        curr_cluster->width = curr_cluster->size();
        curr_cluster->height = 1;
        curr_cluster->is_dense = true;

        clusters.push_back(curr_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}