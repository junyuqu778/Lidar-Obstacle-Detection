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
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    //roi  
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
    
    //wipe out roof 
    std::vector<int> roof_indices;

    typename pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(roof_indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : roof_indices)
        inliers->indices.push_back(point);
    
    typename pcl::ExtractIndices<PointT> roof_extract;
    roof_extract.setInputCloud (cloudRegion);
    roof_extract.setIndices (inliers);
    roof_extract.setNegative (true);
    roof_extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}



//Segmentation with Ransac 3D
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac_seg(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) 
{
     auto startTime = std::chrono::steady_clock::now();
     std::unordered_set<int> inliersResult;
     srand(time(NULL));

     while (maxIterations--)
     {

       std::unordered_set<int> inliers;

       while (inliers.size() < 3)
       {
          inliers.insert(rand()%(cloud->points.size()));
       }

       float x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c, d;

       auto itr = inliers.begin();
       x1 = cloud->points[*itr].x;
       y1 = cloud->points[*itr].y;
       z1 = cloud->points[*itr].z;
       itr++;
       x2 = cloud->points[*itr].x;
       y2 = cloud->points[*itr].y;
       z2 = cloud->points[*itr].z;
       itr++;
       x3 = cloud->points[*itr].x;
       y3 = cloud->points[*itr].y;
       z3 = cloud->points[*itr].z;

       a = (y2 - y1)*(z3-z1)-(z2-z1)*(y3-y1);
       b = (z2 - z1)*(x3-x1)-(x2-x1)*(z3-z1);
       c = (x2 - x1)*(y3-y1)-(y2-y1)*(x3-x1);
       d = -(a*x1 + b*y1 + c*z1);

       for (int index = 0; index < cloud->points.size(); index++)
       {

         if (inliers.count(index) > 0)
           continue;

         float x4 = cloud->points[index].x;
         float y4 = cloud->points[index].y;
         float z4 = cloud->points[index].z;
         float dist = fabs(a * x4 + b * y4 + c *z4 + d) / sqrt(a * a + b * b +c * c);

         if (dist <= distanceTol)
           inliers.insert(index);
       }

       if(inliers.size() > inliersResult.size())
          inliersResult = inliers;

     }
     auto endTime = std::chrono::steady_clock::now();
     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

     typename pcl::PointCloud<PointT>::Ptr  plane_cloud(new typename pcl::PointCloud<PointT>());
     typename pcl::PointCloud<PointT>::Ptr obst_cloud(new typename pcl::PointCloud<PointT>());

     for(int index = 0; index < cloud->points.size(); index++)
     {
         PointT point = cloud->points[index];
         if(inliersResult.count(index))
           plane_cloud->points.push_back(point);
         else
           obst_cloud->points.push_back(point);
     }
     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> seg_result(obst_cloud, plane_cloud);
     return seg_result;
}



//Segmentation with PCL liarary
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now(); 
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // Creating the segmentation objecte
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largerst planar component from the cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout<< "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}



// Euclidean clusterHelper
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int id, const std::vector<PointT, Eigen::aligned_allocator<PointT>>* points, std::vector<int>& cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{ 
    processed[id] = true;
    cluster.push_back(id);
    std::vector<int> nearest = tree->search((*points)[id], distanceTol);

    for (int ix : nearest)
    {
        if (!processed[ix])
            clusterHelper(ix, points, cluster, processed, tree, distanceTol);
        
    }
}


// Euclidean clustering
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const typename std::vector<PointT, Eigen::aligned_allocator<PointT>>& points, KdTree* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);
    for (int i = 0; i < points.size(); i++){
        if (!processed[i])
          {
            std::vector<int> cluster;
            clusterHelper(i, &points, cluster, processed, tree, distanceTol);
            clusters.push_back(cluster);
          }
    }
    return clusters;
}


// KDTree clustring
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::KDTreeClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->points.size(); i++){ 
        tree->insert(cloud->points[i],i);
    }

    std::vector<std::vector<int>> clusters_indices = euclideanCluster(cloud->points, tree, clusterTolerance);
    for (auto cluster_itr = clusters_indices.begin(); cluster_itr != clusters_indices.end(); cluster_itr++)
    {
        if (cluster_itr->size() < minSize || cluster_itr->size() > maxSize)
            continue;

        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new typename pcl::PointCloud<PointT>);

        for (auto ii = cluster_itr->begin(); ii != cluster_itr->end(); ii++){
            cloud_cluster->points.push_back(cloud->points[*ii]);
        } 
      
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;


        clusters.push_back(cloud_cluster);
    }    
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}



//Euclidean Clustering with PCL library
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for (pcl::PointIndices getIndices:clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for (auto index:getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloud->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}



//add bounding box to the obstacles
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