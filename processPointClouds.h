// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
#include <random>

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    
    //ransac 3d
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac_seg(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);


    //kd-tree
    struct Node
    {
	    PointT point;
	    int id;
	    Node* left;
	    Node* right;

       Node(PointT arr, int setId)
       :	point(arr), id(setId), left(NULL), right(NULL)
       {}
    };
    
    struct KdTree
    {
        Node *root;

        KdTree()
        : root(NULL)
        {}

        void insertHelper(Node*& node, PointT point, int depth, int id)
        {
            // TODO: for 3D data
            if (node == NULL)
               node = new Node(point, id);
            else
            {
                // TODO: maybe use four values intensity
                
               if (node->point.data[depth%3] >= point.data[depth%3])
                   insertHelper(node->left, point, depth+1, id);
               else
                   insertHelper(node->right, point, depth+1, id);
            }
        }
        
        void insert(PointT point, int id)
        { 
            insertHelper(root, point, 0, id);
        }

        void searchHelper(Node* node_, PointT target, std::vector<int>* ids_, int depth, float distanceTol)
        {
        // TODO: for 3D data
            if (node_ == NULL)
                return;

		    int cd = depth % 3;
		    float box_x_min = target.data[0] - distanceTol;
		    float box_x_max = target.data[0] + distanceTol;
		    float box_y_min = target.data[1] - distanceTol;
		    float box_y_max = target.data[1] + distanceTol;
		    float box_z_min = target.data[2] - distanceTol;
		    float box_z_max = target.data[2] + distanceTol;
		    if (node_->point.data[0]>box_x_min && node_->point.data[0]<box_x_max && 
		    	node_->point.data[1]>box_y_min && node_->point.data[1]<box_y_max && 
			    node_->point.data[2]>box_z_min && node_->point.data[2]<box_z_max)
		    {
			    float distance = std::sqrt((node_->point.data[0]-target.data[0])*(node_->point.data[0]-target.data[0])+
									       (node_->point.data[1]-target.data[1])*(node_->point.data[1]-target.data[1])+
									       (node_->point.data[2]-target.data[2])*(node_->point.data[2]-target.data[2]));
			    if (distance <= distanceTol)
				    ids_->push_back(node_->id);
		    }

		    if ((target.data[cd]-distanceTol) < node_->point.data[cd])
			    searchHelper(node_->left, target, ids_, depth+1, distanceTol);
		    if ((target.data[cd]+distanceTol) > node_->point.data[cd])
			    searchHelper(node_->right, target, ids_, depth+1, distanceTol);
	    }

	    // return a list of point ids in the tree that are within distance of target
	    std::vector<int> search(PointT target, float distanceTol)
	    {
		    std::vector<int> ids;
		    searchHelper(root, target, &ids, 0, distanceTol);
		    return ids;
	    }
    };
  
  // Euclidean Clustring
    void clusterHelper(int id, const std::vector<PointT, Eigen::aligned_allocator<PointT>>* points, std::vector<int>& cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol);
    std::vector<std::vector<int>> euclideanCluster(const typename std::vector<PointT, Eigen::aligned_allocator<PointT>>& points, KdTree* tree, float distanceTol);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> KDTreeClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

  
};
#endif /* PROCESSPOINTCLOUDS_H_ */