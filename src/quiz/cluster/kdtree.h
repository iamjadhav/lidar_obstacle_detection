/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include "../../processPointClouds.h"
//#include "../../processPointClouds.cpp"
#include <cstdint>
#include <cmath>


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	//Node(std::vector<float> arr, int setId)
	//:	point(arr), id(setId), left(NULL), right(NULL)
	//{}

	Node(pcl::PointXYZI arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	std::pair<float, float> getSplit(pcl::PointXYZI point, Node* &n, uint32_t dimension) {

		if (dimension % 3 == 0) {
			return std::make_pair(point.x, n->point.x);
		}
		else if (dimension % 3 == 1) {
			return std::make_pair(point.y, n->point.y);
		}
		else {
			return std::make_pair(point.z, n->point.z);
		}
	}

	void insertHelper(Node* &p, pcl::PointXYZI point, int index, uint32_t depth) {

		if (p == NULL) {
			p = new Node(point, index);
		}
		else {
			//std::cout << "Depth: " << depth << std::endl;
			// Calculate dim to split X or Y or Z based on depth
			uint32_t dim = depth % 3;

			std::pair<float, float> pts = getSplit(point, p, dim);

			//std::cout << "Depth: " << depth << " and point: " << " (" << point.x << ", " << point.y << ", " << point.z << ")" << "\n" << std::endl;

			if (pts.first < pts.second) {  // insert on left as pt[depth] coor is lesser
				//// sort x-point and insert x median
				//std::vector<float> x_all;
				insertHelper((p->left), point, index, depth + 1);
			}
			else {  // insert on right as pt[depth] coor is greater
				insertHelper((p->right), point, index, depth + 1);
			}
		}
	}

	//void insert(std::vector<float> point, int id)
	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly within the root 
		insertHelper(root, point, id, 0);
	}

	// Find indices of cluster points using a distance threshold near the target
	void searchHelper(pcl::PointXYZI target, Node* &p, int depth, float distanceThresh, std::vector<int>& indices) {

		if (p!=NULL) {
			// Check if in the box: 4 line equations
			if ((p->point.x <= (target.x + distanceThresh)) && (p->point.y <= (target.y + distanceThresh)) && ((target.x - distanceThresh) <= p->point.x) && ((target.y - distanceThresh) <= p->point.y)
																											   && (p->point.z <= (target.z + distanceThresh)) && ((target.z - distanceThresh) <= p->point.z)) {
				// Calculate distance
				float distance = std::sqrt(std::pow(p->point.x - target.x, 2) + std::pow(p->point.y - target.y, 2) + std::pow(p->point.z - target.z, 2));
				// If point lies in the circle inside the box, add it to the cluster
				if (distance <= distanceThresh) {
					indices.push_back(p->id);
				}
			}
			uint32_t dim = depth % 3;  // X or Y split | 0 or 1

			std::pair<float, float> pts = getSplit(target, p, dim);

			// This is to exclude the regions from the search where the box does not overlap with the split line at all (because the target will be far away from excluded regions)
			// At the same time decide which sub-tree to explore
			if (pts.first - distanceThresh < pts.second) {  // leftmost X or bottom Y is lesser than split X or Y
				searchHelper(target, p->left, depth + 1, distanceThresh, indices);
			}
			if (pts.first + distanceThresh > pts.second) {  // rightmost X or top Y is lesser than split X or Y
				searchHelper(target, p->right, depth + 1, distanceThresh, indices);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	
};




