/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cstdint>
#include <cmath>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
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

	// To get all X or Y coordinates in the tree
	std::vector<float> getAllXYCoordinates(Node* &p, std::vector<float> all_x_or_y, int dim) {
		
		if (p) {
			if (dim == 0) {
				//return all_x_or_y.push_back((p->point[0]));
			}
			else {
				//return all_x_or_y.push_back((p->point[1]));
			}
		}
		return all_x_or_y;
	}



	void insertHelper(Node* &p, std::vector<float> point, int index, uint32_t depth) {

		if (p == NULL) {
			p = new Node(point, index);
		}
		else {
			//std::cout << "Depth: " << depth << std::endl;
			// Calculate dim to split X or Y based on depth
			uint32_t dim = depth % 2;

			if (point[dim] < (p->point[dim])) {  // x-split
				//// sort x-point and insert x median
				//std::vector<float> x_all;
				insertHelper((p->left), point, index, depth + 1);
			}
			else {  // y-split
				insertHelper((p->right), point, index, depth + 1);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly within the root 
		insertHelper(root, point, id, 0);
	}

	// Find indices of cluster points using a distance threshold near the target
	void searchHelper(std::vector<float> target, Node* p, int depth, float distanceThresh, std::vector<int>& indices) {

		if (p!=NULL) {
			// Check if in the box: 4 line equations
			if ((p->point[0] <= (target[0] + distanceThresh)) && (p->point[1] <= (target[1] + distanceThresh)) && ((target[0] - distanceThresh) <= p->point[0]) && ((target[1] - distanceThresh) <= p->point[1])) {
				// Calculate distance
				float distance = std::sqrt(std::pow(p->point[0] - target[0], 2) + std::pow(p->point[1] - target[1], 2));
				// If point lies in the circle inside the box, add it to the cluster
				if (distance <= distanceThresh) {
					indices.push_back(p->id);
				}
			}
			uint32_t split = depth % 2;  // X or Y split | 0 or 1

			// This is to exclude the regions from the search where the box does not overlap with the split line at all (because the target will be far away from excluded regions)
			// At the same time decide which sub-tree to explore
			if (target[split] - distanceThresh < p->point[split]) {  // leftmost X or bottom Y is lesser than split X or Y
				searchHelper(target, p->left, depth + 1, distanceThresh, indices);
			}
			if (target[split] + distanceThresh > p->point[split]) {  // rightmost X or top Y is lesser than split X or Y
				searchHelper(target, p->right, depth + 1, distanceThresh, indices);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




