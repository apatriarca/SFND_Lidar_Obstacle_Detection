/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <algorithm>


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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		Node ** node = &root;
		int cd = 0;

		while (*node != NULL) {
			if (point[cd] <= (*node)->point[cd]) {
				node = &((*node)->left);
			} else {
				node = &((*node)->right);
			}
			cd = (cd + 1) % point.size();
		}

		*node = new Node(point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) const
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, root, 0, ids);
		return ids;
	}
	

private:
	float distance(const std::vector<float>& P, const std::vector<float>& Q) const
	{
		float d = 0.0f;

		for (int i = 0; i < P.size(); ++i) {
			float diff = P[i] - Q[i];
			d += diff * diff;
		}

		return sqrtf(d);
	}

	void searchHelper(const std::vector<float>& target, float distanceTol, Node* node, int depth, std::vector<int>& ids) const
	{
		if (!node) {
			return;
		}

		if (distance(node->point, target) <= distanceTol) {
			ids.push_back(node->id);
		}

		const int cd = depth % target.size();
		if ((target[cd] - distanceTol) <= node->point[cd]) {
			searchHelper(target, distanceTol, node->left, depth + 1, ids);
		}

		if ((target[cd] + distanceTol) >= node->point[cd]) {
			searchHelper(target, distanceTol, node->right, depth + 1, ids);
		}
	}
};




