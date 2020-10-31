#pragma once
/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
	PointT point;
	int id;
	std::unique_ptr<Node> left;
	std::unique_ptr<Node> right;

	Node(PointT arr, int setId)
		: point(arr), id(setId), left(), right()
	{
	}
};

template<typename PointT>
struct VectorAccessor
{
	float operator()(const PointT &p, int i) const {
		return p[i];
	}
};

template<typename PointT>
struct DataAccessor
{
	float operator()(const PointT &p, int i) const {
		return p.data[i];
	}
};

template <typename PointT, int K, typename Accessor=VectorAccessor<PointT>>
struct KdTree
{
	using PointId = std::pair<PointT, int>;
	using NodeT = Node<PointT>;
	using NodePtr = std::unique_ptr<NodeT>;

	NodePtr root;

	template<typename Array>
	void insertPoints(const Array &points)
	{
		std::vector<PointId> pointIds;
		pointIds.reserve(points.size());
		for (int i = 0; i < points.size(); ++i)
		{
			pointIds.emplace_back(points[i], i);
		}

		insertPoints(pointIds.begin(), pointIds.end());
	}

	void insert(PointT point, int id)
	{
		NodePtr *node = &root;
		int cd = 0;

		while (*node)
		{
			if (_get(point, cd) <= _get((*node)->point, cd))
			{
				node = &((*node)->left);
			}
			else
			{
				node = &((*node)->right);
			}
			cd = (cd + 1) % K;
		}

		node->reset(new NodeT(point, id));
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol) const
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, root.get(), 0, ids);
		return std::move(ids);
	}

private:
	Accessor _get;

	// Insert points in a balanced way (RandomIter is an iterator over a collection of PointId)
	template <typename RandomIter>
	void insertPoints(RandomIter begin, RandomIter end, int depth = 0)
	{
		if (begin == end)
		{
			return;
		}

		const int cd = depth % K;

		auto mid = begin + (end - begin) / 2;
		std::nth_element(begin, mid, end,
						 [cd, this](PointId &p, PointId q) {
							 return _get(p.first, cd) < _get(q.first, cd);
						 });

		insert(mid->first, mid->second);

		insertPoints(begin, mid, depth + 1);
		insertPoints(mid + 1, end, depth + 1);
	}

	float distance(const PointT &p, const PointT &q) const
	{
		float d = 0.0f;

		for (int i = 0; i < K; ++i)
		{
			const float diff = _get(p, i) - _get(q, i);
			d += diff * diff;
		}

		return std::sqrt(d);
	}

	void searchHelper(const PointT &target, float distanceTol, NodeT *node, int depth, std::vector<int> &ids) const
	{
		if (!node)
		{
			return;
		}

		if (distance(node->point, target) <= distanceTol)
		{
			ids.push_back(node->id);
		}

		const int cd = depth % K;
		const float diff = _get(target, cd) - _get(node->point, cd);
		if (diff <= distanceTol)
		{
			searchHelper(target, distanceTol, node->left.get(), depth + 1, ids);
		}

		if (diff >= - distanceTol)
		{
			searchHelper(target, distanceTol, node->right.get(), depth + 1, ids);
		}
	}
};
