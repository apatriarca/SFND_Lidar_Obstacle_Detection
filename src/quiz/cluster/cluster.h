#pragma once

#include "kdtree.h"

#include <queue>
#include <unordered_set>
#include <vector>

template <typename PointT, int K, typename Accessor=VectorAccessor<PointT>, typename Array=std::vector<PointT>>
std::vector<std::vector<int>> euclideanCluster(const Array &points, KdTree<PointT, K, Accessor> &tree, float distanceTol,
                                               int minPoints = 0, int maxPoints = INT_MAX)
{
    std::vector<std::vector<int>> clusters;
    std::unordered_set<int> processed;

    for (int id = 0; id < points.size(); ++id)
    {
        if (processed.count(id) != 0)
        {
            continue;
        }

        std::vector<int> cluster;

        std::queue<int> to_visit;
        std::unordered_set<int> in_queue;
        
        to_visit.push(id);
        in_queue.insert(id);

        while (!to_visit.empty())
        {
            int pid = to_visit.front();
            to_visit.pop();
            in_queue.erase(pid);
            
            processed.insert(pid);
            cluster.push_back(pid);

            std::vector<int> near = tree.search(points[pid], distanceTol);
            for (int nid : near)
            {
                if (processed.count(nid) == 0 && in_queue.count(nid) == 0)
                {
                    to_visit.push(nid);
                    in_queue.insert(nid);
                }
            }
        }

        if (cluster.size() > minPoints && cluster.size() < maxPoints) {
            clusters.emplace_back(std::move(cluster));
        }
    }

    return clusters;
}
