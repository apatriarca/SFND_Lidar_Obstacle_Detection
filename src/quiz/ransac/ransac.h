#pragma once

#include <pcl/common/common.h>

#include <cmath>
#include <unordered_set>
#include <random>

// RANSAC Algorithm with a Plane model
template <typename PointT>
std::unordered_set<int> PlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> uniform(0, cloud->points.size());

    std::unordered_set<int> result;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // Choose 3 random point indices
        int id0, id1, id2;
        do
        {
            id0 = uniform(gen);
            id1 = uniform(gen);
            id2 = uniform(gen);
        } while (id0 == id1 || id1 == id2 || id2 == id0);

        // Retrieves the points
        PointT p0 = cloud->points[id0];
        PointT p1 = cloud->points[id1];
        PointT p2 = cloud->points[id2];

        // Only uses the XYZ part of the point to compute the plane
        pcl::PointXYZ v1{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
        pcl::PointXYZ v2{p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};

        // Computes the plane equation Ax + By + Cz + D = 0
        float A = v1.y * v2.z - v1.z * v2.y;
        float B = v1.z * v2.x - v1.x * v2.z;
        float C = v1.x * v2.y - v1.y * v2.x;
        float D = -A * p0.x - B * p0.y - C * p0.z;

        std::unordered_set<int> planeInliers;

        // Measure distance between every point and fitted plane
        // If distance is smaller than threshold count it as inlier
        float c = 1.0f / std::sqrt(A * A + B * B + C * C);
        for (int i = 0; i < (int)cloud->points.size(); ++i)
        {
            PointT p = cloud->points[i];
            float dist = c * std::fabs(A * p.x + B * p.y + C * p.z + D);
            if (dist <= distanceTol)
            {
                planeInliers.insert(i);
            }
        }

        if (result.size() < planeInliers.size())
        {
            std::swap(result, planeInliers);
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    return std::move(result);
}
