/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
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
	
	for (int iter = 0; iter < maxIterations; ++iter) {
		// Randomly sample subset and fit line
		int idA, idB;
		do {
			idA = rand() % cloud->points.size();
			idB = rand() % cloud->points.size();
		} while (idA == idB);

		float lineA = cloud->points[idA].y - cloud->points[idB].y;
		float lineB = cloud->points[idB].x - cloud->points[idA].x;
		float lineC = cloud->points[idA].x * cloud->points[idB].y - cloud->points[idA].y * cloud->points[idB].x;

		std::unordered_set<int> inliers;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		float k = 1.0f / sqrtf(lineA * lineA + lineB * lineB);
		for (int i = 0; i < cloud->points.size(); ++i) {
			float dist = k * fabsf(lineA * cloud->points[i].x + lineB * cloud->points[i].y + lineC);
			if (dist < distanceTol) {
				inliers.insert(i);
			}
		}

		if (inliers.size() > inliersResult.size()) {
			std::swap(inliers, inliersResult);
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> PlaneRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	for (int iter = 0; iter < maxIterations; ++iter) {
		int id0, id1, id2;
		do {
			id0 = rand() % cloud->points.size();
			id1 = rand() % cloud->points.size();
			id2 = rand() % cloud->points.size();
		} while (id0 == id1 || id1 == id2 || id0 == id2);

		pcl::PointXYZ p0 = cloud->points[id0];
		pcl::PointXYZ p1 = cloud->points[id1];
		pcl::PointXYZ p2 = cloud->points[id2];

		pcl::PointXYZ v1 { p1.x - p0.x, p1.y - p0.y, p1.z - p0.z };
		pcl::PointXYZ v2 { p2.x - p0.x, p2.y - p0.y, p2.z - p0.z };

		float A = v1.y * v2.z - v1.z * v2.y;
		float B = v1.z * v2.x - v1.x * v2.z;
		float C = v1.x * v2.y - v1.y * v2.x;
		float D = - A * p0.x - B * p0.y - C * p0.z;

		std::unordered_set<int> inliers;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		float k = 1.0f / sqrtf(A * A + B * B + C * C);
		for (int i = 0; i < cloud->points.size(); ++i) {
			pcl::PointXYZ p = cloud->points[i];
			float dist = k * fabsf(A * p.x + B * p.y + C * p.z + D);
			if (dist <= distanceTol) {
				inliers.insert(i);
			}
		}

		if (inliers.size() > inliersResult.size()) {
			std::swap(inliers, inliersResult);
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = PlaneRansac(cloud, 1000, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
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
