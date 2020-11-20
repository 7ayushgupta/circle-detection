// WRITTEN BY AYUSH GUPTA (WEDNESDAY) LINKWIZ RECRUITMENT IITK
#include<iostream>
#include<fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
using namespace std;
using namespace pcl;

typedef struct Point3D
{
	double x;  
	double y;  
	double z; 
	double r;
};

int main()
{
	int num_points;
	FILE *filep_txt;
	Point3D point;
	vector<Point3D> convertedPoints;
	filep_txt = fopen("../data/Hole_Detection_Test_Data.txt", "r");
	if (filep_txt)
		while (fscanf(filep_txt, "%lf %lf %lf", &point.x, &point.y, &point.z) != EOF)
			convertedPoints.push_back(point);
	else
		cout << "Failed to load txt data!" << endl;
	num_points = convertedPoints.size();
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = num_points;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = convertedPoints[i].x;
		cloud.points[i].y = convertedPoints[i].y;
		cloud.points[i].z = convertedPoints[i].z;
	}
	pcl::io::savePCDFileASCII("../data/pcd_data.pcd", cloud);//This place fills the output path
	std::cerr << "Saved " << cloud.points.size() << " data points to pcd_data.pcd." << std::endl;
	// for (size_t i = 0; i < cloud.points.size(); ++i)
	// 	std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
	return 0;

}