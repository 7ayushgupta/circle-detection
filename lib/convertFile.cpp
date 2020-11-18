#include<iostream>
#include<fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
using namespace std;
using namespace pcl;

typedef struct tagPOINT_3D
{
	double x;  //mm world coordinate x
	double y;  //mm world coordinate y
	double z;  //mm world coordinate z
	double r;
}POINT_WORLD;

int main()
{
	/////Load txt data
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_3D TxtPoint;
	vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen("../data/Hole_Detection_Test_Data.txt", "r");//The location of the file filled in this place
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "Failed to load txt data!" << endl;
	number_Txt = m_vTxtPoints.size();
	pcl::PointCloud<pcl::PointXYZ> cloud;
	// Fill in the cloud data
	cloud.width = number_Txt;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = m_vTxtPoints[i].x;
		cloud.points[i].y = m_vTxtPoints[i].y;
		cloud.points[i].z = m_vTxtPoints[i].z;
	}
	pcl::io::savePCDFileASCII("../data/pcd_data.pcd", cloud);//This place fills the output path
	std::cerr << "Saved " << cloud.points.size() << " data points to pcd_data.pcd." << std::endl;
	// for (size_t i = 0; i < cloud.points.size(); ++i)
	// 	std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

	return 0;

}