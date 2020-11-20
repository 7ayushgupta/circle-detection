// WRITTEN BY AYUSH GUPTA (WEDNESDAY) LINKWIZ RECRUITMENT IITK
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_circle.h>
using namespace std;
using namespace pcl;

void detect_circle(pcl::PointCloud<PointXYZ>::Ptr &cloud, pcl::ModelCoefficients::Ptr &coefficients_circle, pcl::PointIndices::Ptr &inliers_plane, int kSearchRadius, int maxRadius, int distanceThreshold, bool is3D)
{
	pcl::PCDReader reader;
	pcl::PassThrough<PointXYZ> pass;
	pcl::NormalEstimation<PointXYZ, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointXYZ, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointXYZ> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());
	pcl::PointCloud<PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(kSearchRadius);
	ne.compute(*cloud_normals);
	seg.setOptimizeCoefficients(true);
	if(is3D)
		seg.setModelType(pcl::SACMODEL_CIRCLE3D);
	else
		seg.setModelType(pcl::SACMODEL_CIRCLE2D);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setRadiusLimits(0.00007, maxRadius);
	seg.setDistanceThreshold(distanceThreshold);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_plane, *coefficients_circle);
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud < pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>),  cloud_g(new pcl::PointCloud<pcl::PointXYZ>), cloud_h(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/filteredHull.pcd", *cloudPointer) == -1){
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}	
	std::cerr << "PointCloud dimensions: " << cloudPointer->width * cloudPointer->height << " data points." << std::endl;

	//finding the first circle
	pcl::PointIndices::Ptr circle1(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_circle1(new pcl::ModelCoefficients);
	detect_circle(cloudPointer, coefficients_circle1, circle1, 50, 10, 1, true);
	extract.setInputCloud(cloudPointer);
	extract.setIndices(circle1);
	extract.setNegative(true);
	extract.filter(*cloud_f);
	std::cerr << "PointCloud representing the first component: " << cloud_f->width * cloud_f->height << " data points." << std::endl;
	// for (size_t i = 0; i < cloud_f->points.size(); ++i)
	// 	std::cerr << "    " << cloud_f->points[i].x << " " << cloud_f->points[i].y << " " << cloud_f->points[i].z << std::endl;

	//finding the circle for the larger rectangle
	pcl::PointIndices::Ptr circle_(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_circle_(new pcl::ModelCoefficients);
	detect_circle(cloud_f, coefficients_circle_, circle_, 1000, 50, 11, false);
	extract.setInputCloud(cloud_f);
	extract.setIndices(circle_);
	extract.setNegative(true);
	extract.filter(*cloud_g);
	std::cerr << "PointCloud representing the second component: " << cloud_g->width * cloud_g->height << " data points." << std::endl;
	// for (size_t i = 0; i < cloud_g->points.size(); ++i)
	// 	std::cerr << "    " << cloud_g->points[i].x << " " << cloud_g->points[i].y << " " << cloud_g->points[i].z << std::endl;

	//finding the circle for the smaller circle
	pcl::PointIndices::Ptr circle2(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_circle2(new pcl::ModelCoefficients);
	detect_circle(cloud_g, coefficients_circle2, circle2, 30, 5, 3, true);
	extract.setInputCloud(cloud_g);
	extract.setIndices(circle2);
	extract.setNegative(false);
	extract.filter(*cloud_h);
	std::cerr << "PointCloud representing the third component: " << cloud_h->width * cloud_h->height << " data points." << std::endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_f, "transformed cloud1");
	viewer->addPointCloud<pcl::PointXYZ>(cloud_h, "transformed cloud3");
	
	cout<<"--------------- INFORMATION -------------"<<endl;
	cout<<"CIRCLE 1"<<endl;
	cout<<"w.r.t x_centroid: "<<coefficients_circle1->values[0]<<"\t w.r.t x_origin: "<<coefficients_circle1->values[0]+63.1533<<endl;
	cout<<"w.r.t y_centroid: "<<coefficients_circle1->values[1]<<"\t w.r.t y_origin: "<<coefficients_circle1->values[1]-113.397<<endl;
	cout<<"w.r.t z_centroid: "<<coefficients_circle1->values[2]<<"\t w.r.t z_origin: "<<coefficients_circle1->values[2]-28.1117<<endl;
	cout<<"Radius (cm): "<<coefficients_circle1->values[3]<<endl;
	cout<<"Normal_x: "<<coefficients_circle1->values[4]<<endl;
	cout<<"Normal_y: "<<coefficients_circle1->values[5]<<endl;
	cout<<"Normal_z: "<<coefficients_circle1->values[6]<<endl;
	cout<<"------------------------------------"<<endl;
	cout<<"CIRCLE 2"<<endl;
	cout<<"w.r.t x_centroid: "<<coefficients_circle2->values[0]<<"\t w.r.t x_origin: "<<coefficients_circle2->values[0]+63.1533<<endl;
	cout<<"w.r.t y_centroid: "<<coefficients_circle2->values[1]<<"\t w.r.t y_origin: "<<coefficients_circle2->values[1]-113.397<<endl;
	cout<<"w.r.t z_centroid: "<<coefficients_circle2->values[2]<<"\t w.r.t z_origin: "<<coefficients_circle2->values[2]-28.1117<<endl;
	cout<<"Radius (cm): "<<coefficients_circle2->values[3]<<endl;
	cout<<"Normal_x: "<<coefficients_circle2->values[4]<<endl;
	cout<<"Normal_y: "<<coefficients_circle2->values[5]<<endl;
	cout<<"Normal_z: "<<coefficients_circle2->values[6]<<endl;

	pcl::ModelCoefficients sphere_coeff1; sphere_coeff1.values.resize (4); 
	sphere_coeff1.values[0] = coefficients_circle1->values[0]; 
	sphere_coeff1.values[1] = coefficients_circle1->values[1]; 
	sphere_coeff1.values[2] = coefficients_circle1->values[2]; 
	sphere_coeff1.values[3] = coefficients_circle1->values[3];

	pcl::ModelCoefficients sphere_coeff2; sphere_coeff2.values.resize (4); 
	sphere_coeff2.values[0] = coefficients_circle2->values[0]; 
	sphere_coeff2.values[1] = coefficients_circle2->values[1]; 
	sphere_coeff2.values[2] = coefficients_circle2->values[2]; 
	sphere_coeff2.values[3] = coefficients_circle2->values[3];

	viewer->addSphere(sphere_coeff1,"circle1");
	viewer->addSphere(sphere_coeff2,"circle2");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
  return 0;
}