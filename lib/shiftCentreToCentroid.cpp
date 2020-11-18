#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
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

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud < pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/pcd_data.pcd", *cloudPointer) == -1) {
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}

	Eigen::Vector4f centroid; 
	pcl::compute3DCentroid (*cloudPointer, centroid); 
	cout << "centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n";
 	Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
	transform_1.translation() << -centroid[0], -centroid[1], -centroid[2];
	std::cout << "Transformation matrix: " << std::endl << transform_1.matrix() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloudPointer, *transformed_cloud1, transform_1);

    // FOR ROTATING AND ALIGNING
	// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// pcl::SACSegmentation<pcl::PointXYZ> seg;
	// seg.setOptimizeCoefficients (true);
	// seg.setModelType (pcl::SACMODEL_PLANE);
	// seg.setMethodType (pcl::SAC_RANSAC);
	// seg.setDistanceThreshold (0.01);
	// seg.setInputCloud (transformed_cloud1);
	// seg.segment (*inliers, *coefficients);
	// Eigen::Matrix<float, 1, 3> plane_normal_vector, xy_plane_normal_vector, rotation_vector;
	// plane_normal_vector[0] = coefficients->values[0];
	// plane_normal_vector[1] = coefficients->values[1];
	// plane_normal_vector[2] = coefficients->values[2];
	// std::cout << plane_normal_vector << std::endl;
	// xy_plane_normal_vector[0] = 0.0;
	// xy_plane_normal_vector[1] = 0.0;
	// xy_plane_normal_vector[2] = 1.0;
	// std::cout << xy_plane_normal_vector << std::endl;
	// rotation_vector = xy_plane_normal_vector.cross(plane_normal_vector);
	// rotation_vector.normalized();
	// std::cout << "Rotation Vector: "<< rotation_vector << std::endl;
	// float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(plane_normal_vector));
	// Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	// transform_2.rotate(Eigen::AngleAxisf (theta, rotation_vector));
	// std::cout << "Transformation matrix: " << std::endl << transform_2.matrix() << std::endl;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
	// pcl::transformPointCloud (*transformed_cloud1, *transformed_cloud2, transform_2);

	pcl::io::savePCDFileASCII("../data/centredData.pcd", *transformed_cloud1);//This place fills the output path

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud1, "shifted cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.6, "shifted cloud");
	viewer->addPointCloud<pcl::PointXYZ>(cloudPointer, "original cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.6, "original cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}