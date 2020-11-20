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
using namespace std;
using namespace pcl;

bool findConcaveHull(pcl::PointCloud <pcl::PointXYZ>::Ptr cloud, pcl::PointCloud <pcl::PointXYZ>::Ptr &hull)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.setAlpha (0.6);
    chull.reconstruct (*hull);
    std::cerr << "Concave hull has: " << hull->points.size () << " data points." << std::endl;
    std::cout<< "Dimensionality: "<< chull.getDimension () << std::endl;
    // for (size_t i =0; i < hull->points.size() ; ++i)
    //    std::cout << "Point " << i << " : [" << hull->points[i].x << ", " << hull->points[i].y << ", " << hull->points[i].z << std::endl;
    return true;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud < pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/centredData.pcd", *cloudPointer) == -1){
      PCL_ERROR ("Couldn't read file pcd \n");
      return (-1);
    }
    pcl::PointCloud <pcl::PointXYZ>::Ptr hull(new pcl::PointCloud <pcl::PointXYZ>);
    findConcaveHull(cloudPointer, hull);
    cout<<"Program ended"<<endl;
    pcl::io::savePCDFileASCII("../data/filteredHull.pcd", *hull);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(hull, "new cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.6, "new cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}