//C++
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

//OpenCV
#include <opencv2/opencv.hpp>

//boost
#include <boost/thread/thread.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
//Customed
#include "gocator3200.h"
#include "config.h"
#include "pointcloud_helper.h"
#include "dbscan.h"
#include "DefectDetect.h"


using namespace std;
using namespace cv;

#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445
#endif
#ifndef pi
#define pi 3.1415926535897932384626433832795028841971693993751058209749445
#endif

//main
int main(int argc, char **argv)
{
	string paraFileName = "../parameters/parameter.yml";
	Config::setParameterFile(paraFileName);

	Gocator3200::Device device("192.168.1.10");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	device.getSingleSnapshot(*cloud);
	std::cout<<"point cloud size:"<<cloud->points.size()<<std::endl;

	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, 0, 255, 0);
	viewer->addPointCloud(cloud, "cloud");
	viewer->addCoordinateSystem(10);
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
	viewer->updatePointCloud(cloud, "cloud");
	viewer->removePointCloud("cloud");
	viewer->removeAllPointClouds();
	viewer->close();
	device.stop();

	std::vector<float> res=process(cloud);
	for (int i = 0; i < res.size(); ++i)
	{
		std::cout<<res[i]<<std::endl;
	}

	return 0;
}