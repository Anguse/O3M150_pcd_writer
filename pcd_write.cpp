#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <sstream>
#include <string>

extern "C" {
  #include <CustomerExample.h>
  #include <CustomerExample.c>
  #include <DI.h>
  #include <DI.c>
  #include <DI_CH08_1_4.h>
  #include <DI_CH08_1_4.c>
  #include <DI_CH14_1_4.h>
  #include <DI_CH14_1_4.c>
  #include <DI_CH20_1_4.h>
  #include <DI_CH20_1_4.c>
  #include <DI_CH256_1_5.h>
  #include <DI_CH256_1_5.c>
}

static int m_argc;
static char** m_argv;

//Creates one pcd cloud with the name file
//OLD
void create_frame(std::string file) {
	customData_t data;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	//Get 1 frame of data, make sure its valid
	while(cloud.empty()){
		refreshCam(m_argc, m_argv);
		data = getData();

		// Fill in the cloud data
		cloud.width = data.distanceImageResult_sensorWidth;
		cloud.height = data.distanceImageResult_sensorHeight;
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		for (size_t i = 0; i < cloud.points.size(); ++i)
		{
			cloud.points[i].x = data.distanceImageResult_X[i];
			cloud.points[i].y = data.distanceImageResult_Y[i];
			cloud.points[i].z = data.distanceImageResult_Z[i];
		}
		if (cloud.empty())
		{
			std::cout << "Missing packet, retrying...";
		}
	}
	std::stringstream ss;
	ss << "clouds/" << file << ".pcd";
	std::string fileName = ss.str();
	pcl::io::savePCDFileASCII(fileName, cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

	return;
}
//Creates 60 pcd clouds with the name file
//OLD
void create_frame_serie(std::string file){
	customData_t data;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	//Get data from 30seconds of recording
	for (int j = 0; j < 60; j++)
	{
			refreshCam(m_argc, m_argv);
			data = getData();

			// Fill in the cloud data
			cloud.width = data.distanceImageResult_sensorWidth;
			cloud.height = data.distanceImageResult_sensorHeight;
			cloud.is_dense = false;
			cloud.points.resize(cloud.width * cloud.height);

			for (size_t i = 0; i < cloud.points.size(); ++i)
			{
				cloud.points[i].x = data.distanceImageResult_X[i];
				cloud.points[i].y = data.distanceImageResult_Y[i];
				cloud.points[i].z = data.distanceImageResult_Z[i];
			}
			if (cloud.empty())
			{
				std::cout << "Missing packet, retrying...";
			}
		std::stringstream ss;
		ss << "clouds/series/" << file << j << ".pcd";
		std::string fileName = ss.str();
		pcl::io::savePCDFileASCII(fileName, cloud);
		std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
	}

	std::cout << "Complete";
	return;
}
//Creates a 5 second overview of the enviroment
//NEW
/*pcl::PointCloud<pcl::PointXYZ>::Ptr create_frame_overview() {
	refreshCam(m_argc,m_argv);
	return getOverview();
}*/
///////Visualizer functions//////////////
void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualisation example\n"
		<< "-r           RGB colour visualisation example\n"
		<< "-c           Custom colour visualisation example\n"
		<< "-n           Normals visualisation example\n"
		<< "-a           Shapes visualisation example\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	//------------------------------------
	//-----Add shapes at cloud points-----
	//------------------------------------
	viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],
		cloud->points[cloud->size() - 1], "line");
	viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

	//---------------------------------------
	//-----Add shapes at other locations-----
	//---------------------------------------
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");

	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);

	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

		char str[512];
		sprintf(str, "text#%03d", text_id++);
		viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);


	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer.get());

	return (viewer);
}
//Visualizes @param basic_cloud_ptr in a new command window
int live_visualize() {
	// ------------------------------------
	// -----Create example point cloud-----
	// ------------------------------------

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	customData_t data;

	refreshCam(m_argc, m_argv);
	data = getData();

	// Fill in the cloud data
	cloud->width = data.distanceImageResult_sensorWidth;
	cloud->height = data.distanceImageResult_sensorHeight;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = data.distanceImageResult_X[i];
		cloud->points[i].y = data.distanceImageResult_Y[i];
		cloud->points[i].z = data.distanceImageResult_Z[i];
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	// ----------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.05-----
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);

	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = customColourVis(cloud);

	while (!viewer->wasStopped())
	{
		refreshCam(m_argc, m_argv);
  		data = getData();
		// Fill in the cloud data
		cloud->width = data.distanceImageResult_sensorWidth;
		cloud->height = data.distanceImageResult_sensorHeight;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);

		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			cloud->points[i].x = data.distanceImageResult_X[i];
			cloud->points[i].y = data.distanceImageResult_Y[i];
			cloud->points[i].z = data.distanceImageResult_Z[i];
		}
		viewer->updatePointCloud(cloud,"sample cloud");
		viewer->spinOnce(100);
		//Try changing this value for less latency
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return RESULT_OK;
}

int visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr) {
	// ------------------------------------
	// -----Create example point cloud-----
	// ------------------------------------

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	// ----------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.05-----
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);

	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = customColourVis(basic_cloud_ptr);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return RESULT_OK;
}

int visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr) {
	// ------------------------------------
	// -----Create example point cloud-----
	// ------------------------------------

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	// ----------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.05-----
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);

	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbVis(basic_cloud_ptr);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return RESULT_OK;
}

int main(int argc, char** argv)
{
	m_argc = argc;
	m_argv = argv;
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	std::string name;
	std::getline(std::cin, name);

	pcl::io::loadPCDFile(name,*cloud);

	visualize(cloud);*/

	live_visualize();

	return (0);
}
