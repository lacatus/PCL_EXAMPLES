// PCL
#include <pcl/point_types.h>

#include <pcl/range_image/range_image.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>

#include <pcl/filters/filter.h>

#include <pcl/keypoints/narf_keypoint.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char* argv[])
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input	(new pcl::PointCloud<pcl::PointXYZ>); // original
		

	// Load cloud_input
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_input) < 0)	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return -1;
	}

	// Remove NAN points from the cloud
      	//std::vector<int> indices;
	//pcl::removeNaNFromPointCloud(*cloud_input,*cloud_input, indices); 

	pcl::PointCloud<pcl::PointXYZ> cloud(*cloud_input);
	 
	// We now want to create a range image from the above point cloud	
	float angularResolution = (float) (  0.1f * (M_PI/180.0f));  //   1.0 degree in radians
	float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
	float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel=0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	// Create range image
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	// Extract NARF keypoints
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector;
	narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage (&rangeImage);
	narf_keypoint_detector.getParameters ().support_size = 0.2f;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute (keypoint_indices);
	std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

	// VISUALIZATION 
	// range image
	pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
	range_image_widget.showRangeImage (rangeImage);

	// pcl visualizer
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.addPointCloud (cloud_input,"cloud");

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;

	keypoints.points.resize (keypoint_indices.points.size ());
	for (size_t i=0; i<keypoint_indices.points.size (); ++i)
		keypoints.points[i].getVector3fMap () = rangeImage.points[keypoint_indices.points[i]].getVector3fMap ();

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	
	while (!range_image_widget.wasStopped())
	{
		range_image_widget.spinOnce();  // process GUI events
		viewer.spinOnce();
	}
	return 0;
	
}

