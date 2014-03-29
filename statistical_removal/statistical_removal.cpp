#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;


// Visualization removed noise
	printf("\norig_cloud\n");
	pcl::visualization::PCLVisualizer viewer1 ("orig_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 0, 0, 0); // Where 255,255,255 are R,G,B colors
	viewer1.addPointCloud (cloud, cloud_color_handler, "orig_cloud");	// We add the point cloud to the viewer and pass the color handler

	//viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer1.setBackgroundColor(0.95, 0.95, 0.95, 0); // Setting background to a dark grey
	viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "orig_cloud");
	
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer1.wasStopped ()) { // Display the visualiser untill 'q' key is pressed
		viewer1.spinOnce ();
	}





  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);




// Visualization removed noise
	printf("\nNoise removed\n");
	pcl::visualization::PCLVisualizer viewer ("Noise removed");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_color_handler (cloud_filtered, 0, 0, 0); // Where 255,255,255 are R,G,B colors
	viewer.addPointCloud (cloud_filtered, cloud_filtered_color_handler, "removed_cloud");	// We add the point cloud to the viewer and pass the color handler

	//viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.95, 0.95, 0.95, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "removed_cloud");
	
	//viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped ()) { // Display the visualiser untill 'q' key is pressed
		viewer.spinOnce ();
	}
 //Para generar los outliers
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return (0);
}


