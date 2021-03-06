#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

bool next_iteration = false;

void printMatix4f(const Eigen::Matrix4f & matrix) {

	printf ("Rotation matrix :\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0,0), matrix (0,1), matrix (0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1,0), matrix (1,1), matrix (1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2,0), matrix (2,1), matrix (2,2));
	printf ("Translation vector :\n");
	printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0,3), matrix (1,3), matrix (2,3));
}

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int
main (int argc, char* argv[])
{
	// The point clouds we will be using
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in_1	(new pcl::PointCloud<pcl::PointXYZRGBA>); // Original point cloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in_2	(new pcl::PointCloud<pcl::PointXYZRGBA>); // Transformed point cloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_icp	(new pcl::PointCloud<pcl::PointXYZRGBA>); // ICP output point cloud

	// Load two pcd files
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("PCD/src_1.pcd", *cloud_in_1) < 0)	{
		PCL_ERROR("Error loading cloud PCD/src_1.pcd");
		return -1;
	}

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("PCD/src_2.pcd", *cloud_in_2) < 0)	{
		PCL_ERROR("Error loading cloud PCD/src_2.pcd");
		return -1;
	}

	
	int iterations = 1;
	/*
	// If the user passed the number of iteration as an argument
	if (argc > 2) {
		iterations = atoi(argv[3]);
	}

	if (iterations < 1) {
		PCL_ERROR("Number of initial iterations must be >= 1\n");
		return -1;
	}
	*/
	printf ("\nLoaded files successfully\n\n");

	pcl::copyPointCloud(*cloud_in_2, *cloud_icp);

	// The Iterative Closest Point algorithm
	std::cout << "Initial iterations number is set to : " << iterations << "\n";
	
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	
	icp.setMaximumIterations(iterations);
	
	icp.setInputSource(cloud_in_1);
	
	icp.setInputTarget(cloud_in_2);
	
	icp.align(cloud_icp);
/*
	icp.setMaximumIterations(1); // For the next time we will call .align() function

	// Defining a rotation matrix and translation vector
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

	if (icp.hasConverged()) {
		printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation();
		printMatix4f(transformation_matrix);
	} else {
		PCL_ERROR ("\nICP has not converged.\n");
		return -1;
	}

	// Visualization
	pcl::visualization::PCLVisualizer viewer ("ICP demo");
	// Create two verticaly separated viewports
	int v1(0);
	viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);


	// The color we will be using
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0-bckgr_gray_level; 

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_in_1_color_h (cloud_in_1, (int)255* txt_gray_lvl, (int)255* txt_gray_lvl, (int)255* txt_gray_lvl);
	viewer.addPointCloud (cloud_in_1, cloud_in_1_color_h, "cloud_in_1_v1", v1);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_in_2_color_h (cloud_in_2, 20, 180, 20);
	viewer.addPointCloud (cloud_in_2, cloud_in_2_color_h, "cloud_in_2_v1", v1);

	// Adding text descriptions in each viewport
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);

	std::stringstream ss; ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v1);

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);

	// Set camera position and orientation
	//viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	//viewer.setSize(1280, 1024); // Visualiser window size

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);

	// Display the visualiser
	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
		
		// The user pressed "space" :
		if (next_iteration) {
			icp.align(*cloud_in_1);

			if (icp.hasConverged()) {
				printf("\033[11A"); // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation();	// This is not very accurate !
				printMatix4f(transformation_matrix);					// Print the transformation between original pose and current pose

				ss.str (""); ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud (cloud_in_1, cloud_in_1_color_h, "cloud_icp_v2");
			} else {
				PCL_ERROR ("\nICP has not converged.\n");
				return -1;
			}
		}
		next_iteration = false;
	}
*/
 	return 0;
}
