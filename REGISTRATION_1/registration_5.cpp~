#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

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
	// Cloud 1
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in 	(new pcl::PointCloud<pcl::PointXYZRGBA>); 
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input	(new pcl::PointCloud<pcl::PointXYZRGBA>); 

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output; 

	// Cloud 2
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in_2	(new pcl::PointCloud<pcl::PointXYZRGBA>); 
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input_2	(new pcl::PointCloud<pcl::PointXYZRGBA>); 

	// Cloud normals
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in_with_normals   (new pcl::PointCloud<pcl::PointNormal>);
  	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in_2_with_normals (new pcl::PointCloud<pcl::PointNormal>);


	// Checking program arguments
	if (argc < 3) {
		printf ("Usage :\n");
		printf ("\t\t%s file.pcd number_of_ICP_iterations\n", argv[0]);
		PCL_ERROR("Provide two pcd file.\n");
		return -1;
	}
	
	// Load cloud_1
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud_input) < 0)	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return -1;
	}

	// Load cloud_2
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[2], *cloud_input_2) < 0)	{
		PCL_ERROR("Error loading cloud %s.\n", argv[2]);
		return -1;
	}

	// Initial iterations (Optional)
	int iterations = 1;
	// If the user passed the number of iteration as an argument
	if (argc > 3) {
		iterations = atoi(argv[3]);
	}

	if (iterations < 1) {
		PCL_ERROR("Number of initial iterations must be >= 1\n");
		return -1;
	}

	printf ("\nLoaded file %s with %d points successfully", argv[1], (int)cloud_input->size());
	printf ("\nLoaded file %s with %d points successfully\n", argv[2], (int)cloud_input_2->size());

	// Remove NAN points from the cloud
      	std::vector<int> indices, indices_2;
	pcl::removeNaNFromPointCloud(*cloud_input,*cloud_input, indices); // cloud_1
	pcl::removeNaNFromPointCloud(*cloud_input_2,*cloud_input_2, indices); // cloud_2

	// Downsample Cloud points
	// cloud_1
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1);
	approximate_voxel_filter.setInputCloud (cloud_input);
	approximate_voxel_filter.filter (*cloud_in);
	// cloud_2
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> approximate_voxel_filter_2;
	approximate_voxel_filter_2.setLeafSize (0.1, 0.1, 0.1);
	approximate_voxel_filter_2.setInputCloud (cloud_input_2);
	approximate_voxel_filter_2.filter (*cloud_in_2);

	printf ("\nReduced file %s to %d points successfully", argv[1], (int)cloud_in->size());
	printf ("\nReduced file %s to %d points successfully\n\n", argv[2], (int)cloud_in_2->size());

	// NORMAL ESTIMATION
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	norm_est.setRadiusSearch(0.03); 
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (30);

	norm_est.setInputCloud (cloud_in);
	norm_est.compute (*cloud_in_with_normals);
	//pcl::copyPointCloud (*cloud_in, *cloud_in_with_normals);

	norm_est.setInputCloud (cloud_in_2);
	norm_est.compute (*cloud_in_2_with_normals);
	//pcl::copyPointCloud (*cloud_in_2, *cloud_in_2_with_normals);	

	// FPFH
	pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::PointNormal, pcl::FPFHSignature33> fpfh_1, fpfh_2;

	fpfh_1.setInputCloud (cloud_in);
  	fpfh_1.setInputNormals (cloud_in_with_normals);

	fpfh_2.setInputCloud (cloud_in_2);
  	fpfh_2.setInputNormals (cloud_in_2_with_normals);

	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_2 (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	fpfh_1.setSearchMethod (tree_2);
	fpfh_2.setSearchMethod (tree_2);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_1 (new pcl::PointCloud<pcl::FPFHSignature33> ());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_2 (new pcl::PointCloud<pcl::FPFHSignature33> ());

	fpfh_1.setRadiusSearch (0.08);
	fpfh_2.setRadiusSearch (0.08);

  	// Compute the features
  	fpfh_1.compute (*fpfhs_1);
	fpfh_2.compute (*fpfhs_2);

	// Defining a rotation matrix and translation vector
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	
	/*
	// The Iterative Closest Point algorithm
	std::cout << "Initial iterations number is set to : " << iterations << "\n";
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_in_2_with_normals);
	icp.setInputTarget(cloud_in_with_normals);
	*/


	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::FPFHSignature33> sac_ia; 
	Eigen::Matrix4f final_transformation;	
	sac_ia.setInputSource( cloud_in_2 ); 
	sac_ia.setSourceFeatures( fpfhs_2 ); 
	sac_ia.setInputTarget( cloud_in ); 
	sac_ia.setTargetFeatures( fpfhs_1 ); 
	sac_ia.setMaximumIterations( 500 ); 	
	
	
	// Visualization
	pcl::visualization::PCLVisualizer viewer ("Registration ICP");

	// The color we will be using
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0-bckgr_gray_level; 

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h (cloud_in_with_normals, (int)255* txt_gray_lvl, (int)255* txt_gray_lvl, (int)255* txt_gray_lvl);
	viewer.addPointCloud (cloud_in_with_normals, cloud_in_color_h, "cloud_in");
	
	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_2_color_h (cloud_in_2_with_normals, 180, 20, 20);
	viewer.addPointCloud (cloud_in_2_with_normals, cloud_in_2_color_h, "cloud_in_2");

	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024); // Visualiser window size

	// Register keyboard callback :
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
	
	while(!viewer.wasStopped ())
	{
		viewer.spinOnce ();
		// The user pressed "space" :
		if (next_iteration) {
			//sac_ia.align( cloud_in_2 ); 
			sac_ia.align(output);
		}
		next_iteration = false;

	}

	return 0;
}
