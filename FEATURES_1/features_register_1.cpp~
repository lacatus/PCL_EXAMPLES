// PCL INCLUDES
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/pfh.h>

#include <pcl/filters/filter.h>

#include <pcl/keypoints/narf_keypoint.h>

#include <pcl/io/pcd_io.h>

#include <pcl/range_image/range_image.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/point_types.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>


pcl::PointCloud<pcl::Normal>::Ptr estimateNormalsFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
{
	pcl::PointCloud<pcl::Normal>::Ptr norm_res (new pcl::PointCloud<pcl::Normal>); 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>()); 
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne; 
	
	ne.setInputCloud (cloud_input); 
	ne.setSearchMethod (tree_src); 
	ne.setRadiusSearch (0.02); 
	ne.compute(*norm_res);

	return norm_res;
}

pcl::RangeImage createRangeImageFromPointCloud(pcl::PointCloud<pcl::PointXYZ> cloud_to_range)
{
	pcl::RangeImage range_image;
	
	// We now want to create a range image from the above point cloud	
	float angularResolution = (float) (  0.1f * (M_PI/180.0f));  //   1.0 degree in radians
	float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
	float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel=0.00;
	float minRange = 0.0f;
	int borderSize = 1;	

	range_image.createFromPointCloud(cloud_to_range, angularResolution, maxAngleWidth, maxAngleHeight,
		                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	return range_image;
}

pcl::PointCloud<int> extractNARFKeypointsFromRangeImage(pcl::RangeImage range_image)
{
	// Extract NARF keypoints
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector;
	pcl::PointCloud<int> keypoint_indices;

	narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&range_image);
	narf_keypoint_detector.getParameters().support_size = 0.5f;
	narf_keypoint_detector.compute(keypoint_indices);
	
	// optional
	//std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

	return keypoint_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsIndicesToCloudXYZ(
	pcl::PointCloud<int> keypoint_indices, 
	pcl::RangeImage range_image)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;

	keypoints.points.resize (keypoint_indices.points.size ());
	for (size_t i=0; i<keypoint_indices.points.size (); ++i)
		keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();

	return keypoints_ptr;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr keypointsCloudPFHDescriptors(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoint)
{
	pcl::PointCloud<pcl::PFHSignature125>::Ptr signature (new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_est; 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_pfh (new pcl::search::KdTree<pcl::PointXYZ>());

	pfh_est.setSearchMethod (tree_pfh); 
	pfh_est.setRadiusSearch (0.08); 
	pfh_est.setSearchSurface (cloud);   
	pfh_est.setInputNormals (cloud_normal); 
	pfh_est.setInputCloud (cloud_keypoint);  
	pfh_est.compute (*signature); 

	return signature;
}

boost::shared_ptr<pcl::Correspondences> estimateCorrespondencesBetweenPFHDescriptors(
	pcl::PointCloud<pcl::PFHSignature125>::Ptr desc_pfh_1, 
	pcl::PointCloud<pcl::PFHSignature125>::Ptr desc_pfh_2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoint_1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoint_2)
{
	boost::shared_ptr<pcl::Correspondences> cor_all (new pcl::Correspondences);
	boost::shared_ptr<pcl::Correspondences> cor_inliers (new pcl::Correspondences);

	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> corEst; 
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac; 

	corEst.setInputSource (desc_pfh_1); 
	corEst.setInputTarget (desc_pfh_2); 	
	corEst.determineReciprocalCorrespondences (*cor_all); 

	sac.setInputSource (cloud_keypoint_1); 
	sac.setInputTarget (cloud_keypoint_2); 
	sac.setInlierThreshold (0.1); 
	sac.setMaximumIterations (1500); 
	sac.setInputCorrespondences (cor_all);	
	sac.getCorrespondences (*cor_inliers); 

	return cor_inliers;
}

Eigen::Matrix4f rigidTransformationSVD (
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints_1, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints_2, 
        boost::shared_ptr<pcl::Correspondences> cor_inliers) 
        { 

        Eigen::Matrix4f transform; 
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformSVD;
 
        transformSVD.estimateRigidTransformation (*cloud_keypoints_1, *cloud_keypoints_2, *cor_inliers, transform); 

        return transform; 
} 

// ###############################################################################
// ############################		MAIN	      ############################
// ###############################################################################

int main (int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_1 (new pcl::PointCloud<pcl::PointXYZ>); // original_1		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_2 (new pcl::PointCloud<pcl::PointXYZ>); // original_2

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst_1 (new pcl::PointCloud<pcl::PointXYZ>); // original_1 --> dst		

	PCL_INFO("\nLoading files ...\n");

	// Load cloud_input
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_input_1) < 0)	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return -1;
	}
	// Load cloud_input
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud_input_2) < 0)	{
		PCL_ERROR("Error loading cloud %s.\n", argv[2]);
		return -1;
	}
	
	// PTR a VARIABLE
	pcl::PointCloud<pcl::PointXYZ> cloud_1(*cloud_input_1);
	pcl::PointCloud<pcl::PointXYZ> cloud_2(*cloud_input_2);

	PCL_INFO("\nComputing normals ...\n");

	// COMPUTE NORMALS
	pcl::PointCloud<pcl::Normal>::Ptr norm_1 =  estimateNormalsFromPointCloud(cloud_input_1);
	pcl::PointCloud<pcl::Normal>::Ptr norm_2 =  estimateNormalsFromPointCloud(cloud_input_2);
	
	PCL_INFO("\nCreating range images ...\n");	

	// Create range image
	pcl::RangeImage range_image_1 = createRangeImageFromPointCloud(cloud_1);
	pcl::RangeImage range_image_2 = createRangeImageFromPointCloud(cloud_2);
	
	PCL_INFO("\nExtracting NARF keypoints ...\n");

	// Extract NARF keypoints
	pcl::PointCloud<int> keypoints_indices_1 = extractNARFKeypointsFromRangeImage(range_image_1);
	pcl::PointCloud<int> keypoints_indices_2 = extractNARFKeypointsFromRangeImage(range_image_2);

	PCL_INFO("\nNARF keypoints to PointCloudXYZ ...\n");

	// NARF Keypoints to XYZCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints_1 = keypointsIndicesToCloudXYZ(keypoints_indices_1, range_image_1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints_2 = keypointsIndicesToCloudXYZ(keypoints_indices_2, range_image_2);

	PCL_INFO("\nExtracting keypoints PFH descriptors ...\n");

	// PFH NARF keypoints descriptors
	pcl::PointCloud<pcl::PFHSignature125>::Ptr desc_1 = keypointsCloudPFHDescriptors(cloud_input_1,norm_1,cloud_keypoints_1);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr desc_2 = keypointsCloudPFHDescriptors(cloud_input_2,norm_2,cloud_keypoints_2);

	PCL_INFO("\nEstimating and rejecting correspondencies between PFH descriptors ...\n");

	// Correspondecies between PFH descriptors
	boost::shared_ptr<pcl::Correspondences> corresp = 
		estimateCorrespondencesBetweenPFHDescriptors(desc_1,desc_2,cloud_keypoints_1,cloud_keypoints_2);

	PCL_INFO("\nEstimating transformation ...\n");

	// SVD transformation estimation
	Eigen::Matrix4f transf_matrix = rigidTransformationSVD(cloud_keypoints_1,cloud_keypoints_2,corresp);

	PCL_INFO("\nApplying transformation ...\n");

	// transformation
	pcl::transformPointCloud (*cloud_input_1, *cloud_dst_1, transf_matrix); 

	PCL_INFO("\nVisualizing results ...\n");

	// VISUALIZATION 
	// pcl visualizer
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");	
	
	// Color handler properties --> cambia colores de las nubes de puntos	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_1_color_handler (cloud_dst_1, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_2_color_handler (cloud_input_2, 255, 0, 0);

	viewer.addPointCloud<pcl::PointXYZ> (cloud_dst_1, cloud_1_color_handler, "cloud_1");
	viewer.addPointCloud<pcl::PointXYZ> (cloud_input_2, cloud_2_color_handler, "cloud_2");
	//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
	
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	
	PCL_INFO("\n\n");	

	return 0;
	
}

