#include <pcl/io/openni_grabber.h>

// --> PCLViewer <--
#include <pcl/visualization/pcl_visualizer.h>

// --> normales <--
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

// --> removenannormals <--
#include <pcl/filters/filter.h>

// --> filter passthrough <--
#include <pcl/filters/passthrough.h>


class SimpleOpenNIViewer
{
	public:
	SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {
		viewer.initCameraParameters ();
		
	}

	// --> clase que llama run() y que pasa la 'cloud' obtenida del openni_grabber <--
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_ni)
	{	
		
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud_ni));
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		pcl::PassThrough<pcl::PointXYZRGBA> pass;
		pass.setInputCloud(cloud_ptr);
		pass.setFilterFieldName("z");
		// distancias --> 50,105 cm
		pass.setFilterLimits(0.5, 1.0);
		//pass.setFilterLimitsNegative(true);
		pass.filter(*cloud);		

		if (!viewer.wasStopped())
		{	
			viewer.addPointCloud(cloud);
			viewer.updatePointCloud(cloud);
			viewer.spinOnce(100);
			viewer.removePointCloud();
		}
		
		 
	}

	void run ()
	{
		pcl::Grabber* interface = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
		
		interface->registerCallback (f);

		interface->start ();

		while (!viewer.wasStopped())
		{
		 	boost::this_thread::sleep (boost::posix_time::seconds (1));
		}

		interface->stop ();
	}

	// OpenMP mejora
	pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne;
	pcl::visualization::PCLVisualizer viewer;
	
	bool inicializacion;

};

int main ()
{
	SimpleOpenNIViewer v;
	v.run ();
	return 0;
}
