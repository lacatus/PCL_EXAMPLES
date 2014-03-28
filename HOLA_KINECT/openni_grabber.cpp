#include <pcl/io/openni_grabber.h>

// --> PCLViewer <--
#include <pcl/visualization/pcl_visualizer.h>

// --> CloudViewer <--
// #include <pcl/visualization/cloud_viewer.h>

// --> normales <--
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

// --> mostrar imagenes RGB, declarar clase ImageViewer <--
// #include <pcl/visualization/image_viewer.h>

class SimpleOpenNIViewer
{
	public:
	SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {
		viewer.initCameraParameters ();
		inicializacion = false;
	}

	// --> clase que llama run() y que pasa la 'cloud' obtenida del openni_grabber <--
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{	
		if(inicializacion == false)
		{
			viewer.addPointCloud(cloud,"cloud");
			inicializacion = true;
		}	
		
		if (!viewer.wasStopped())
		{	
			viewer.updatePointCloud(cloud);
			viewer.spinOnce(100);

			// --> mostrar imagen RGB, crear display RGB <--
			// viewer_2.addRGBImage<pcl::PointXYZRGBA> (cloud);

			// --> CloudViewer <--
			//viewer.showCloud (cloud);
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

	pcl::visualization::PCLVisualizer viewer;

	bool inicializacion;
	
	// --> CloudViewer <--
	// pcl::visualization::CloudViewer viewer;

	// --> mostrar imagenes RGB, declarar clase ImageViewer <--
	// pcl::visualization::ImageViewer viewer_2;
};

int main ()
{
SimpleOpenNIViewer v;
v.run ();
return 0;
}
