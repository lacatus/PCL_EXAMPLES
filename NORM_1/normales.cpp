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
		inicializacion = false;
	}

	// --> clase que llama run() y que pasa la 'cloud' obtenida del openni_grabber <--
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{	

		if(inicializacion == false)
		{
			viewer.addPointCloud(cloud,"cloud_RGBA-D");

			ne.setInputCloud (cloud);

			ne.setRadiusSearch (0.03);

			inicializacion = true;
		}else{
			viewer.removePointCloud(); // se carga "cloud", osease cloud_normals
		}
		
		if (!viewer.wasStopped())
		{	
			viewer.updatePointCloud(cloud);

			pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());

			ne.setSearchMethod (tree);

			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_out (new pcl::PointCloud<pcl::Normal>);

			ne.compute (*cloud_normals);

			/*
			// --> eliminar lineas nan <--
			std::vector<int> index;
			pcl::removeNaNNormalsFromPointCloud(*cloud_normals,*cloud_normals_out, index); 

			// remover outliers de las normales
			// probar a recorrer cloud_normals y aquellos indices que sean NAN, ponerlos a 0
			*/

			viewer.addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloud, cloud_normals); // asigna "cloud" a cloud_normals

			viewer.spinOnce(100);
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
