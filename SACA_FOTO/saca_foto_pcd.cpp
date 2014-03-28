#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>

char key = 0; 

const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// http://en.cppreference.com/w/cpp/chrono/c/strftime
	strftime(buf, sizeof(buf), "%d_%m_%Y_%H_%M_%S", &tstruct);
	return buf;
}


class SimpleOpenNIViewer
{
	public:
		// Keyboard event 
		// c is save the point clound 
		static void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void)	
		{ 
			if (event.getKeySym () == "c" && event.keyDown ()) 
			{ 
				key = 'c'; 
			} 
			if (event.keyUp()) 
			{ 
				key = 0; 
			}
		} 
		
		SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {
			viewer.registerKeyboardCallback(SimpleOpenNIViewer::keyboardEventOccurred,(void*)NULL); 
		}

		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_ni)
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud_ni));
			std::ostringstream oss;

			if (!viewer.wasStopped())
			{
				viewer.showCloud (cloud);
				switch (key) 
		                { 
		                        case 'c': 
						//printf("HI callback \n");					
						//std::cout << "currentDateTime()=" << currentDateTime() << std::endl;
	 					oss << "PCD_files/foto_" << currentDateTime() << ".pcd"; 
						pcl::io::savePCDFileASCII (oss.str(),*cloud);
						boost::this_thread::sleep (boost::posix_time::seconds (1));
		                                break; 
		                        default: 
		                                break; 
		                } 
			}
		}

		void run ()
		{
			pcl::Grabber* interface = new pcl::OpenNIGrabber();

			boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
				boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

			boost::signals2::connection c = interface->registerCallback (f);

			interface->start ();

			while (!viewer.wasStopped())
			{
				boost::this_thread::sleep (boost::posix_time::seconds (1));
			}

			interface->stop ();
		}

	pcl::visualization::CloudViewer viewer;
};

int main ()
{
SimpleOpenNIViewer v;
v.run ();
return 0;
}
