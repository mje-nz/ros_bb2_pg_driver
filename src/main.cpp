// Alexandre Coninx
// ISIR CNRS/UPMC
// 17/02/2015

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <ctime>

#include <sys/stat.h>
#include <errno.h>

#include <sstream>



//TODO:
// - ROS node

#include "rectify_image.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>



typedef struct {
	FC2::Camera * camera;
	TriclopsContext * triclops;
	unsigned long int i;
//	ros::Publisher * pub;
	image_transport::Publisher * pub_left;
	image_transport::Publisher * pub_right;
} cb_data_type_t;

volatile sig_atomic_t done = 0;


// Inspired from https://github.com/ros-perception/vision_opencv/blob/indigo/cv_bridge/src/cv_bridge.cpp @347-367
void triclopspackedimage_to_imagemsg(TriclopsPackedColorImage * tpci, sensor_msgs::Image * im, unsigned long int id, ros::Time ts)
{
	im->header.stamp = ts; //ros::Time::now();
	im->header.seq = id;
	im->height = tpci->nrows;
	im->width = tpci->ncols;
	im->encoding = std::string("bgra8");
	im->is_bigendian = 0;
	im->step = tpci->rowinc;
	size_t size = im->step*im->height;
	im->data.resize(size);
	memcpy((char*)(&im->data[0]), &tpci->data[0], size);
}

void cb_new_image(FC2::Image * img, const void* data)
{
	char leftname[64], rightname[64];
	TriclopsPackedColorImage rectimages[2];
	// Cast context data
	cb_data_type_t * ctxt = (cb_data_type_t *)data;
	//Increment index
	ctxt->i++;
	// Do procesing
	process_and_rectify_image(*ctxt->triclops, *img, rectimages[LEFT], rectimages[RIGHT]);
	// Save
	//triclopsSavePackedColorImage(&rectimages[LEFT], const_cast<char *>(leftname) );
	//triclopsSavePackedColorImage(&rectimages[RIGHT], const_cast<char *>(rightname) );
	
	ros::Time common_ts = ros::Time::now();
	sensor_msgs::Image left_img,right_img;
	triclopspackedimage_to_imagemsg(&rectimages[LEFT], &left_img,ctxt->i, common_ts);
	triclopspackedimage_to_imagemsg(&rectimages[RIGHT], &right_img,ctxt->i, common_ts);
	ctxt->pub_left->publish(left_img);
	ctxt->pub_right->publish(right_img);
	
/*	std::stringstream ss;
	ss << "Image " << ctxt->i << " published";
	std_msgs::String msg;
	msg.data = ss.str();
	ctxt->pub->publish(msg);*/
	ros::spinOnce();
	
//	printf("Saved images %d\n",ctxt->i);
}


void terminate(int signum)
{
	done = 1;
}



int main( int argc, char* argv[])
{
	//Sig handler
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = &terminate;
	sigaction(SIGTERM, &action, NULL);
	sigaction(SIGINT, &action, NULL);



	//ROS stuff
	ros::init(argc, argv, "ros_bb2_pg_driver");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Publisher pub_left = it.advertise("ros_bb2_pg_driver/left/rectified", 10);
	image_transport::Publisher pub_right = it.advertise("ros_bb2_pg_driver/right/rectified", 10);
	
	//ros::Publisher test_pub = n.advertise<std_msgs::String>("ros_bb2_pg_driver/teststring", 10);
	

	//Output folder
	// time_t t = time(NULL);   // get time now
	//struct tm * now = localtime(&t);
	

	// Triclops stuff
	TriclopsContext triclops;
	FC2::Camera camera;

	camera.Connect();

	// configure camera
	if ( configureCamera( camera ) )
	{
		return EXIT_FAILURE;
	}


	// generate the Triclops context 
	if ( generateTriclopsContext( camera, triclops ) )
	{
		return EXIT_FAILURE;
	}

	//camera.SetCallback(&cb_new_image,NULL);


	cb_data_type_t cb_data;
	cb_data.camera = &camera;
	cb_data.triclops = &triclops;
	cb_data.i = 0;
//	cb_data.pub = &test_pub;
	cb_data.pub_left = &pub_left;
	cb_data.pub_right = &pub_right;

	printf("Starting capture, press Ctrl-C to stop\n");
	camera.StartCapture(&cb_new_image,(void*)&cb_data);


	while(!done && ros::ok() );

	printf("Signal caught or ROS died; closing.\n");


	// Stop capture and close the camera
	camera.StopCapture();
	printf("Capture stopped\n");
	
	camera.Disconnect();

	// clean up context
	TriclopsError te;
	te = triclopsDestroyContext( triclops ) ;
	_HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te);
	printf("%lu image pairs processed.\n",cb_data.i);
	return 0;
}


void do_capture_loop()
{
	return;
}


