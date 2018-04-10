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


// Inspired from https://github.com/ros-perception/vision_opencv/blob/indigo/cv_bridge/src/cv_bridge.cpp @347-367
void triclopscolorimage_to_imagemsg(const TriclopsColorImage &tci, sensor_msgs::Image &im, unsigned long int id,
                                    ros::Time t) {
    im.header.stamp = t;
    im.header.seq = id;
    im.height = tci.nrows;
    im.width = tci.ncols;
    im.encoding = std::string("rgba8");
    im.is_bigendian = 0;
    im.step = tci.rowinc;
    size_t size = im.step * im.height;
    im.data.resize(size);
    memcpy((char *) (&im.data[0]), &tci.data[0], size);
}


class TriclopsNode {
public:
    TriclopsNode()
      : it_(nh_) {
    }

    void init() {
        camera_.Connect();

        // configure camera
        if (configureCamera(camera_)) {
            //return EXIT_FAILURE;
        }


        // generate the Triclops context
        if (generateTriclopsContext(camera_, context_)) {
            //return EXIT_FAILURE;
        }

        float f, cx, cy, baseline;
        triclopsGetFocalLength(context_, &f);
        triclopsGetImageCenter(context_, &cy, &cx);
        //TODO: I reckon the correct principal point is (cx*width, cy*height)
        // My calibration has f=1326.9, cx=503.7, cy=383.2
        printf("Stereo parameters: f=%f cx=%f cy=%f\n", f, cx, cy);


        pub_left_ = it_.advertise("camera/left/image_rect_color", 10);
        pub_right_ = it_.advertise("camera/right/image_rect_color", 10);
    }

    void run() {
        printf("Starting capture, press Ctrl-C to stop\n");
        auto callback = [](FC2::Image *img, const void *data) {
            // Not sure why the callback gets a const void*
            TriclopsNode *this_ = const_cast<TriclopsNode*>(static_cast<const TriclopsNode*>(data));
            this_->triclopsCallback(img);
        };
        camera_.StartCapture(callback, static_cast<void*>(this));
    }

    ~TriclopsNode() {
        // Stop capture and close the camera
        camera_.StopCapture();
        printf("Capture stopped\n");

        camera_.Disconnect();

        // clean up context
        TriclopsError te;
        te = triclopsDestroyContext(context_);
        _HANDLE_TRICLOPS_ERROR("triclopsDestroyContext()", te);
        printf("%lu image pairs processed.\n", frame_index_);
    }

private:

    void triclopsCallback(FC2::Image *img) {
        TriclopsColorImage rectified_images[2];
        frame_index_++;

        // Do procesing
        process_and_rectify_image(context_, *img, rectified_images[LEFT], rectified_images[RIGHT]);

        // Save
        //triclopsSaveColorImage(&rectified_images[LEFT], "left.png", TriImg_Color_Pixel_Format_RGB);
        //triclopsSaveColorImage(&rectified_images[LEFT], "left.png", TriImg_Color_Pixel_Format_RGB);

        ros::Time t = ros::Time::now();
        sensor_msgs::Image left_img, right_img;
        if (pub_left_.getNumSubscribers() > 0) {
            triclopscolorimage_to_imagemsg(rectified_images[LEFT], left_img, frame_index_, t);
            pub_left_.publish(left_img);
        }
        if (pub_right_.getNumSubscribers() > 0) {
            triclopscolorimage_to_imagemsg(rectified_images[RIGHT], right_img, frame_index_, t);
            pub_right_.publish(right_img);
        }

    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    FC2::Camera camera_;
    TriclopsContext context_;
    unsigned long int frame_index_;
    image_transport::Publisher pub_left_;
    image_transport::Publisher pub_right_;
};


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bb2_pg_driver");

    TriclopsNode node;
    node.init();
    node.run();

    ros::spin();
    printf("Exiting\n");

    return 0;
}
