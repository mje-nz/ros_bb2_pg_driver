/* Point Grey Bumblebee2 ROS node using Triclops SDK
 *
 * Author: Matthew Edwards (UC), 2018
 * Author: Alexandre Coninx (ISIR CNRS/UPMC), 17/02/2015
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <ctime>

#include <sys/stat.h>
#include <errno.h>

#include <sstream>


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
        auto connect_error = camera_.Connect();
        handleError("FlyCapture2::Camera::Connect()", connect_error, __LINE__);

        configureCamera(camera_);
        generateTriclopsContext(camera_, context_);

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
        if (camera_.IsConnected()) {
            // Stop capture and close the camera
            auto camera_error = camera_.StopCapture();
            handleError("FlyCapture2::Camera::StopCapture()", camera_error, __LINE__);
            printf("Capture stopped\n");

            camera_error = camera_.Disconnect();
            handleError("FlyCapture2::Camera::Disconnect()", camera_error, __LINE__);

            // clean up context
            auto context_error = triclopsDestroyContext(context_);
            handleError("triclopsDestroyContext()", context_error, __LINE__);
            printf("%lu image pairs processed.\n", frame_index_);
        }

    }

private:

    void triclopsCallback(FC2::Image *img) {
        TriclopsColorImage left_image, right_image;
        frame_index_++;

        // Do procesing
        process_and_rectify_image(context_, *img, left_image, right_image);

        // Save
        //triclopsSaveColorImage(&rectified_images[LEFT], "left.png", TriImg_Color_Pixel_Format_RGB);
        //triclopsSaveColorImage(&rectified_images[LEFT], "left.png", TriImg_Color_Pixel_Format_RGB);

        ros::Time t = ros::Time::now();
        sensor_msgs::Image left_message, right_message;
        if (pub_left_.getNumSubscribers() > 0) {
            triclopscolorimage_to_imagemsg(left_image, left_message, frame_index_, t);
            pub_left_.publish(left_message);
        }
        if (pub_right_.getNumSubscribers() > 0) {
            triclopscolorimage_to_imagemsg(right_image, right_message, frame_index_, t);
            pub_right_.publish(right_message);
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
    try {
        node.init();
        node.run();
    } catch(const TriclopsException &e) {
        printf(e.what());
        return 1;
    }

    ros::spin();
    printf("Exiting\n");

    return 0;
}
