/* Point Grey Bumblebee2 ROS node using Triclops SDK
 *
 * Author: Matthew Edwards (UC), 2018
 * Author: Alexandre Coninx (ISIR CNRS/UPMC), 17/02/2015
 */

#include <cstdint>
#include <cstdio>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "rectify_image.hpp"



// Inspired from https://github.com/ros-perception/vision_opencv/blob/indigo/cv_bridge/src/cv_bridge.cpp @347-367
void triclopscolorimage_to_imagemsg(const TriclopsColorImage &tci, sensor_msgs::Image &im, unsigned int id) {
    im.header.seq = id;
    im.height = static_cast<uint32_t>(tci.nrows);
    im.width = static_cast<uint32_t>(tci.ncols);
    im.encoding = std::string("bgra8");
    im.is_bigendian = 0;
    im.step = static_cast<uint32_t>(tci.rowinc);
    size_t size = im.step*im.height;
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

        // Fill in camera info
        float f, cx, cy, baseline;
        camera_info_left_.width = 1024;  // Hardcoded for now
        camera_info_left_.height = 768;
        triclopsGetFocalLength(context_, &f);
        camera_info_left_.P[0] = f;
        camera_info_left_.P[5] = f;
        triclopsGetImageCenter(context_, &cy, &cx);
        camera_info_left_.P[2] = cx*camera_info_left_.width;
        camera_info_left_.P[6] = cy*camera_info_left_.height;
        camera_info_left_.P[10] = 1;
        camera_info_right_ = camera_info_left_;
        triclopsGetBaseline(context_, &baseline);
        camera_info_right_.P[3] = -f*baseline;

        pub_left_ = it_.advertiseCamera("camera/left/image_rect_color", 10);
        pub_right_ = it_.advertiseCamera("camera/right/image_rect_color", 10);
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
            triclopscolorimage_to_imagemsg(left_image, left_message, frame_index_);
            camera_info_left_.header.seq = frame_index_;
            pub_left_.publish(left_message, camera_info_left_, t);
        }
        if (pub_right_.getNumSubscribers() > 0) {
            triclopscolorimage_to_imagemsg(right_image, right_message, frame_index_);
            camera_info_right_.header.seq = frame_index_;
            pub_right_.publish(right_message, camera_info_right_, t);
        }

    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::CameraPublisher pub_left_, pub_right_;
    sensor_msgs::CameraInfo camera_info_left_, camera_info_right_;

    FC2::Camera camera_;
    TriclopsContext context_ = nullptr;
    unsigned int frame_index_ = 0;
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
