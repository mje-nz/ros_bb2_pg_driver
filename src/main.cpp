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
void fc2_to_ros_image(const FC2::Image &in, sensor_msgs::Image &out) {
    out.height = in.GetRows();
    out.width = in.GetCols();
    auto pixel_format = in.GetPixelFormat();
    switch (pixel_format) {
        case FC2::PIXEL_FORMAT_MONO8:
            out.encoding = "mono8";
            break;
        case FC2::PIXEL_FORMAT_BGRU:
            out.encoding = "bgra8";
            break;
        default:
            ROS_WARN_ONCE("Unknown pixel format %X", pixel_format);
            out.encoding = "raw";
    }
    out.is_bigendian = 0;
    out.step = in.GetStride();
    size_t size = out.step*out.height;
    out.data.resize(size);
    memcpy((char *) (&out.data[0]), &in.GetData()[0], size);
}


class TriclopsNode {
public:
    TriclopsNode()
      : nh_("camera"), it_(nh_) {
    }

    void init() {
        auto connect_error = camera_.Connect();
        handleError("FlyCapture2::Camera::Connect()", connect_error, __LINE__);

        configureCamera(camera_);
        generateTriclopsContext(camera_, context_);

        // Fill in camera info
        left_camera_info_.header.frame_id = "bumblebee2_optical";
        float f, cx, cy, baseline;
        left_camera_info_.width = 1024;  // Hardcoded for now
        left_camera_info_.height = 768;
        triclopsGetFocalLength(context_, &f);
        left_camera_info_.P[0] = f;
        left_camera_info_.P[5] = f;
        triclopsGetImageCenter(context_, &cy, &cx);
        left_camera_info_.P[2] = cx*left_camera_info_.width;
        left_camera_info_.P[6] = cy*left_camera_info_.height;
        left_camera_info_.P[10] = 1;
        right_camera_info_ = left_camera_info_;
        triclopsGetBaseline(context_, &baseline);
        right_camera_info_.P[3] = -f*baseline;

        // Set up publishers
        left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
        left_color_pub_ = it_.advertise("left/image_color", 1);
        left_mono_pub_ = it_.advertise("left/image_mono", 1);
        left_rect_color_pub_ = it_.advertise("left/image_rect_color", 1);
        left_rect_mono_pub_ = it_.advertise("left/image_rect_mono", 1);
        right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
        right_color_pub_ = it_.advertise("right/image_color", 1);
        right_mono_pub_ = it_.advertise("right/image_mono", 1);
        right_rect_color_pub_ = it_.advertise("right/image_rect_color", 1);
        right_rect_mono_pub_ = it_.advertise("right/image_rect_mono", 1);
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
            printf("%u image pairs processed.\n", frame_index_);
        }

    }

private:

    void triclopsCallback(FC2::Image *raw_image) {
        // Reuse these for each topic
        sensor_msgs::Image message;
        FC2::Image mono;

        ros::Time t = ros::Time::now();
        frame_index_++;
        left_camera_info_.header.stamp = t;
        left_camera_info_.header.seq = frame_index_;
        right_camera_info_.header = left_camera_info_.header;
        message.header = left_camera_info_.header;
        left_info_pub_.publish(left_camera_info_);
        right_info_pub_.publish(right_camera_info_);

        // Publish unrectified image no matter what to make this easier
        FC2::Image left_color, right_color;
        unpack_raw_image(*raw_image, left_color, right_color);
        fc2_to_ros_image(left_color, message);
        left_color_pub_.publish(message);
        fc2_to_ros_image(right_color, message);
        right_color_pub_.publish(message);

        if (left_mono_pub_.getNumSubscribers() > 0) {
            color_to_mono(left_color, mono);
            fc2_to_ros_image(mono, message);
            left_mono_pub_.publish(message);
        }
        if (right_mono_pub_.getNumSubscribers() > 0) {
            color_to_mono(right_color, mono);
            fc2_to_ros_image(mono, message);
            right_mono_pub_.publish(message);
        }

        const bool should_rectify = (
            left_rect_color_pub_.getNumSubscribers() > 0 ||
            right_rect_color_pub_.getNumSubscribers() > 0 ||
            left_rect_mono_pub_.getNumSubscribers() > 0 ||
            right_rect_mono_pub_.getNumSubscribers() > 0
        );
        if (should_rectify) {
            FC2::Image left_rect_color, right_rect_color;
            rectify_color(context_, left_color, right_color, left_rect_color, right_rect_color);
            fc2_to_ros_image(left_rect_color, message);
            left_rect_color_pub_.publish(message);
            fc2_to_ros_image(right_rect_color, message);
            right_rect_color_pub_.publish(message);

            if (left_rect_mono_pub_.getNumSubscribers() > 0) {
                color_to_mono(left_rect_color, mono);
                fc2_to_ros_image(mono, message);
                left_rect_mono_pub_.publish(message);
            }
            if (right_rect_mono_pub_.getNumSubscribers() > 0) {
                color_to_mono(right_rect_color, mono);
                fc2_to_ros_image(mono, message);
                right_rect_mono_pub_.publish(message);
            }

        }
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    ros::Publisher left_info_pub_, right_info_pub_;
    image_transport::Publisher left_color_pub_, left_mono_pub_, left_rect_color_pub_, left_rect_mono_pub_;
    image_transport::Publisher right_color_pub_, right_mono_pub_, right_rect_color_pub_, right_rect_mono_pub_;
    sensor_msgs::CameraInfo left_camera_info_, right_camera_info_;

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
