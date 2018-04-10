# bumblebee2_driver

ROS driver for the PointGrey BumbleBee2 stereoscopic camera, performing rectification using the PointGrey proprietary software.

I've forked the original version ([alexendy/ros_bb2_pg_driver](https://github.com/alexendy/ros_bb2_pg_driver)), updated it to work with the latest version of the Point Grey libraries, and tidied it up a bit.
The rest of this readme is largely unchanged.



## What is this ?

This is a ROS node that:
- Grabs images from a [PointGrey BumbleBee2 stereoscopic camera](https://www.ptgrey.com/bumblebee2-firewire-stereo-vision-camera-systems) using the PointGrey proprietary FlyCap2 library
- Performs rectification using the PointGrey proprietary Triclops library
- Publishes the rectified images in ROS


### How is that different from other PointGrey ROS drivers ?

You can also drive a PointGrey FireWire camera (stereoscopic or not) using the [ROS official pointgrey\_camera\_driver](http://wiki.ros.org/pointgrey_camera_driver), or even the vanilla [camera1394](http://wiki.ros.org/camera1394) driver.
However, **those drivers will just get the raw images**.
You will have to calibrate the camera and rely on the ROS \[stereo\_\]image\_proc node to process the data.

This is a pity since the point of buying those expensive PointGrey stereo cameras is that they are precalibrated and come with very accurate rectification tools.

This driver directly uses the PointGrey proprietary libraries to grab and rectify the images and gives you the rectified images without relying on stereo\_image\_proc.
It uses the camera's factory calibration parameter and does not require any manual calibration.

(The driver does not do any stereo matching or disparity computation.
You will have to plug you own node to do that, or improve that module to do the disparity computation using Triclops.)

### Compatible cameras

Right now it has only been tested on a [BB2-03S2C-25](https://www.ptgrey.com/bumblebee2-stereo-vison-03-mp-color-firewire-1394a-25mm-sony-icx424-camera) (Color, 0.3MP, 2.5mm focal length).

It should work out of the box for all color BB2 (at a reduced resolution for the 0.8MP models).

Making it work for the mono cameras and the XB3 would require some minor improvements and code adaptation (see below).


## Instructions

### Dependancies

* ROS: std_msgs, sensory_msgs, image_transport
* FlyCap2 SDK and Triclops SDK (available from the PointGrey web site after creating an account)

### Build instructions

* Clone into the `src/` directory of your catkin workspace
* `catkin build` and `catkin install`

### Use

* You can run the node with the provided launchfile: `roslaunch bumblebee2_driver bumblebee2_driver`
* The node publishes the left and right rectified images on topics `/camera/[left,right]/image_rect_colorg_driver/left/rectified`


## Limitations

I am a beginner at ROS and the present code is little more than a quick hack suited to my specific needs; I only share it in the hope it can be useful to someone else.
To the best of my knowledge no other similar free software is available to do that, and the existing documentation and example code for the Triclops / FlyCap2 APIs is rather limited.

Known limitations and problems:

1. Everything is hardcoded:
   * Resolution is hardcoded to 1024x768
   * It is designed for color cameras and will likely not work with grayscale ones
   * It uses the camera default calibration
   * It always uses the first camera (if several are plugged)
   * Paths to triclops and flycap2 libraries are hardcoded
   * Node names, etc. are hardcoded

2. There is an issue with the timestamps of the images grabbed through FlyCap2, it seems to be always set to 0.
   So the timestamps of the Image messages are set to the time when the ROS message are crafted and sent, it is not really the timestamp of the physical image.

3. The processing is asynchronous and driven by a callback called by the FlyCap2 library.
   There is no rate control, no buffering, and no guarantee that no frame will be dropped.

4. It only publishes the rectified images.
   It does not publish the raw images, and it does not do any disparity computation.

Most of those issues are easy to fix (look at the Triclops examples and API if needed).
I don't have the time to take care of it now, but if you want to improve the code you are most welcome.

