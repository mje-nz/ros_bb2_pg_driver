/* Internals
 *
 * Author: Matthew Edwards (UC), 2018
 * Author: Alexandre Coninx (ISIR CNRS/UPMC), 17/02/2015
 *
 * Originally based on grabrectifiedcolor.cpp by Point Grey Research.
 * 2018 version based on Triclops 4 Grab_Stereo example by PGR.
 *
 */


#include <cstdio>
#include <cstdlib>
#include <string>

#include "rectify_image.hpp"


// struct containing image needed for processing
struct ImageContainer {
    FC2::Image tmp[2];
    FC2::Image unprocessed[2];
};

enum IMAGE_SIDE {
    RIGHT = 0, LEFT
};


void throwTriclopsException(const std::string &function_name, const std::string &error, int line_number) {
    std::string what = "Triclops Error: " + error + " (line " + std::to_string(line_number) + ")\n" + \
                       "Function called: " + function_name + "\n";
    throw TriclopsException(what);
}

void handleError(const std::string &function_name, TriclopsError status, int lineNumber) {
    if (status != TriclopsErrorOk) {
        throwTriclopsException(function_name, triclopsErrorToString(status), lineNumber);
    }
}

void handleError(const std::string &function_name, const FlyCapture2::Error &error, int lineNumber) {
    if (error != FlyCapture2::PGRERROR_OK) {
        throwTriclopsException(function_name, error.GetDescription(), lineNumber);
    }
}

void handleError(const std::string &function_name, const Fc2Triclops::ErrorType &error, int lineNumber) {
    if (error != Fc2Triclops::ErrorType::ERRORTYPE_OK) {
        throwTriclopsException(function_name, "FlyCapture 2 bridge error", lineNumber);
    }
}


void configureCamera(FC2::Camera &camera) {
    auto mode = FC2T::TWO_CAMERA;
    auto mode_error = FC2T::setStereoMode(camera, mode);
    handleError("Fc2Triclops::setStereoMode()", mode_error, __LINE__);

    // Set framerate to 15fps
    auto framerate = FC2::Property{FC2::FRAME_RATE};
    framerate.autoManualMode = false;
    framerate.absControl = false;
    framerate.onOff = true;  // False to enable extended shutter
    framerate.valueA = 15;
    auto camera_error = camera.SetProperty(&framerate);
    handleError("FlyCapture2::Camera::SetProperty(FRAME_RATE)", camera_error, __LINE__);

    // Disable auto gain and set gain to 0
    auto gain = FC2::Property{FC2::GAIN};
    gain.autoManualMode = false;
    gain.absControl = true;
    gain.absValue = 0; camera_error = camera.SetProperty(&gain);
    handleError("FlyCapture2::Camera::SetProperty(GAIN)", camera_error, __LINE__);

    // Disable auto shutter and set shutter to as long as it can go
    auto shutter = FC2::Property{FC2::SHUTTER};
    shutter.autoManualMode = false;
    shutter.absControl = true;
    shutter.absValue = 1000;
    camera_error = camera.SetProperty(&shutter);
    handleError("FlyCapture2::Camera::SetProperty(SHUTTER)", camera_error, __LINE__);

    // Set auto exposure to 0EV
    auto auto_exposure = FC2::Property{FC2::AUTO_EXPOSURE};
    auto_exposure.onOff = true;
    auto_exposure.autoManualMode = false;
    auto_exposure.absControl = true;
    auto_exposure.absValue = 0;
    camera_error = camera.SetProperty(&auto_exposure);
    handleError("FlyCapture2::Camera::SetProperty(AUTO_EXPOSURE)", camera_error, __LINE__);

    // Enable buffered mode to (hopefully) prevent glitches
    auto config = FC2::FC2Config();
    camera.GetConfiguration(&config);
    config.highPerformanceRetrieveBuffer = true;
    config.numBuffers = 10;  // Arbitrary large number to avoid stomping buffers
    config.grabMode = FC2::DROP_FRAMES;  // Use the most recent frame in the buffers
    camera.SetConfiguration(&config);
}


void generateTriclopsContext(FC2::Camera &camera, TriclopsContext &context) {
    FC2::CameraInfo camInfo;
    auto info_error = camera.GetCameraInfo(&camInfo);
    handleError("FlyCapture2::Camera::GetCameraInfo()", info_error, __LINE__);

    auto serial_error = FC2T::getContextFromCamera(camInfo.serialNumber, &context);
    handleError("Fc2Triclops::getContextFromCamera()", serial_error, __LINE__);

    auto resolution_error = triclopsSetResolution(context, 768, 1024);
    handleError("triclopsSetResolution()", resolution_error, __LINE__);
}


void debayer_color_image(FC2::Image &in, FC2::Image &out) {
    auto error = in.SetColorProcessing(FC2::HQ_LINEAR);
    handleError("FlyCapture2::Image::SetColorProcessing()", error, __LINE__);
    error = in.Convert(FC2::PIXEL_FORMAT_BGRU, &out);
    handleError("FlyCapture2::Image::Convert()", error, __LINE__);
}

/** Convert the interleaved bayer image from the camera into two unrectified colour images */
void unpack_raw_image(const FC2::Image &raw, FC2::Image &color_left, FC2::Image &color_right) {
    FC2::Image raw_left, raw_right;
    const bool is_little_endian = true;  /*assume little endian*/
    auto unpack_error = FC2T::unpackUnprocessedRawOrMono16Image(raw, is_little_endian, raw_left, raw_right);
    handleError("Fc2Triclops::unpackUnprocessedRawOrMono16Image()", unpack_error, __LINE__);
    debayer_color_image(raw_left, color_left);
    debayer_color_image(raw_right, color_right);
}


void triclops_to_fc2_color_image(const TriclopsColorImage &in, FC2::Image &out) {
    auto rows = static_cast<unsigned int>(in.nrows);
    auto cols = static_cast<unsigned int>(in.ncols);
    auto stride = static_cast<unsigned int>(in.rowinc);
    auto data = reinterpret_cast<unsigned char*>(in.data);
    auto size = rows*stride;
    out = FC2::Image(rows, cols, stride, data, size, size, FC2::PIXEL_FORMAT_BGRU);
}

void rectify_color(const TriclopsContext &context, FC2::Image &color_left, FC2::Image &color_right,
                   FC2::Image &rect_left, FC2::Image &rect_right) {
    // Convert to TriclopsImages
    TriclopsColorImage triclops_left, triclops_right;
    auto error = triclopsLoadColorImageFromBuffer(
            reinterpret_cast<TriclopsColorPixel *>(color_left.GetData()),
            color_left.GetRows(), color_left.GetCols(), color_left.GetStride(), &triclops_left);
    handleError("triclopsLoadColorImageFromBuffer()", error, __LINE__);
    error = triclopsLoadColorImageFromBuffer(
            reinterpret_cast<TriclopsColorPixel *>(color_right.GetData()),
            color_right.GetRows(), color_right.GetCols(), color_right.GetStride(), &triclops_right);
    handleError("triclopsLoadColorImageFromBuffer()", error, __LINE__);

    // Pack into TriclopsColorStereoPair
    TriclopsColorStereoPair stereoPair;
    error = triclopsBuildColorStereoPairFromBuffers(context, &triclops_right, &triclops_left, &stereoPair);
    handleError("triclopsBuildColorStereoPairFromBuffers()", error, __LINE__);

    // Rectify
    error = triclopsColorRectify(context, &stereoPair);
    handleError("triclopsColorRectify()", error, __LINE__);

    // Get images back out
    TriclopsColorImage triclops_rect_left, triclops_rect_right;
    error = triclopsGetColorImage(context, TriImg_RECTIFIED_COLOR, TriCam_LEFT, &triclops_rect_left);
    handleError("triclopsGetImage()", error, __LINE__);
    error = triclopsGetColorImage(context, TriImg_RECTIFIED_COLOR, TriCam_RIGHT, &triclops_rect_right);
    handleError("triclopsSaveColorImage()", error, __LINE__);

    // Extract FC2 images (note Triclops manages these buffers)
    triclops_to_fc2_color_image(triclops_rect_left, rect_left);
    triclops_to_fc2_color_image(triclops_rect_right, rect_right);
}


void color_to_mono(const FC2::Image &in, FC2::Image &out) {
    in.Convert(FC2::PIXEL_FORMAT_MONO8, &out);
}
