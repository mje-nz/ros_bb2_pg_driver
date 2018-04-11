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


void generateColorStereoInput(TriclopsContext const &context, FC2::Image const &grabbedImage,
                             ImageContainer &imageCont, TriclopsColorStereoPair &stereoPair) {
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError te;

    TriclopsColorImage triclopsImageContainer[2];
    FC2::Image *tmpImage = imageCont.tmp;
    FC2::Image *unprocessedImage = imageCont.unprocessed;

    // Convert the pixel interleaved raw data to de-interleaved and color processed data
    const bool is_little_endian = true;  /*assume little endian*/
    auto unpack_error = FC2T::unpackUnprocessedRawOrMono16Image(grabbedImage, is_little_endian,
                                                                tmpImage[RIGHT], tmpImage[LEFT]);
    handleError("Fc2Triclops::unpackUnprocessedRawOrMono16Image()", unpack_error, __LINE__);

    // preprocess color image
    for (int i = 0; i < 2; ++i) {
        auto error = tmpImage[i].SetColorProcessing(FC2::HQ_LINEAR);
        handleError("FlyCapture2::Image::SetColorProcessing()", error, __LINE__);

        // convert preprocessed color image to BGRU format
        error = tmpImage[i].Convert(FC2::PIXEL_FORMAT_BGRU, &unprocessedImage[i]);
        handleError("FlyCapture2::Image::Convert()", error, __LINE__);
    }

    // create triclops image for right and left lens
    for (size_t i = 0; i < 2; ++i) {
        TriclopsColorImage *image = &triclopsImageContainer[i];
        auto error = triclopsLoadColorImageFromBuffer(
                reinterpret_cast<TriclopsColorPixel *>(unprocessedImage[i].GetData()),
                unprocessedImage[i].GetRows(), unprocessedImage[i].GetCols(), unprocessedImage[i].GetStride(),
                image);
        handleError("triclopsLoadColorImageFromBuffer()", error, __LINE__);
    }

    // create stereo input from the triclops images constructed above
    // pack image data into a TriclopsColorStereoPair structure
    auto pair_error = triclopsBuildColorStereoPairFromBuffers(context, &triclopsImageContainer[RIGHT],
                                                              &triclopsImageContainer[LEFT], &stereoPair);
    handleError("triclopsBuildColorStereoPairFromBuffers()", pair_error, __LINE__);
}

void doRectification(const TriclopsContext &context, TriclopsColorStereoPair *colorStereoInput,
                    TriclopsColorImage &leftImage, TriclopsColorImage &rightImage) {
    auto error = triclopsColorRectify(context, colorStereoInput);
    handleError("triclopsColorRectify()", error, __LINE__);
    error = triclopsGetColorImage(context, TriImg_RECTIFIED_COLOR, TriCam_LEFT, &leftImage);
    handleError("triclopsGetImage()", error, __LINE__);
    error = triclopsGetColorImage(context, TriImg_RECTIFIED_COLOR, TriCam_RIGHT, &rightImage);
    handleError("triclopsSaveColorImage()", error, __LINE__);
}


void process_and_rectify_image(TriclopsContext &context, const FC2::Image &initialimage,
                              TriclopsColorImage &out_left, TriclopsColorImage &out_right) {
    // declare container of images used for processing
    ImageContainer imageContainer;
    TriclopsColorStereoPair colorStereoInput;

    // generate color triclops input from grabbed image
    generateColorStereoInput(context, initialimage, imageContainer, colorStereoInput);

    // carry out rectification from triclops color input
    doRectification(context, &colorStereoInput, out_left, out_right);
}