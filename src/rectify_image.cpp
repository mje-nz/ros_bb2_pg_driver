// Alexandre Coninx
// ISIR CNRS/UPMC
// 17/02/2015


// based on grabrectifiedcolor.cpp by Point Grey Research


//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>

#include "rectify_image.hpp"


int configureCamera(FC2::Camera &camera) {

    FC2T::ErrorType fc2TriclopsError;
    FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    fc2TriclopsError = FC2T::setStereoMode(camera, mode);
    if (fc2TriclopsError) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }

    return 0;
}

int generateTriclopsContext(FC2::Camera &camera,
                            TriclopsContext &triclops) {
    FC2::CameraInfo camInfo;
    FC2::Error fc2Error = camera.GetCameraInfo(&camInfo);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    FC2T::ErrorType fc2TriclopsError;
    fc2TriclopsError = FC2T::getContextFromCamera(camInfo.serialNumber,
                                                  &triclops);
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK) {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
                                            "getContextFromCamera");
    }

    //Set resolution to 1024x768
    TriclopsError te;
    te = triclopsSetResolution(triclops, 768, 1024);
    _HANDLE_TRICLOPS_ERROR("triclopsSetResolution()", te);

    return 0;
}

int grabImage(FC2::Camera &camera, FC2::Image &grabbedImage) {
    FC2::Error fc2Error = camera.StartCapture();
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = camera.RetrieveBuffer(&grabbedImage);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

int convertToBGRU(FC2::Image &image, FC2::Image &convertedImage) {
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGRU, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK) {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

int generateColorStereoInput(TriclopsContext const &context,
                             FC2::Image const &grabbedImage,
                             ImageContainer &imageCont,
                             TriclopsColorStereoPair &stereoPair) {
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError te;

    TriclopsColorImage triclopsImageContainer[2];
    FC2::Image *tmpImage = imageCont.tmp;
    FC2::Image *unprocessedImage = imageCont.unprocessed;

    // Convert the pixel interleaved raw data to de-interleaved and color processed data
    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
            grabbedImage,
            true /*assume little endian*/,
            tmpImage[RIGHT],
            tmpImage[LEFT]);

    _HANDLE_FC2T_ERROR("Fc2Triclops::unpackUnprocessedRawOrMono16Image()", fc2TriclopsError);

    // preprocess color image
    for (int i = 0; i < 2; ++i) {
        FC2::Error fc2Error;
        fc2Error = tmpImage[i].SetColorProcessing(FC2::HQ_LINEAR);
        //_HANDLE_TRICLOPS_ERROR("FlyCapture2::Image::SetColorProcessing()", fc2Error);

        // convert preprocessed color image to BGRU format
        fc2Error = tmpImage[i].Convert(FC2::PIXEL_FORMAT_BGRU,
                                       &unprocessedImage[i]);
        _HANDLE_FC2_ERROR("FlyCapture2::Image::Convert()", fc2Error);
    }

    // create triclops image for right and left lens
    for (size_t i = 0; i < 2; ++i) {
        TriclopsColorImage *image = &triclopsImageContainer[i];
        te = triclopsLoadColorImageFromBuffer(
                reinterpret_cast<TriclopsColorPixel *>(unprocessedImage[i].GetData()),
                unprocessedImage[i].GetRows(),
                unprocessedImage[i].GetCols(),
                unprocessedImage[i].GetStride(),
                image);
        _HANDLE_TRICLOPS_ERROR("triclopsLoadColorImageFromBuffer()", te);
    }

    // create stereo input from the triclops images constructed above
    // pack image data into a TriclopsColorStereoPair structure
    te = triclopsBuildColorStereoPairFromBuffers(
            context,
            &triclopsImageContainer[RIGHT],
            &triclopsImageContainer[LEFT],
            &stereoPair);
    _HANDLE_TRICLOPS_ERROR("triclopsBuildColorStereoPairFromBuffers()", te);
    return 0;
}

int doRectification(const TriclopsContext &context, TriclopsColorStereoPair *colorStereoInput,
                    TriclopsColorImage &leftImage, TriclopsColorImage &rightImage) {
    TriclopsError te;
    te = triclopsColorRectify(context, colorStereoInput);
    _HANDLE_TRICLOPS_ERROR("triclopsColorRectify()", te);
    te = triclopsGetColorImage(context, TriImg_RECTIFIED_COLOR, TriCam_LEFT, &leftImage);
    _HANDLE_TRICLOPS_ERROR("triclopsGetImage()", te);
    te = triclopsGetColorImage(context, TriImg_RECTIFIED_COLOR, TriCam_RIGHT, &rightImage);
    _HANDLE_TRICLOPS_ERROR("triclopsGetImage()", te);
    return 0;
}


int process_and_rectify_image(TriclopsContext &context, const FC2::Image &initialimage,
                              TriclopsColorImage &out_left, TriclopsColorImage &out_right) {
    // declare container of images used for processing
    ImageContainer imageContainer;
    TriclopsColorStereoPair colorStereoInput;

    // generate color triclops input from grabbed image
    if (generateColorStereoInput(context, initialimage, imageContainer, colorStereoInput)) {
        return EXIT_FAILURE;
    }

    // carry out rectification from triclops color input
    if (doRectification(context, &colorStereoInput, out_left, out_right)) {
        return EXIT_FAILURE;
    }

    return 0;
}