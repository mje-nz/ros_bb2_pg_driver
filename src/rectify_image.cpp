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

int process_and_rectify_image(TriclopsContext & triclops, const FC2::Image & initialimage, TriclopsPackedColorImage & out_left, TriclopsPackedColorImage & out_right)
{
    // declare container of images used for processing
    ImageContainer imageContainer;
    TriclopsInput triclopsColorInputLeft, triclopsColorInputRight;

    // generate color triclops input from grabbed image
    if ( generateTriclopsInput( initialimage, 
        imageContainer,
        triclopsColorInputRight, triclopsColorInputLeft) 
       )
    {
		return EXIT_FAILURE;
    }

    // carry out rectification from triclops color input
    if ( doRectification( triclops, triclopsColorInputLeft, TriCam_LEFT, out_left ) )
    {
		return EXIT_FAILURE;
    }

    if ( doRectification( triclops, triclopsColorInputRight, TriCam_RIGHT, out_right ) )
    {
		return EXIT_FAILURE;
    }
   return 0;
}






int configureCamera( FC2::Camera & camera )
{

	FC2T::ErrorType fc2TriclopsError;	      
	FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    fc2TriclopsError = FC2T::setStereoMode( camera, mode );
    if ( fc2TriclopsError )
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }

    return 0;
}

int generateTriclopsContext( FC2::Camera & camera, 
                              TriclopsContext & triclops )
{
	FC2::CameraInfo camInfo;
	FC2::Error fc2Error = camera.GetCameraInfo(&camInfo);
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}
   
	FC2T::ErrorType fc2TriclopsError; 
    fc2TriclopsError = FC2T::getContextFromCamera( camInfo.serialNumber, 
	                                                &triclops );
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
		                                    "getContextFromCamera");
    }
    
	//Set resolution to 640x480
	TriclopsError te;
	te = triclopsSetResolution( triclops, 480, 640 );
	_HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te );
	
	return 0;
}

int grabImage ( FC2::Camera & camera, FC2::Image& grabbedImage )
{
	FC2::Error fc2Error = camera.StartCapture();
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}

	fc2Error = camera.RetrieveBuffer(&grabbedImage);
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}
	
	return 0;
}

int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage )
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGRU, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

int generateTriclopsInput( const FC2::Image & grabbedImage, 
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInputRight, TriclopsInput   & triclopsColorInputLeft ) 
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError; 

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                                grabbedImage, 
                                true /*assume little endian*/,
                                unprocessedImage[RIGHT], unprocessedImage[LEFT]);

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
                                     "deinterleaveRawOrMono16Image");
    }

/*    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    unprocessedImage[RIGHT].Save("rawRightImage.pgm", &pgmOpt);
    unprocessedImage[LEFT].Save("rawLeftImage.pgm", &pgmOpt);*/

    FC2::Image * bgruImage = imageContainer.bgru;

    for ( int i = 0; i < 2; ++i )
    {
        if ( convertToBGRU(unprocessedImage[i], bgruImage[i]) )
        {
            return 1;
        }
    }

/*    FC2::PNGOption pngOpt;
    pngOpt.interlaced = false;
    pngOpt.compressionLevel = 9;
    bgruImage[RIGHT].Save("colorImageRight.png", &pngOpt);
    bgruImage[LEFT].Save("colorImageLeft.png", &pngOpt);*/


    TriclopsError te;
    te = triclopsBuildPackedTriclopsInput(
                    bgruImage[LEFT].GetCols(),
                    bgruImage[LEFT].GetRows(),
                    bgruImage[LEFT].GetStride(),
                    (unsigned long)bgruImage[LEFT].GetTimeStamp().seconds, 
                    (unsigned long)bgruImage[LEFT].GetTimeStamp().microSeconds, 
                    bgruImage[LEFT].GetData(),
                    &triclopsColorInputLeft );
                    


    _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );

    te = triclopsBuildPackedTriclopsInput(
                    bgruImage[RIGHT].GetCols(),
                    bgruImage[RIGHT].GetRows(),
                    bgruImage[RIGHT].GetStride(),
                    (unsigned long)bgruImage[RIGHT].GetTimeStamp().seconds, 
                    (unsigned long)bgruImage[RIGHT].GetTimeStamp().microSeconds, 
                    bgruImage[RIGHT].GetData(),
                    &triclopsColorInputRight);
                    


    _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );



    return 0;
}

int doRectification( const TriclopsContext & triclops, 
                     const TriclopsInput & colorTriclopsInput,
                     TriclopsCamera tricam,
                     TriclopsPackedColorImage & rectifiedPackedColorImage
                     )
{
    // rectify the color image
    TriclopsError te;
    te = triclopsRectifyPackedColorImage( triclops, 
				   tricam, 
				   const_cast<TriclopsInput *>(&colorTriclopsInput), 
				   &rectifiedPackedColorImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyPackedColorImage()", te );
    

    return 0;
}

