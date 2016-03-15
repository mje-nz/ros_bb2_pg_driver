// Alexandre Coninx
// ISIR CNRS/UPMC
// 17/02/2015

#ifndef RECTIFY_IMAGE_H
#define RECTIFY_IMAGE_H 

#include "triclops.h"
#include "fc2triclops.h"


//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

//
// aliases namespaces
//
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// struct containing image needed for processing
struct ImageContainer
{
    FC2::Image unprocessed[2];
    FC2::Image bgru[2];
//    FC2::Image packed;
} ;

enum IMAGE_SIDE
{
	RIGHT = 0, LEFT
};

// configure the connected camera
int configureCamera( FC2::Camera & camera );

// generate triclops context from connected camera
int generateTriclopsContext( FC2::Camera & camera, 
                         TriclopsContext & triclops );

// capture image from connected camera
int grabImage ( FC2::Camera & camera, FC2::Image& rGrabbedImage );

// generate triclops input from grabbed color image 
int generateTriclopsInput( const FC2::Image & grabbedImage, 
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInputRight, TriclopsInput   & triclopsColorInputLeft ); 

// carry out rectification from triclops color input
int doRectification( const TriclopsContext & triclops, 
                     const TriclopsInput & colorTriclopsInput,
                     TriclopsCamera tricam,
                     TriclopsPackedColorImage & rectifiedPackedColorImage
                     );


int process_and_rectify_image(TriclopsContext & triclops, const FC2::Image & initialimage, TriclopsPackedColorImage & out_left, TriclopsPackedColorImage & out_right);

#endif
