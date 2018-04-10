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
#define _HANDLE_TRICLOPS_ERROR(function, error) \
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

#define _HANDLE_FC2_ERROR(function, error) \
{ \
   if( error != FlyCapture2::PGRERROR_OK ) \
   { \
      printf( \
     "ERROR: %s reported %s.\n", \
     function, \
     error.GetDescription() ); \
      exit( 1 ); \
   } \
} \

#define _HANDLE_FC2T_ERROR(function, error) \
{ \
   if( error != Fc2Triclops::ErrorType::ERRORTYPE_OK ) \
   { \
      printf( \
     "ERROR: %s reported %s.\n", \
     function, \
     "FlyCapture 2 bridge error" ); \
      exit( 1 ); \
   } \
} \

//
// aliases namespaces
//
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// struct containing image needed for processing
struct ImageContainer {
    FC2::Image tmp[2];
    FC2::Image unprocessed[2];
};

enum IMAGE_SIDE {
    RIGHT = 0, LEFT
};

// configure the connected camera
int configureCamera(FC2::Camera &camera);

// generate triclops context from connected camera
int generateTriclopsContext(FC2::Camera &camera,
                            TriclopsContext &triclops);


int process_and_rectify_image(TriclopsContext &context, const FC2::Image &initialimage,
                              TriclopsColorImage &out_left, TriclopsColorImage &out_right);

#endif
