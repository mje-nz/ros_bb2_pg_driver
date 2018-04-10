/* Internals
 *
 * Author: Matthew Edwards (UC), 2018
 * Author: Alexandre Coninx (ISIR CNRS/UPMC), 17/02/2015
 */

#ifndef RECTIFY_IMAGE_H
#define RECTIFY_IMAGE_H

#include <stdexcept>

#include "triclops.h"
#include "fc2triclops.h"


/* Error handling code based on Triclops examples */
using TriclopsException = std::runtime_error;

void handleError(const std::string &function_name, TriclopsError status, int lineNumber);

void handleError(const std::string &function_name, const FlyCapture2::Error &error, int lineNumber);

void handleError(const std::string &function_name, const Fc2Triclops::ErrorType &error, int lineNumber);


//
// aliases namespaces
//
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// configure the connected camera
void configureCamera(FC2::Camera &camera);

// generate triclops context from connected camera
void generateTriclopsContext(FC2::Camera &camera, TriclopsContext &context);


void process_and_rectify_image(TriclopsContext &context, const FC2::Image &initialimage,
                              TriclopsColorImage &out_left, TriclopsColorImage &out_right);

#endif
