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

void configureCamera(FC2::Camera &camera);
void generateTriclopsContext(FC2::Camera &camera, TriclopsContext &context);


void unpack_raw_image(const FC2::Image &raw, FC2::Image &color_left, FC2::Image &color_right);
void rectify_color(const TriclopsContext &context, FC2::Image &color_left, FC2::Image &color_right,
                   FC2::Image &rect_left, FC2::Image &rect_right);
void color_to_mono(const FC2::Image &in, FC2::Image &out);

#endif
