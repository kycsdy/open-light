////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file   Calibration\UtilProCam.h
///
/// @brief  Declares the util pro camera class.
///
/// @ingroup Calibration
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Common.h"
#include "Camera.h"

// Calculate the base 2 logarithm.
double log2(double x);

// Fit a hyperplane to a set of ND points.
void FitPlane(const CvMat* points, float* plane);

// Find intersection between a 3D plane and a 3D line.
void intersectLineWithPlane3D(const float* q, const float* v, const float* w, float* p, float& depth);

// Find closest point to two 3D lines.
void intersectLineWithLine3D(const float* q1, const float* v1, const float* q2, const float* v2, float* p);

// Define camera capture (support Logitech QuickCam 9000 raw-mode).
IplImage* QueryFrame2(CvCapture* capture, struct slParams* sl_params, bool return_raw = false);

// Capture live image stream (e.g., for adjusting object placement).
int camPreview(Camera* camera, struct slParams* sl_params, struct slCalib* sl_calib);

// Shade a grayscale image using the "winter" colormap (similar to Matlab's).  
void colorizeWinter(IplImage* src, IplImage*& dst, IplImage* mask);

// Show an image, resampled to desired size.
void ShowImageResampled(char* name, IplImage* image, int width, int height);

// Save a VRML-formatted point cloud.
int savePointsVRML(char* filename, CvMat* points, CvMat* normals, CvMat* colors, CvMat* mask);

// Save a OBJ-formatted point cloud.
int savePointsOBJ(char* filename, CvMat* points, CvMat* faces, CvMat* normals, CvMat* uvCoords, CvMat* colors, CvMat* mask);

// Save in a format used by sba - sfm
int savePointsTxt(char* filename, CvMat* points, IplImage*& gray_decoded_cols, IplImage*& gray_decoded_rows, CvMat* mask, struct slParams* sl_params);

// Save XML-formatted configuration file.
void writeConfiguration(const char* filename, struct slParams* sl_params);

// Read XML-formatted configuration file.
void readConfiguration(const char* filename, struct slParams* sl_params);

// In-place conversion of a 10-bit raw image to an 8-bit BGR image.
// Note: Only works with Logitech QuickCam 9000 in 10-bit raw-mode (with a Bayer BGGR mosaic).
void CvtLogitech9000Raw(IplImage* image, bool return_raw);

IplImage* Gray2BGR(IplImage* frame);

void PrintMatrix(std::string name, cv::Mat &mat);