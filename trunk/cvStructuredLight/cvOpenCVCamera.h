// *************************************************************
// OpenCV Camera Class
//
// This wraps the OpenCV camera allowing for an abstracted interface
//
// Brett Jones 2009
// *************************************************************

#ifndef CV_OPENCV_CAMERA_H
#define CV_OPENCV_CAMERA_H

#include "cvCamera.h"
#include "cvStructuredLight.h"

class CVOpenCVCamera : public CVCamera
{
private:
    CvCapture* capture;
    struct slParams* sl_params;

public:
    CVOpenCVCamera();
    ~CVOpenCVCamera();

    int Init(struct slParams* sl_params_);
    int StartCapture();
    int EndCapture();

    IplImage* QueryFrame();
    IplImage* QueryFrameSafe();
};

#endif //CV_CAMERA_H