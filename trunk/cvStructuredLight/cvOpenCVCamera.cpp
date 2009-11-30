#include "stdafx.h"

#include "cvOpenCVCamera.h"
#include "cvUtilProCam.h"

CVOpenCVCamera::CVOpenCVCamera(struct slParams* sl_params_)
{
    sl_params = sl_params_;
    
    capture = NULL;
}

CVOpenCVCamera::~CVOpenCVCamera()
{
    if(capture != NULL)
        cvReleaseCapture(&capture);
}

int CVOpenCVCamera::Init()
{
    // Initialize capture from any detected device.
    printf("Initializing camera and projector...\n"); 
    if(sl_params->Logitech_9000){
	    printf("Enabling Bayer mode for Logitech QuickCam 9000...\n");
	    system("Bayer.exe 1 10 > nul");
    }
    capture = cvCaptureFromCAM(CV_CAP_ANY);
    if(capture == NULL){
	    printf("ERROR: No camera was detected by OpenCV!\n");
	    printf("Press any key to exit.\n");
	    _getch();
	    return -1;
    }
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH,  sl_params->cam_w);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, sl_params->cam_h);

    return 0;
}

int CVOpenCVCamera::StartCapture()
{
    return 0;
}

int CVOpenCVCamera::EndCapture()
{
    return 0;
}

IplImage* CVOpenCVCamera::QueryFrame()
{
    IplImage* cam_frame = cvQueryFrame2(capture, sl_params);
    return cam_frame;
}

IplImage* CVOpenCVCamera::QueryFrameSafe()
{
    IplImage* cam_frame = cvQueryFrameSafe(capture, sl_params, false);
    return cam_frame;
}

