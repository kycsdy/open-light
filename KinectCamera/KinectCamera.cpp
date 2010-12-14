////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file   KinectCamera\KinectCamera.cpp
///
/// @brief  Implements the kinect camera class. 
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "KinectCamera.h"
#include "KinectInterface.h"

#include "cv.h"
#include "highgui.h"

KinectCamera::KinectCamera()
{
	mCurFrame = cvCreateImage(cvSize(640,  480), IPL_DEPTH_8U, 3);
}

KinectCamera::~KinectCamera()
{
    if(!mKinectInterface)
        delete mKinectInterface;
    if(!mKinect)
        delete mKinect;
}

void KinectCamera::Init(CameraConfigParams* camParams)
{
    mCamParams = camParams;
}

void KinectCamera::InitHardware(Kinect::Kinect* kinect)
{
    mKinect = kinect;

    mKinectInterface = new KinectInterface(kinect);

    // initialize the camera based on the config parameters
    mKinect->SetMotorPosition(1);
    mKinect->SetLedMode(::Kinect::Led_Yellow);
    mKinect->AddListener(mKinectInterface);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Queries the frame. </summary>
///
/// <returns>   null if it fails, else the frame. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage* KinectCamera::QueryFrame(void)
{
	mKinectInterface->update();

	IplImage* cvImage = cvCreateImage(cvSize(640,  480), IPL_DEPTH_8U, 3);
	memcpy(cvImage->imageData, mKinectInterface->getKinect()->mColorBuffer, 640*480*3 );

    return cvImage;
}