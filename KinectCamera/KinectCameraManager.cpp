////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	KinectCamera\KinectCameraManager.cpp
//
// summary:	Implements the kinect sdk interface class
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "KinectCameraManager.h"

#include "CalibrationExceptions.h"
#include "KinectCamera.h"

// Kinect 
#include "Kinect-win32.h"
#include "Kinect-Utility.h"

// OpenCV
#include "cv.h"
#include "highgui.h"

void KinectCameraManager::Init(CameraConfigParams* camParams)
{
    mKinectManager = new Kinect::KinectFinder();

    int numCam = mKinectManager->GetKinectCount();

	if(numCam < 1)
	{
        throw new HardwareNotFound("Kinect Camera");
	}

    for(int camIter = 0; camIter < numCam; camIter++)
    {
        Kinect::Kinect* kinect = mKinectManager->GetKinect(camIter);
	    if(!kinect)
	    {
		    throw new HardwareInit("Kinect Camera");
	    }

        KinectCamera* kinectCamera = new KinectCamera();

        kinectCamera->Init(camParams);
        kinectCamera->InitHardware(kinect);

        mCameras.push_back(kinectCamera);
    }
}


void KinectCameraManager::CleanUp()
{
    std::vector<Camera*>::iterator camIter;
    for(camIter = mCameras.begin(); camIter != mCameras.end(); camIter++)
    {
        Camera* cam = *camIter;
        if(cam)
            delete cam;        
    }

    if(mKinectManager)
        delete mKinectManager;
}
