////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file   KinectCamera\KinectCamera.h
///
/// @brief  Declares the kinect camera class. 
/// @defgroup KinectCamera KinectCamera
///       Library for interfacing with Kinect camera API
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// project includes
#include "Common.h"
#include "Camera.h"

#include <conio.h>
#include <windows.h>
#include <math.h>

#include "Calibration.h"

// Export definitions
#ifdef OS_WIN
    #ifdef KINECT_CAMERA_DECL_EXPORT
        #define KINECT_CAMERA_DECL __declspec(dllexport)
    #else
        #define KINECT_CAMERA_DECL __declspec(dllimport)
    #endif
#else
    #define KINECT_CAMERA_DECL
#endif

// forward declarations
class KinectInterface;

namespace Kinect
{
    class Kinect;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
/// @class  KinectCamera
///
/// @brief  Kinect camera. 
///
/// 
/// @ingroup KinectCamera
/// @author Brett Jones
/// @date   12/12/2010
////////////////////////////////////////////////////////////////////////////////////////////////////
class KINECT_CAMERA_DECL KinectCamera: public Camera
{
public:
	KinectCamera();
	~KinectCamera();

    virtual void Init(CameraConfigParams* camParams);
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// <summary>   Initialises the hardware with the Kinect passed in by KinectCameraManager. </summary>
    ///
    /// <param name="kinect">   Kinect created by CameraManager. </param>
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    void InitHardware(Kinect::Kinect* kinect);
    
    virtual void StartCapture()
        { return; };

	virtual void EndCapture()
        { return; };

    virtual IplImage* QueryFrame();

private:
    KinectInterface* mKinectInterface;
    Kinect::Kinect *mKinect;
};