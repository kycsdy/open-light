////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file   KinectCamera\KinectCameraManager.h
///
/// @brief  Declares the kinect camera manager class. 
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Project includes
#include "KinectCamera.h"
#include "CameraManager.h"

#include <conio.h>
#include <windows.h>
#include <math.h>

// Forward declarations
namespace Kinect
{
    class KinectFinder;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @class  KinectCameraManager
///
/// @brief  Manager for kinect cameras. 
///
/// @ingroup KinectCamera
///
/// @author Brett Jones
/// @date   12/12/2010
////////////////////////////////////////////////////////////////////////////////////////////////////
class KINECT_CAMERA_DECL KinectCameraManager: public CameraManager
{

public:
    void Init(CameraConfigParams* camParams);
    void CleanUp();

private:

    Kinect::KinectFinder *mKinectManager;
};