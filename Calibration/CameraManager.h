////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	Calibration\CameraManager.h
//
// summary:	Abstracted Camera SDK Interface Class.
//          Virtual class that defines the interfaces for managing camera SDKs
//
// Brett Jones & Rajinder Sodhi 2010
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Common.h"
#include "Camera.h"
#include "CameraConfigParams.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Virtual class that defines the interfaces for managing camera SDKs. </summary>
////////////////////////////////////////////////////////////////////////////////////////////////////
class CameraManager
{
public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// <summary>   Initialises this object. </summary>
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual void Init(CameraConfigParams* camParams) = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// <summary>   Clean up all resources. </summary>
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual void CleanUp() = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// <summary>   Gets the valid cameras list. </summary>
    ///
    /// <returns>   null if it fails, else the cameras. </returns>
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<Camera*> GetCameras()
        { return mCameras; };
    
protected:
    /// <summary> A list of all cameras.  </summary>
    std::vector<Camera*> mCameras;

    /// <summary> Did the SDK load correctly.  </summary>
    bool mIsLoaded;

    /// <summary> Camera configuration parameters </summary>
    CameraConfigParams* mCamParams;
};
