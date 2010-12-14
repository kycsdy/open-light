// *************************************************************
// Abstracted Camera Class
//
// Brett Jones 2009
// *************************************************************

#pragma once

#include "Common.h"
#include "CameraConfigParams.h"

#include "CalibrationExceptions.h"

class Camera
{
public:
    virtual void Init(CameraConfigParams* camParams) = 0;
    virtual void StartCapture() = 0;
    virtual void EndCapture() = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// <summary>   Return a color image of the current camera frame. </summary>
    ///
    /// <returns>   IplImage ptr of the current camera image. </returns>
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual IplImage* QueryFrame() = 0;

    IplImage* QueryFrameSafe(int delayFrames=0);

    IplImage* QueryFrameRGB(int delayFrames=0);
    IplImage* QueryFrameR(int delayFrames=0);
    IplImage* QueryFrameG(int delayFrames=0);
    IplImage* QueryFrameB(int delayFrames=0);
    IplImage* QueryFrameGray(int delayFrames=0);

    // Accessor methods
    int GetWidth() { return mWidth; };
    int GetHeight() {return mHeight; };

protected:

    /// <summary> width of the image.  </summary>
    int mWidth;

    /// <summary> height of the image.  </summary>
    int mHeight;

    /// <summary> the current frame.  </summary>
    IplImage* mCurFrame;

    /// <summary> Camera configuration parameters </summary>
    CameraConfigParams* mCamParams;

    /// <summary> Did everything load and is the camera enabled.  </summary>
    bool mEnabled;
};
