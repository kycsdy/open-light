// *************************************************************
// Camera Class using the EDSDK for Canon Cameras
//
// Brett Jones 2009
// *************************************************************

#ifndef CV_CANON_CAMERA_H
#define CV_CANON_CAMERA_H

#include "stdafx.h"
#include "cvCamera.h"

#include "EDSDK.h"
#include "EDSDKErrors.h"
#include "EDSDKTypes.h"

#include <wincodec.h>
#include <wincodecsdk.h>

typedef struct _EVF_DATASET 
{
	EdsStreamRef stream; // JPEG stream.
	EdsUInt32 zoom;
	EdsPoint zoomPosition;
	EdsPoint imagePosition;
	EdsUInt32 histogram[256 * 4]; //(YRGB) YRGBYRGBYRGBYRGB....
	EdsSize sizeJpegLarge;
}EVF_DATASET;

class CVCanonCamera : public CVCamera
{
private:
    EdsCameraRef camera;
    bool isSDKLoaded;
	IplImage* captureImage;
    bool captureImageInit;

    EdsError UpdateView();

public:
    CVCanonCamera();
    ~CVCanonCamera();

    int Init(struct slParams* sl_params_);
    int StartCapture();
    int EndCapture();

    IplImage* QueryFrame();
    IplImage* QueryFrameSafe();
};

#endif //CV_CANON_CAMERA_H