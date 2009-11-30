#include "stdafx.h"

#include "cvCanonCamera.h"

EdsError EDSCALLBACK handleObjectEvent( EdsObjectEvent event, EdsBaseRef object, EdsVoid * context)
{
    // do something
    // Object must be released
    if(object)
    {
        EdsRelease(object);
    }

    return EDS_ERR_OK;
}

EdsError EDSCALLBACK handlePropertyEvent (EdsPropertyEvent event, EdsPropertyID property, EdsUInt32 inParam, EdsVoid * context)
{
    // do something

    return EDS_ERR_OK;
}

EdsError EDSCALLBACK handleStateEvent (EdsStateEvent event, EdsUInt32 parameter, EdsVoid * context)
{
    // do something

    return EDS_ERR_OK;
}

EdsError getFirstCamera(EdsCameraRef *camera)
{
    EdsError err = EDS_ERR_OK;
    EdsCameraListRef cameraList = NULL;
    EdsUInt32 count = 0;

    // Get camera list
    err = EdsGetCameraList(&cameraList);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetCameraList: 0x%X\n", err);
        return err;    
    }

    err = EdsGetChildCount(cameraList, &count);
    if(count == 0)
    {
        EdsRelease(cameraList);
        err = EDS_ERR_DEVICE_NOT_FOUND;
        return err;
    }

    err = EdsGetChildAtIndex(cameraList, 0 ,camera);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetChildAtIndex: 0x%X\n", err);
        EdsRelease(cameraList);
        return err;    
    }

    // Release camera list
    if(cameraList != NULL)
    {
        EdsRelease(cameraList);
        cameraList = NULL;
    }

    return EDS_ERR_OK;
}

CVCanonCamera::CVCanonCamera()
{
    camera = NULL;
    isSDKLoaded = false;
    captureImageInit = false;
}

CVCanonCamera::~CVCanonCamera()
{
    EdsError err = EDS_ERR_OK;
    // Close session with camera
    err = EdsCloseSession(camera);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsCloseSession: 0x%X\n", err);  
    }

    // Release camera
    if(camera != NULL)
    {
        EdsRelease(camera);
    }

    // Terminate SDK
    if(isSDKLoaded)
    {
        EdsTerminateSDK();
    }
}

int CVCanonCamera::Init()
{
    CoInitializeEx(NULL, COINIT_MULTITHREADED);

    EdsError err = EDS_ERR_OK;
    
    // Initialize SDK
    err = EdsInitializeSDK();
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsInitializeSDK: 0x%X\n", err);
        return err;    
    }

    isSDKLoaded = true;

    err = getFirstCamera(&camera);
    if(err != EDS_ERR_OK)
    {
        printf("Error in getFirstCamera: 0x%X\n", err);
        return err;    
    }

    err = EdsSetObjectEventHandler(camera, kEdsObjectEvent_All, handleObjectEvent, NULL);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsSetObjectEventHandler: 0x%X\n", err);
        return err;    
    }

    err = EdsSetPropertyEventHandler(camera, kEdsPropertyEvent_All, handlePropertyEvent, NULL);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsSetPropertyEventHandler: 0x%X\n", err);
        return err;    
    }

    err = EdsSetCameraStateEventHandler(camera, kEdsStateEvent_All, handleStateEvent, NULL);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsSetCameraStateEventHandler: 0x%X\n", err);
        return err;    
    }

    err = EdsOpenSession(camera);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsOpenSession: 0x%X\n", err);
        return err;    
    }

    return EDS_ERR_OK;        
}

int CVCanonCamera::StartCapture()
{
    EdsError err = EDS_ERR_OK;

    // Get the output device for the live view image
    EdsUInt32 device;
    err = EdsGetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0 ,sizeof(device), &device );
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetPropertyData: 0x%X\n", err);
        return err;    
    }
    
    device |= kEdsEvfOutputDevice_PC;
    err = EdsSetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0 ,sizeof(device), &device);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsSetPropertyData: 0x%X\n", err);
        return err;    
    }

    // A property change event notification is issued from the camera if property settings are made successfully.
    // Start downloading of the live view image once the property change notification arrives.

    return EDS_ERR_OK;
}

int CVCanonCamera::EndCapture()
{
    EdsError err = EDS_ERR_OK;

    // Get the output device for the live view image
    EdsUInt32 device;
    err = EdsGetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device );
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetPropertyData: 0x%X\n", err);
        return err;    
    }

    device &= ~kEdsEvfOutputDevice_PC;

    err = EdsSetPropertyData(camera, kEdsPropID_Evf_OutputDevice, 0, sizeof(device), &device);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsSetPropertyData: 0x%X\n", err);
        return err;    
    }

    return EDS_ERR_OK;
}

EdsError CVCanonCamera::UpdateView()
{
    EdsError err = EDS_ERR_OK;
    EdsStreamRef stream = NULL;
    EdsEvfImageRef evfImage = NULL;

    // Create memory stream.
    err = EdsCreateMemoryStream(0, &stream);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsCreateMemoryStream: 0x%X\n", err);
        return err;    
    }    

    err = EdsCreateEvfImageRef(stream, &evfImage);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsCreateEvfImageRef: 0x%X\n", err);
        return err;    
    }    

    bool wait = true;
    int numTries = 0;
    while(wait && numTries++ < 20)
    {
        err = EdsDownloadEvfImage(camera, evfImage);
        if(err != EDS_ERR_OK && err != EDS_ERR_OBJECT_NOTREADY)
        {
            printf("Error in EdsDownloadEvfImage: 0x%X\n", err);
            return err;    
        }
        if(err == EDS_ERR_OBJECT_NOTREADY)
        {
            printf("Waiting for camera\n");
            Sleep(250);
        }
        else
        {
            wait = false;
        }
    }
    if(numTries > 20)
    {
        printf("ERROR: camera is taking too long for EdsDownloadEvfImage\n");
        return -1;
    }

	EVF_DATASET dataSet;

    dataSet.stream = stream;

	// Get magnification ratio (x1, x5, or x10).
	err = EdsGetPropertyData(evfImage, kEdsPropID_Evf_Zoom, 0, sizeof(dataSet.zoom),  &dataSet.zoom);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetPropertyData Zoom: 0x%X\n", err);
        return err;    
    }

	// Get position of the focus border.
	// Upper left coordinate of the focus border using JPEG Large size as a reference.
	err = EdsGetPropertyData(evfImage, kEdsPropID_Evf_ZoomPosition, 0, sizeof(dataSet.zoomPosition), &dataSet.zoomPosition);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetPropertyData Zoom Position: 0x%X\n", err);
        return err;    
    }

	// Get position of image data. (when enlarging)
	// Upper left coordinate using JPEG Large size as a reference.
	err = EdsGetPropertyData(evfImage, kEdsPropID_Evf_ImagePosition, 0, sizeof(dataSet.imagePosition), &dataSet.imagePosition);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetPropertyData Image Position: 0x%X\n", err);
        return err;    
    }

	// Get histogram (RGBY).
	err = EdsGetPropertyData(evfImage, kEdsPropID_Evf_Histogram, 0, sizeof(dataSet.histogram), dataSet.histogram);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetPropertyData Histogram: 0x%X\n", err);
        return err;    
    }

    // ****************************************************************
    // To do:
    // Warning!
    // Hard coded for a T1i
    // ****************************************************************
	dataSet.sizeJpegLarge.width = 4752;
	dataSet.sizeJpegLarge.height = 3168;

    unsigned char* pByteImage = NULL;
    // Get image (JPEG) pointer.
    err = EdsGetPointer(stream, (EdsVoid**)&pByteImage );
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetPointer Histogram: 0x%X\n", err);
        return err;    
    }

    EdsUInt32 size;
    err = EdsGetLength(stream, &size);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetLength Histogram: 0x%X\n", err);
        return err;    
    }

    EdsImageRef image = NULL;
    EdsImageInfo imageInfo;

    err = EdsCreateImageRef(stream, &image);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsCreateImageRef: 0x%X\n", err);
        return err;    
    }

    err = EdsGetImageInfo(image, kEdsImageSrc_FullView, &imageInfo);
    if(err != EDS_ERR_OK)
    {
        printf("Error in EdsGetImageInfo: 0x%X\n", err);
        return err;    
    }

    if(imageInfo.componentDepth != 8)
    {
        printf("Error imageInfo.componentDepth != 8\n");
        return err;
    }

    captureImage = cvCreateImage(cvSize(imageInfo.width, imageInfo.height), IPL_DEPTH_8U, imageInfo.numOfComponents);

    //HRESULT result;
    //CComPtr<IWICBitmapDecoder> decoder;
    //
    //result = decoder.CoCreateInstance(CLSID_WICJpegDecoder);

    EdsUInt32 DataSize = 0;

    ATL::CImage cImage;
    HRESULT hr;

    CComPtr<IStream> iStream = NULL;
    HGLOBAL hMem = GlobalAlloc(GHND, size);
    LPVOID pBuff = GlobalLock(hMem);
    memcpy(pBuff, pByteImage, size);
    GlobalUnlock(hMem);
    hr = CreateStreamOnHGlobal(hMem, TRUE, &iStream);

    // Get the bitmap image from the stream
    if ((hr = cImage.Load(iStream)) == S_OK)
    {
        // Copy the dib data to the provided buffer
        int pitch = cImage.GetPitch();
        int height = cImage.GetHeight();
        BYTE* pBits = (BYTE*)cImage.GetBits();
        if (pitch < 0)
            pBits += (pitch *(height -1));
        memcpy(captureImage->imageData, pBits, abs(pitch) * height);
    }

    // Free resources
    GlobalFree(hMem);

    //// Create stream object here...

    //HR(decoder->Initialize(
    //  stream,
    //  WICDecodeMetadataCacheOnDemand))

    //UINT frameCount = 0;
    //result = decoder->GetFrameCount(&frameCount);
    //for (UINT index = 0; index < frameCount; ++index)
    //{
    //    CComPtr<IWICBitmapFrameDecode> frame;

    //    result = decoder->GetFrame(index, &frame);
    //}

    //UINT width = 0;
    //UINT height = 0;
    //result = frame->GetSize(&width, &height);

    //EdsPoint rectOrigin;
    //rectOrigin.x = 0;
    //rectOrigin.y = 0;
    //EdsSize rectSize;
    //rectSize.width = 0;//imageInfo.width;
    //rectSize.height = 0;//imageInfo.height;
    //EdsRect rect;
    //rect.point = rectOrigin;
    //rect.size = rectSize;

    //err = EdsGetImage(image, kEdsImageSrc_FullView, kEdsTargetImageType_RGB, rect, rectSize, (EdsStreamRef)(&(captureImage->imageData)));
    //if(err != EDS_ERR_OK)
    //{
    //    printf("Error in EdsGetImage: 0x%X\n", err);
    //    return err;    
    //}

    //cvSaveImage("test2.jpg", captureImage);
    cvFlip(captureImage, NULL, 0);

    // Release stream
    if(stream != NULL)
    {
        err = EdsRelease(stream);
        if(err != EDS_ERR_OK)
        {
            printf("Error in EdsRelease: 0x%X\n", err);
            return err;    
        }
        stream = NULL;
    }

    // Release evfImage
    if(evfImage != NULL)
    {
        err = EdsRelease(evfImage);
        if(err != EDS_ERR_OK)
        {
            printf("Error in EdsRelease: 0x%X\n", err);
            return err;    
        }
        evfImage = NULL;
    }

    return EDS_ERR_OK;
}

IplImage* CVCanonCamera::QueryFrame()
{
    EdsError err = EDS_ERR_OK;

    err = UpdateView();

    if(err != EDS_ERR_OK)
    {
        printf("Error in UpdateView: 0x%X\n", err);
        return NULL;
    }

    return captureImage;
}

IplImage* CVCanonCamera::QueryFrameSafe()
{
    return QueryFrame();
}