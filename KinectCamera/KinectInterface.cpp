////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file   KinectCamera\KinectInterface.cpp
///
/// @brief  Implements the kinect interface class. 
////////////////////////////////////////////////////////////////////////////////////////////////////


#include "KinectInterface.h"

#include "cv.h"
#include "highgui.h"

KinectInterface::KinectInterface()
{}

KinectInterface::KinectInterface(Kinect::Kinect *K)
{
	for (int i=0; i<2048; i++)
		mGammaMap[i] = (unsigned short)(float)(powf(i/2048.0f, 3)*6*6*256);

	for (int i =0;i<640*480;i++) 
		mMaxDepthBuffer[i] = -10000000;

	mKinect = K;

	mLastDepthFrameCounter = 0;
	mDepthFrameCounter = 0;

	colorAvailable = false;
	depthAvailable = false;

    mDebugInfo = false;
    mEnableColor = true;
    mEnableDepth = false;

    if(mDebugInfo)
    {
        if(mEnableColor)
	        cvNamedWindow("ColorImage", CV_WINDOW_AUTOSIZE);
        if(mEnableDepth)
	        cvNamedWindow("DepthImage", CV_WINDOW_AUTOSIZE);
    }
}

KinectInterface::~KinectInterface()
{
	mKinect->RemoveListener(this);
}


bool KinectInterface::update()
{
	bool updated = false;

	if (isColorReady() && mEnableColor)
	{
		colorAvailable = false;
		updated = true;

		memcpy( mColorBuffer, mKinect->mColorBuffer, 640*480*3);
		parseColor();
	}
	if (isDepthReady() && mEnableDepth)
	{
		depthAvailable = false;
		updated = true;

		parseDepth();
	}

	return updated;
}

void KinectInterface::parseColor()
{
    if(mDebugInfo)
    {
	    IplImage* cvImage = cvCreateImage(cvSize(640,  480), IPL_DEPTH_8U, 3);
	    memcpy( cvImage->imageData, mKinect->mColorBuffer, 640*480*3 );
	    cvCvtColor(cvImage,cvImage,CV_BGR2RGB);
	    cvShowImage("ColorImage", cvImage);
	    cvWaitKey(1);
	    cvReleaseImage(&cvImage);
    }
}

void KinectInterface::parseDepth()
{
	int i=0;
	for (int y=0; y<480; y++)
	{
		unsigned char* destrow = mColoredDepthBuffer + ((y)*(640))*3;
		float *actualDepth = mDepthBuffer + ((y)*640);
		float *maxDepth = mMaxDepthBuffer + ((y)*640);
		for (int x=0; x<640; x++)
		{
			unsigned short Depth = mKinect->mDepthBuffer[i];
			if(::Kinect::Kinect_IsDepthValid(Depth))
			{
				float depthValue = ::Kinect::Kinect_DepthValueToZ(Depth);
				*actualDepth++ = depthValue;
				if(depthValue > *maxDepth)
				{
					*maxDepth = depthValue;
				}
			}
			else
			{
				*actualDepth++ = -100000;
			}

			maxDepth++;

			int pval = mGammaMap[Depth];
			int lb = pval & 0xff;
			switch (pval>>8) 
			{
				case 0:
					destrow[2] = 255;
					destrow[1] = 255-lb;
					destrow[0] = 255-lb;
					break;
				case 1:
					destrow[2] = 255;
					destrow[1] = lb;
					destrow[0] = 0;
					break;
				case 2:
					destrow[2] = 255-lb;
					destrow[1] = 255;
					destrow[0] = 0;
					break;
				case 3:
					destrow[2] = 0;
					destrow[1] = 255;
					destrow[0] = lb;
					break;
				case 4:
					destrow[2] = 0;
					destrow[1] = 255-lb;
					destrow[0] = 255;
					break;
				case 5:
					destrow[2] = 0;
					destrow[1] = 0;
					destrow[0] = 255-lb;
					break;
				default:
					destrow[2] = 0;
					destrow[1] = 0;
					destrow[0] = 0;
					break;
			}
			destrow += 3;
			i++;
		}
	}

    if(mDebugInfo)
    {
	    IplImage* cvImage = cvCreateImage(cvSize(640,  480), IPL_DEPTH_8U, 3);
	    memcpy( cvImage->imageData, mColoredDepthBuffer, 640*480*3);
	    cvShowImage("DepthImage", cvImage);
	    cvWaitKey(1);
	    cvReleaseImage(&cvImage);
    }
}

void KinectInterface::DepthReceived(Kinect::Kinect *kinect)
{
    if(mEnableDepth)
    {
	    mDepthFrameCounter++;
	    kinect->ParseDepthBuffer();
	    depthAvailable = true;
    }
}

void KinectInterface::ColorReceived(Kinect::Kinect *kinect)
{
    if(mEnableColor)
    {
	    kinect->ParseColorBuffer();
	    colorAvailable = true;
    }
}