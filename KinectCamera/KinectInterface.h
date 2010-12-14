#pragma once

#include "Kinect-win32.h"
#include "Kinect-Utility.h"

#include <conio.h>
#include <windows.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @class  KinectInterface
///
/// @brief  Kinect interface. 
///
///
/// @ingroup KinectCamera
///
/// @author Brett Jones
/// @date   12/12/2010
////////////////////////////////////////////////////////////////////////////////////////////////////
class KinectInterface: public ::Kinect::KinectListener
{

public:
	KinectInterface();
	KinectInterface(::Kinect::Kinect *K);
	~KinectInterface();

	void parseDepth();
	void parseColor();

	virtual void DepthReceived(::Kinect::Kinect *K);
	virtual void ColorReceived(::Kinect::Kinect *K);

	unsigned char* getColorDepthBuffer()	{return mColoredDepthBuffer;};
	unsigned char* getColorBuffer()			{return mColorBuffer;};
	float* getDepthBuffer()					{return mDepthBuffer;};

	bool isColorReady()						{ return colorAvailable; };
	bool isDepthReady()						{ return depthAvailable; };
	bool update();

	void setKinect(::Kinect::Kinect *k)		{ mKinect = k; };
	Kinect::Kinect* getKinect()			{return mKinect;};

private:

	Kinect::Kinect *mKinect;

	bool colorAvailable;
	bool depthAvailable;

	unsigned short mGammaMap[2048];
	unsigned char mColoredDepthBuffer[640*480*3];
	unsigned char mColorBuffer[640*480*3];
	float mDepthBuffer[640*480];
	float mMaxDepthBuffer[640*480];

	float mMotorPosition;
	int mLedMode;

	int mLastDepthFrameCounter;
	int mDepthFrameCounter; 

    bool mDebugInfo;
    bool mEnableColor;
    bool mEnableDepth;

};