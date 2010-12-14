////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	Calibration\Camera.cpp
//
// summary:	Implements the camera class
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Common.h"

#include "Camera.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Returns a camera image after a delay of a certain number of frames. This is
///             useful with cameras that autoexpose. </summary>
///
/// <param name="delayFrames">  The number of frames to delay. </param>
///
/// <returns>   Ptr to the IplImage of the camera image. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage* Camera::QueryFrameSafe(int delayFrames)
{
    

    IplImage* curFrame;

	for(int picIter = 0; picIter < delayFrames - 1; picIter++)
    {
        curFrame = QueryFrame();
        cvReleaseImage(&curFrame);
    }

    mCurFrame = QueryFrame();

	return mCurFrame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Queries a frame rgb. </summary>
///
/// <param name="delayFrames">  The delay frames. </param>
///
/// <returns>   null if it fails, else the frame rgb. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage* Camera::QueryFrameRGB(int delayFrames)
{
    mCurFrame = QueryFrameSafe(delayFrames);

    IplImage* curFrameRGB;

    // if color image
    if(mCurFrame->nChannels == 1)
    {
        curFrameRGB = cvCreateImage(cvSize(mCurFrame->width, mCurFrame->height), mCurFrame->depth, 1);
        cvCvtColor(mCurFrame, curFrameRGB, CV_GRAY2RGB);
        cvReleaseImage(&mCurFrame);
        mCurFrame = curFrameRGB;
    }

    return mCurFrame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Queries a frame r. </summary>
///
/// <param name="delayFrames">  The delay frames. </param>
///
/// <returns>   null if it fails, else the frame r. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage* Camera::QueryFrameR(int delayFrames)
{
    mCurFrame = QueryFrameSafe(delayFrames);

    // if color image
    if(mCurFrame->nChannels > 1)
    {
        IplImage* curFrame_r = cvCreateImage(cvSize(mCurFrame->width, mCurFrame->height), mCurFrame->depth, 1);
        cvSplit(mCurFrame, NULL, NULL, curFrame_r, NULL);
        cvReleaseImage(&mCurFrame);
        mCurFrame = curFrame_r;
    }

    return mCurFrame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Queries a frame g. </summary>
///
/// <param name="delayFrames">  The delay frames. </param>
///
/// <returns>   null if it fails, else the frame g. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage* Camera::QueryFrameG(int delayFrames)
{
    mCurFrame = QueryFrameSafe(delayFrames);

    // if color image
    if(mCurFrame->nChannels > 1)
    {
        IplImage* curFrame_gr = cvCreateImage(cvSize(mCurFrame->width, mCurFrame->height), mCurFrame->depth, 1);
        cvSplit(mCurFrame, NULL, curFrame_gr, NULL, NULL);
        cvReleaseImage(&mCurFrame);
        mCurFrame = curFrame_gr;
    }

    return mCurFrame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Queries a frame b. </summary>
///
/// <param name="delayFrames">  The delay frames. </param>
///
/// <returns>   null if it fails, else the frame b. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage* Camera::QueryFrameB(int delayFrames)
{
    mCurFrame = QueryFrameSafe(delayFrames);

    // if color image
    if(mCurFrame->nChannels > 1)
    {
        IplImage* curFrame_b = cvCreateImage(cvSize(mCurFrame->width, mCurFrame->height), mCurFrame->depth, 1);
        cvSplit(mCurFrame, curFrame_b, NULL, NULL, NULL);
        cvReleaseImage(&mCurFrame);
        mCurFrame = curFrame_b;
    }

    return mCurFrame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>   Queries a frame gray. </summary>
///
/// <param name="delayFrames">  The delay frames. </param>
///
/// <returns>   null if it fails, else the frame gray. </returns>
////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage* Camera::QueryFrameGray(int delayFrames)
{
    mCurFrame = QueryFrameSafe(delayFrames);

    // if color image
    if(mCurFrame->nChannels > 1)
    {
        IplImage* curFrame_g = cvCreateImage(cvSize(mCurFrame->width, mCurFrame->height), mCurFrame->depth, 1);
        cvCvtColor(mCurFrame, curFrame_g, CV_BGR2GRAY);
        cvReleaseImage(&mCurFrame);
        mCurFrame = curFrame_g;
    }

    return mCurFrame;
}

