// *************************************************************
// Abstracted Camera Class
//
// Brett Jones 2009
// *************************************************************

#ifndef CV_CAMERA_H
#define CV_CAMERA_H

class CVCamera
{
public:
    CVCamera();
    ~CVCamera();

    virtual int Init(struct slParams* sl_params_) = 0;
    virtual int StartCapture() = 0;
    virtual int EndCapture() = 0;

    virtual IplImage* QueryFrame() = 0;
    virtual IplImage* QueryFrameSafe() = 0;
};

#endif //CV_CAMERA_H