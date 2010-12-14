////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	Calibration\CalibrateProCam.h
//
// summary:	Declares the calibrate pro camera class
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Common.h"
#include "Calibration.h"
#include "Camera.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @class  CalibrateProCam
///
/// @brief  Calibrate pro camera. 
///
/// @ingroup Calibration
///
/// @author Brett Jones
/// @date   12/12/2010
////////////////////////////////////////////////////////////////////////////////////////////////////
class CalibrateProCam
{
private:
    Camera* camera;

public:
    CalibrateProCam(Camera *camera_);

    ~CalibrateProCam();

    // Display the camera calibration results to the console.
    void displayCamCalib(struct slCalib* sl_calib);

    // Display the projector calibration results to the console.
    void displayProjCalib(struct slCalib* sl_calib);

    // Generate a chessboard pattern for projector calibration.
    int generateChessboard(struct slParams* sl_params, IplImage*& board, int& border_cols, int& border_rows);

    int generateChessboardScale(struct slParams* sl_params, IplImage*& board, int& border_cols, int& border_rows, float scale);

    // Detect chessboard corners (with subpixel refinement).
    // Note: Returns 1 if chessboard is found, 0 otherwise.
    int detectChessboard(IplImage* frame, CvSize board_size, CvPoint2D32f* corners, int* corner_count CV_DEFAULT(NULL));

    // Run projector-camera calibration (including intrinsic and extrinsic parameters).
    int runProjectorCalibration(struct slParams* sl_params, struct slCalib* sl_calib, bool calibrate_both);

private:
    // helper functions

};