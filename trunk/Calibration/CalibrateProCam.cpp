////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	Calibration\CalibrateProCam.cpp
//
// summary:	Implements the calibrate pro camera class
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Common.h"
#include "Calibration.h"
#include "CalibrateProCam.h"
#include "UtilProCam.h"
#include <fstream>

using namespace std;
using namespace cv;

// Constructor
CalibrateProCam::CalibrateProCam(Camera *camera_)
{
    camera = camera_;
}

// Destructor
CalibrateProCam::~CalibrateProCam()
{
}

// Display the camera calibration results to the console.
void CalibrateProCam::displayCamCalib(struct slCalib* sl_calib){
	printf("***Camera calibration:\n");
	if(sl_calib->cam_intrinsic_calib){
		printf("+ Intrinsic parameters = \n");
		for(int i=0; i<3; i++){
			printf("   ");
			for(int j=0; j<3; j++)
				printf("%7.3f ", cvmGet(sl_calib->cam_intrinsic, i, j));
			printf("\n");
		}
		printf("+ Extrinsic parameters = \n");
		for(int i=0; i<2; i++){
			printf("   ");
			for(int j=0; j<3; j++)
                printf("%7.3f ", cvmGet(sl_calib->cam_extrinsic, i, j));
			printf("\n");
		}
		printf("+ Distortion coefficients = \n   ");
		for(int i=0; i<5; i++)
			printf("%7.3f ", cvmGet(sl_calib->cam_distortion, i, 0));
		printf("\n");
	}
	else
		printf("+ Camera has not been calibrated!\n");
}

// Display the projector calibration results to the console.
void CalibrateProCam::displayProjCalib(struct slCalib* sl_calib){
	printf("***Projector calibration:\n");
	if(sl_calib->proj_intrinsic_calib){
		printf("+ Intrinsic parameters = \n");
		for(int i=0; i<3; i++){
			printf("   ");
			for(int j=0; j<3; j++)
				printf("%7.3f ", cvmGet(sl_calib->proj_intrinsic, i, j));
			printf("\n");
		}
		printf("+ Extrinsic parameters = \n");
		for(int i=0; i<2; i++){
			printf("   ");
			for(int j=0; j<3; j++)
                printf("%7.3f ", cvmGet(sl_calib->proj_extrinsic, i, j));
			printf("\n");
		}
		printf("+ Distortion coefficients = \n   ");
		for(int i=0; i<5; i++)
			printf("%7.3f ", cvmGet(sl_calib->proj_distortion, i, 0));
		printf("\n");
	}
	else
		printf("+ Projector has not been calibrated!\n");
}

// Generate a chessboard pattern for projector calibration.
int CalibrateProCam::generateChessboard(struct slParams* sl_params, IplImage*& board, int& border_cols, int& border_rows){

	// Calculate chessboard border.
	border_cols = (int)floor((board->width -(sl_params->proj_board_w+1)*sl_params->proj_board_w_pixels)/2.0);
	border_rows = (int)floor((board->height-(sl_params->proj_board_h+1)*sl_params->proj_board_h_pixels)/2.0);

	// Check for chessboard errors.
	if( (border_cols < 0) || (border_rows < 0) ){
		printf("ERROR: Cannot create chessboard with user-requested dimensions!\n");
		return -1;
	}

	// Initialize chessboard with white image.
	cvSet(board, cvScalar(255));

	// Create odd black squares.
	uchar* data = (uchar*)board->imageData;
	int step = board->widthStep/sizeof(uchar);
	for(int r=0; r<(sl_params->proj_board_h+1); r+=2)
		for(int c=0; c<(sl_params->proj_board_w+1); c+=2)
			for(int i=(r*sl_params->proj_board_h_pixels+border_rows); 
				i<((r+1)*sl_params->proj_board_h_pixels+border_rows); i++)
				for(int j=(c*sl_params->proj_board_w_pixels+border_cols); 
					j<((c+1)*sl_params->proj_board_w_pixels+border_cols); j++)
					data[i*step+j] = 0;

	// Create even black squares.
	for(int r=1; r<sl_params->proj_board_h; r+=2)
		for(int c=1; c<sl_params->proj_board_w; c+=2)
			for(int i=(r*sl_params->proj_board_h_pixels+border_rows); 
				i<((r+1)*sl_params->proj_board_h_pixels+border_rows); i++)
				for(int j=(c*sl_params->proj_board_w_pixels+border_cols); 
					j<((c+1)*sl_params->proj_board_w_pixels+border_cols); j++)
					data[i*step+j] = 0;

	// Return without errors.
	return 0;
}

// Generate a chessboard pattern for projector calibration.
int CalibrateProCam::generateChessboardScale(struct slParams* sl_params, IplImage*& board, int& border_cols, int& border_rows, float scale){

	// Calculate chessboard border.
	border_cols = (int)floor((board->width -(sl_params->proj_board_w+1)*sl_params->proj_board_w_pixels*scale)/2.0);
	border_rows = (int)floor((board->height-(sl_params->proj_board_h+1)*sl_params->proj_board_h_pixels*scale)/2.0);

	// Check for chessboard errors.
	if( (border_cols < 0) || (border_rows < 0) ){
		printf("ERROR: Cannot create chessboard with user-requested dimensions!\n");
		return -1;
	}

	// Initialize chessboard with white image.
	cvSet(board, cvScalar(255));

	// Create odd black squares.
	uchar* data = (uchar*)board->imageData;
	int step = board->widthStep/sizeof(uchar);
	for(int r=0; r<(sl_params->proj_board_h+1); r+=2)
		for(int c=0; c<(sl_params->proj_board_w+1); c+=2)
			for(int i=(r*sl_params->proj_board_h_pixels*scale+border_rows); 
				i<((r+1)*sl_params->proj_board_h_pixels*scale+border_rows); i++)
				for(int j=(c*sl_params->proj_board_w_pixels*scale+border_cols); 
					j<((c+1)*sl_params->proj_board_w_pixels*scale+border_cols); j++)
					data[i*step+j] = 0;

	// Create even black squares.
	for(int r=1; r<sl_params->proj_board_h; r+=2)
		for(int c=1; c<sl_params->proj_board_w; c+=2)
			for(int i=(r*sl_params->proj_board_h_pixels*scale+border_rows); 
				i<((r+1)*sl_params->proj_board_h_pixels*scale+border_rows); i++)
				for(int j=(c*sl_params->proj_board_w_pixels*scale+border_cols); 
					j<((c+1)*sl_params->proj_board_w_pixels*scale+border_cols); j++)
					data[i*step+j] = 0;

	// Return without errors.
	return 0;
}

// Detect chessboard corners (with subpixel refinement).
// Note: Returns 1 if chessboard is found, 0 otherwise.
int CalibrateProCam::detectChessboard(IplImage* frame, CvSize board_size,
                     CvPoint2D32f* corners,
                     int* corner_count){

	// Find chessboard corners. 
	int found = cvFindChessboardCorners(
		frame, board_size, corners, corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

	// Refine chessboard corners.
	IplImage* gray_frame = cvCreateImage(cvGetSize(frame), frame->depth, 1);
	if(frame->nChannels > 1)
		cvCvtColor(frame, gray_frame, CV_BGR2GRAY);
	else
		cvCopyImage(frame, gray_frame);
	cvFindCornerSubPix(gray_frame, corners, *corner_count, 
		cvSize(11,11), cvSize(-1,-1), 
		cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

	// Release allocated resources.
	cvReleaseImage(&gray_frame);

	// Return without errors (i.e., chessboard was found).						 
	return found;
}

static void printMatrix(CvMat *mat, std::string name)
{
    printf("printMatrix: %s\n", name.c_str());
    for(int row = 0; row < mat->rows; row++)
    {
        for(int col = 0; col < mat->cols; col++)
        {
            printf("%4.3f ", cvmGet(mat, row, col));
        }
        printf("\n");
    }
}

static void printPoints(CvPoint2D32f *points, int length, std::string name)
{
    printf("printPoints %s\n", name.c_str());
    for(int pointIndex = 0; pointIndex < length; pointIndex++)
    {
        printf("%.3f %.3f\n", points->x, points->y);
        points++;
    }
}

// Run projector-camera calibration (including intrinsic and extrinsic parameters).
int CalibrateProCam::runProjectorCalibration(struct slParams* sl_params, 
					        struct slCalib* sl_calib,
							bool calibrate_both){

	// Reset projector (and camera) calibration status (will be set again, if successful.
	sl_calib->proj_intrinsic_calib   = false;
	sl_calib->procam_extrinsic_calib = false;
	if(calibrate_both)
		sl_calib->cam_intrinsic_calib = false;

	// Create camera calibration directory (clear previous calibration first).
	char str[1024], calibDir[1024];
	if(calibrate_both){
		printf("Creating camera calibration directory (overwrites existing data)...\n");
		sprintf(calibDir, "%s\\calib\\cam", sl_params->outdir);
		sprintf(str, "%s\\calib", sl_params->outdir);
		_mkdir(str);
		_mkdir(calibDir);
		sprintf(str, "rd /s /q \"%s\"", calibDir);
		system(str);
		if(_mkdir(calibDir) != 0){
			printf("ERROR: Cannot open output directory!\n");
			printf("Projector-camera calibration was not successful and must be repeated.\n");
			return -1;
		}
	}
	else{
		if(!sl_calib->cam_intrinsic_calib){
			printf("ERROR: Camera must be calibrated first or simultaneously!\n");
			printf("Projector calibration was not successful and must be repeated.\n");
			return -1;	
		}
	}

	// Create projector calibration directory (clear previous calibration first).
	printf("Creating projector calibration directory (overwrites existing data)...\n");
	sprintf(calibDir, "%s\\calib\\proj", sl_params->outdir);
	sprintf(str, "%s\\calib", sl_params->outdir);
	_mkdir(str);
	_mkdir(calibDir);
	sprintf(str, "rd /s /q \"%s\"", calibDir);
	system(str);
	if(_mkdir(calibDir) != 0){
		printf("ERROR: Cannot open output directory!\n");
		if(calibrate_both)
			printf("Projector-camera calibration was not successful and must be repeated.\n");
		else
			printf("Projector calibration was not successful and must be repeated.\n");
		return -1;
	}

	// Prompt user for maximum number of calibration boards.
	printf("Enter the maximum number of calibraiton images, then press return.\n");
	printf("+ Maximum number of images = ");
	int n_boards;
	scanf("%d", &n_boards);
	if(n_boards<2){
		printf("ERROR: At least two images are required!\n");
	    if(calibrate_both)
			printf("Projector-camera calibration was not successful and must be repeated.\n");
		else
			printf("Projector calibration was not successful and must be repeated.\n");
		return -1;
	}
	
	// Evaluate derived camera parameters and allocate storage.
	int cam_board_n            = sl_params->cam_board_w*sl_params->cam_board_h;
	CvSize cam_board_size      = cvSize(sl_params->cam_board_w, sl_params->cam_board_h);
	CvMat* cam_image_points    = cvCreateMat(n_boards*cam_board_n, 2, CV_32FC1);
    CvMat* cam_object_points   = cvCreateMat(n_boards*cam_board_n, 3, CV_32FC1);
    CvMat* cam_point_counts    = cvCreateMat(n_boards, 1, CV_32SC1);
	IplImage** cam_calibImages = new IplImage* [n_boards];

	// Evaluate derived projector parameters and allocate storage.
	int proj_board_n            = sl_params->proj_board_w*sl_params->proj_board_h;
	CvSize proj_board_size      = cvSize(sl_params->proj_board_w, sl_params->proj_board_h);
	CvMat* proj_image_points    = cvCreateMat(n_boards*proj_board_n, 2, CV_32FC1);
    CvMat* proj_image_points2    = cvCreateMat(n_boards*proj_board_n, 2, CV_32FC1);
    CvMat* proj_point_counts    = cvCreateMat(n_boards, 1, CV_32SC1);
	IplImage** proj_calibImages = new IplImage* [n_boards];

	// Generate projector calibration chessboard pattern.
	IplImage* proj_chessboard = cvCreateImage(cvSize(sl_params->proj_w, sl_params->proj_h), IPL_DEPTH_8U, 1);
	int proj_border_cols, proj_border_rows;
	if(generateChessboard(sl_params, proj_chessboard, proj_border_cols, proj_border_rows) == -1){
		if(calibrate_both)
			printf("Projector-camera calibration was not successful and must be repeated.\n");
		else
			printf("Projector calibration was not successful and must be repeated.\n");
		return -1;
	}

	// Generate projector calibration chessboard pattern.
	IplImage* proj_chessboard_sm = cvCreateImage(cvSize(sl_params->proj_w, sl_params->proj_h), IPL_DEPTH_8U, 1);
	int proj_sm_border_cols, proj_sm_border_rows;
	if(generateChessboard(sl_params, proj_chessboard_sm, proj_sm_border_cols, proj_sm_border_rows) == -1){
		if(calibrate_both)
			printf("Projector-camera calibration was not successful and must be repeated.\n");
		else
			printf("Projector calibration was not successful and must be repeated.\n");
		return -1;
	}
	
	// Generate projector calibration sinusoidal pattern.

	// Initialize capture and allocate storage.
	printf("Press 'n' (in 'Camera Correspondences') to capture next image, or 'ESC' to quit.\n");
	IplImage* cam_frame;
    cam_frame = camera->QueryFrame();
	IplImage* cam_frame_1 = cvCreateImage(cvGetSize(cam_frame), cam_frame->depth, cam_frame->nChannels);
	IplImage* cam_frame_2 = cvCreateImage(cvGetSize(cam_frame), cam_frame->depth, cam_frame->nChannels);
	IplImage* cam_frame_3 = cvCreateImage(cvGetSize(cam_frame), cam_frame->depth, cam_frame->nChannels);
	for(int i=0; i<n_boards; i++)
		cam_calibImages[i]  = cvCreateImage(cvGetSize(cam_frame), cam_frame->depth, cam_frame->nChannels);
	for(int i=0; i<n_boards; i++)
		proj_calibImages[i] = cvCreateImage(cvGetSize(cam_frame), cam_frame->depth, cam_frame->nChannels);

	// Create a window to display capture frames.
	cvNamedWindow("Camera Correspondences", CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar("Cam. Gain",  "Camera Correspondences", &sl_params->cam_gain,  100, NULL);
	cvCreateTrackbar("Proj. Gain", "Camera Correspondences", &sl_params->proj_gain, 100, NULL);
	HWND camCorrWindow = (HWND)cvGetWindowHandle("Camera Correspondences");
	BringWindowToTop(camCorrWindow);
	cvWaitKey(1);

	// Create a window to display capture frames.
	cvNamedWindow("Projector Correspondences", CV_WINDOW_AUTOSIZE);
	HWND projCorrWindow = (HWND)cvGetWindowHandle("Projector Correspondences");
	BringWindowToTop(projCorrWindow);
	cvWaitKey(1);

    // Create a window to display projector image.
	IplImage* proj_frame = cvCreateImage(cvSize(sl_params->proj_w, sl_params->proj_h), IPL_DEPTH_8U, 3);
	IplImage* proj_fram_gray = cvCreateImage(cvSize(sl_params->proj_w, sl_params->proj_h), IPL_DEPTH_8U, 1);
	cvSet(proj_frame, cvScalar(255.0, 0.0, 0.0));
    //cvSet(proj_frame, cvScalar(0.0, 0.0, 255.0));
	cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
	cvShowImage("projWindow", proj_frame);
    printf("Press any key to capture\n");
	cvWaitKey(1);
	IplImage* proj_zero = cvCreateImage(cvSize(sl_params->proj_w, sl_params->proj_h), IPL_DEPTH_8U, 1);
    cvZero(proj_zero);

	// Allocate storage for grayscale images.
	IplImage* cam_frame_1_gray = cvCreateImage(cvGetSize(cam_frame), IPL_DEPTH_8U, 1);
	IplImage* cam_frame_2_gray = cvCreateImage(cvGetSize(cam_frame), IPL_DEPTH_8U, 1);
    IplImage* cam_frame_red = cvCreateImage(cvGetSize(cam_frame), IPL_DEPTH_8U, 1);

    // determine projector-camera homography to project the projector checkerboard 
    cvZero(proj_frame);
    cvMerge(proj_chessboard, proj_chessboard, proj_chessboard, NULL, proj_frame);
    cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
	cvShowImage("projWindow", proj_frame);
    cvWaitKey(sl_params->delay);

    bool capturedH = false;

	CvMat* proj_points = cvCreateMat(proj_board_n, 2, CV_32FC1);
    CvMat* proj_sm_points = cvCreateMat(proj_board_n, 2, CV_32FC1);

	// Define image points corresponding to projector chessboard (i.e., considering projector as an inverse camera).
	if(!sl_params->proj_invert){
		for(int j=0; j<proj_board_n; ++j){
			CV_MAT_ELEM(*proj_points, float, j, 0) = 
				sl_params->proj_board_w_pixels*float(j%sl_params->proj_board_w) + (float)proj_border_cols + (float)sl_params->proj_board_w_pixels - (float)0.5;
			CV_MAT_ELEM(*proj_points, float, j, 1) = 
				sl_params->proj_board_h_pixels*float(j/sl_params->proj_board_w) + (float)proj_border_rows + (float)sl_params->proj_board_h_pixels - (float)0.5;
		}
	}
	else{
		for(int j=0; j<proj_board_n; ++j){
			CV_MAT_ELEM(*proj_points, float, j, 0) = 
				sl_params->proj_board_w_pixels*float((proj_board_n-j-1)%sl_params->proj_board_w) + (float)proj_border_cols + (float)sl_params->proj_board_w_pixels - (float)0.5;
			CV_MAT_ELEM(*proj_points, float, j, 1) = 
				sl_params->proj_board_h_pixels*float((proj_board_n-j-1)/sl_params->proj_board_w) + (float)proj_border_rows + (float)sl_params->proj_board_h_pixels - (float)0.5;
		}
	}

    // to do
    // fix hack on projector points
	CvMat* camToProjHomography = cvCreateMat(3, 3, CV_32FC1);
    while(!capturedH)
    {
        cam_frame = camera->QueryFrame();
		cvScale(cam_frame, cam_frame, 2.*(sl_params->cam_gain/100.), 0);

		CvPoint2D32f* cam_corners = new CvPoint2D32f[cam_board_n];
		int cam_corner_count;
		int cam_found =	detectChessboard(cam_frame, proj_board_size, cam_corners, &cam_corner_count);

		cvDrawChessboardCorners(cam_frame, proj_board_size, cam_corners, cam_corner_count, cam_found);
		ShowImageResampled("Camera Correspondences", cam_frame, sl_params->window_w, sl_params->window_h);
        cvWaitKey(1);

		// if we see the projected checkerboard pattern
		if(cam_corner_count == proj_board_n)
        {
			CvMat* cam_src    = cvCreateMat(proj_board_n, 3, CV_32FC1);
			CvMat* cam_dst    = cvCreateMat(proj_board_n, 3, CV_32FC1);
			for(int j=0; j<proj_board_n; ++j){
                CV_MAT_ELEM(*cam_src, float, j, 0) = cam_corners[j].x;
				CV_MAT_ELEM(*cam_src, float, j, 1) = cam_corners[j].y;
				CV_MAT_ELEM(*cam_src, float, j, 2) = 1.0;
				CV_MAT_ELEM(*cam_dst, float, j, 0) = CV_MAT_ELEM(*proj_points, float, j, 0);
				CV_MAT_ELEM(*cam_dst, float, j, 1) = CV_MAT_ELEM(*proj_points, float, j, 1);
				CV_MAT_ELEM(*cam_dst, float, j, 2) = 1.0;
                //printf("Corner: %i cam_corner: %f %f proj_points: %f %f\n", j, cam_corners[j].x, cam_corners[j].y, CV_MAT_ELEM(*proj_points, float, j, 0), CV_MAT_ELEM(*proj_points, float, j, 1));
			}

			cvFindHomography(cam_src, cam_dst, camToProjHomography);
			cvReleaseMat(&cam_src);
			cvReleaseMat(&cam_dst);

            capturedH = true;
        }

        IplImage* cam_warp = cvCreateImage(cvGetSize(cam_frame), IPL_DEPTH_8U, cam_frame->nChannels);
        cvSaveImage("cam_frame.tiff", cam_frame);
        cvWarpPerspective(cam_frame, cam_warp, camToProjHomography);
        cvSaveImage("cam_warp.tiff", cam_warp);

        cvReleaseImage(&cam_frame);
    }

	//cvSet(proj_frame, cvScalar(255.0, 0.0, 0.0));
    cvSet(proj_frame, cvScalar(0.0, 0.0, 255.0));
	cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
	cvShowImage("projWindow", proj_frame);
    cvWaitKey(sl_params->delay);

	CvMat* projToCamHomography = cvCreateMat(3, 3, CV_32FC1);

	// Capture live image stream, until "ESC" is pressed or calibration is complete.
	int successTimer = 0;
    const int numSuccessTimerMax = 3;
    int successes = 0;
	bool captureFrame = false;
	int cvKey = -1, cvKey_temp = -1;
	while(successes < n_boards)
    {
		// Get next available "safe" frame.
        cam_frame = camera->QueryFrameR();
		cvScale(cam_frame, cam_frame, 2.*(sl_params->cam_gain/100.), 0);

        IplImage* cam_frame_BGR = Gray2BGR(cam_frame);
        //cvSplit(cam_frame, NULL, NULL, cam_frame_red, NULL);
        //cvMerge(cam_frame_red, cam_frame_red, cam_frame_red, NULL, cam_frame);

		// Find camera chessboard corners.
		CvPoint2D32f* cam_corners = new CvPoint2D32f[cam_board_n];
		int cam_corner_count;
		//int cam_found =	detectChessboard(cam_frame_red, cam_board_size, cam_corners, &cam_corner_count);
		int cam_found =	detectChessboard(cam_frame, cam_board_size, cam_corners, &cam_corner_count);

        //for(int i = 0; i < cam_board_n; i++)
        //{
        //    printf("cam_corners[%i] = %f, %f\n", i, cam_corners[i].x, cam_corners[i].y);
        //}
		cvDrawChessboardCorners(cam_frame_BGR, cam_board_size, cam_corners, cam_corner_count, cam_found);
		ShowImageResampled("Camera Correspondences", cam_frame_BGR, sl_params->window_w, sl_params->window_h);
        cvReleaseImage(&cam_frame_BGR);

		// If camera chessboard is found, attempt to detect projector chessboard.
		if(cam_corner_count == cam_board_n){

            // calculate projector points
	        CvMat* projToCamHomography = cvCreateMat(3, 3, CV_32FC1);

			CvMat* cam_src    = cvCreateMat(proj_board_n, 3, CV_32FC1);
			CvMat* cam_dst    = cvCreateMat(proj_board_n, 3, CV_32FC1);
			for(int j=0; j<proj_board_n; ++j){
                CV_MAT_ELEM(*cam_src, float, j, 0) = cam_corners[j].x;
				CV_MAT_ELEM(*cam_src, float, j, 1) = cam_corners[j].y;
				CV_MAT_ELEM(*cam_src, float, j, 2) = 1.0;
				CV_MAT_ELEM(*cam_dst, float, j, 0) = CV_MAT_ELEM(*proj_points, float, j, 0);
				CV_MAT_ELEM(*cam_dst, float, j, 1) = CV_MAT_ELEM(*proj_points, float, j, 1);
				CV_MAT_ELEM(*cam_dst, float, j, 2) = 1.0;
			}

			cvFindHomography(cam_dst, cam_src, projToCamHomography);
			cvReleaseMat(&cam_src);
			cvReleaseMat(&cam_dst);

            CvMat* projToProjHomography = cvCreateMat(3, 3, CV_32FC1);
            cvMatMul(camToProjHomography, projToCamHomography, projToProjHomography);

            // Red light checkerboard detection successful
            ostringstream os;
            os << "CameraImage" << 5*successes << ".png";
            cvSaveImage(os.str().c_str(), cam_frame);

            // Get a white checkboard image
            //cvSet(proj_frame, cvScalar(255.0, 255.0, 255.0));
            cvSet(proj_frame, cvScalar(255.0, 255.0, 255.0));

	        //cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
			cvShowImage("projWindow", proj_frame);

			// Get next available "safe" frame (after appropriate delay).
			cvKey_temp = cvWaitKey(sl_params->delay);
			if(cvKey_temp != -1) 
				cvKey = cvKey_temp;
		    // Get next available "safe" frame.
            cam_frame_1_gray = camera->QueryFrameGray();
			//cvCvtColor(cam_frame_1, cam_frame_1_gray, CV_RGB2GRAY);
            //cvSplit(cam_frame_1, NULL, cam_frame_1_gray, NULL, NULL);
            //cvCopyImage(cam_frame_1, cam_frame_1_gray);

            os.str("");
            os << "CameraImage" << 5*successes+1 << ".png";
            cvSaveImage(os.str().c_str(), cam_frame_1_gray);
			ShowImageResampled("Projector Correspondences", cam_frame_1_gray, sl_params->window_w, sl_params->window_h);


			//cvCopy(proj_chessboard, proj_frame);
            // Display projector chessboard.
            cvZero(proj_frame);
            //cvMerge(proj_zero, proj_chessboard, proj_zero, NULL, proj_frame);
            cvMerge(proj_chessboard, proj_chessboard, proj_chessboard, NULL, proj_frame);

            // warp the input based on the camera checkerboard and the precomputed proCam homography
            
            cvSaveImage("projFrame.png", proj_frame);
            cvSaveImage("cam_frame.png", cam_frame);

            //IplImage* projWarp = cvCreateImage(cvGetSize(proj_frame), IPL_DEPTH_8U, 3);
            //cvWarpPerspective(proj_frame, projWarp, projToCamHomography);
            //cvSaveImage("projWarp.png", projWarp);

            //IplImage* projWarp2 = cvCreateImage(cvGetSize(proj_frame), IPL_DEPTH_8U, 3);
            //cvWarpPerspective(projWarp, projWarp2, camToProjHomography);
            //cvSaveImage("projWarp2.png", projWarp2);

            IplImage* projWarp2 = cvCreateImage(cvGetSize(proj_frame), IPL_DEPTH_8U, 3);
            //cvSet(projWarp2, cvScalar(1.0, 0.0, 1.0));
            cvWarpPerspective(proj_frame, projWarp2, projToProjHomography, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalar(255.0, 255.0, 255.0));
            cvSaveImage("projWarp.png", projWarp2);
            cvScale(projWarp2, projWarp2, 2.*(sl_params->proj_gain/100.), 0);


            //cvWarpPerspective(proj_frame, proj_frame, camToProjHomography);

			cvShowImage("projWindow", projWarp2);
            //cvWaitKey(sl_params->delay);
            //cvSaveImage("projWarp.tiff", projWarp);

			// Get next available "safe" frame (after appropriate delay).
			cvKey_temp = cvWaitKey(sl_params->delay);
			if(cvKey_temp != -1) 
				cvKey = cvKey_temp;
		    // Get next available "safe" frame.
            cam_frame_2_gray = camera->QueryFrameGray();

            ShowImageResampled("Projector Correspondences", cam_frame_2_gray, sl_params->window_w, sl_params->window_h);

            //cvCopyImage(cam_frame, cam_frame_2);

            //cvCopyImage(cam_frame, cam_frame_2_gray);
            //cvCopyImage(cam_frame, cam_frame_2);
            //cvCvtColor(cam_frame, cam_frame_1_gray, CV_RGB2GRAY);
            //cvSplit(cam_frame, NULL, cam_frame_1_gray, NULL, NULL);
            //cvShowImageResampled("Camera Correspondences", cam_frame_2_gray, sl_params->window_w, sl_params->window_h);
            os.str("");
            os << "CameraImage" << 5*successes+2 << ".png";
            cvSaveImage(os.str().c_str(), cam_frame_2_gray);

			//cvScale(cam_frame, cam_frame, 2.*(sl_params->cam_gain/100.), 0);
			//cvCopyImage(cam_frame, cam_frame_2);
			//cvCopyImage(cam_frame, cam_frame_3);

			// Convert frames to grayscale and apply background subtraction.
			//cvCvtColor(cam_frame_1, cam_frame_1_gray, CV_RGB2GRAY);
			//cvCvtColor(cam_frame_2, cam_frame_2_gray, CV_RGB2GRAY);
            //cvSplit(cam_frame_1, NULL, cam_frame_1_gray, NULL, NULL);
            //cvSplit(cam_frame_2, NULL, cam_frame_2_gray, NULL, NULL);
            cvSub(cam_frame_1_gray, cam_frame_2_gray, cam_frame_2_gray);

            os.str("");
            os << "CameraImage" << 5*successes+3 << ".png";
            cvSaveImage(os.str().c_str(), cam_frame_2_gray);

			// Invert chessboard image.
			double min_val, max_val;
			cvMinMaxLoc(cam_frame_2_gray, &min_val, &max_val);
			cvConvertScale(cam_frame_2_gray, cam_frame_2_gray, 
				-255.0/(max_val-min_val), 255.0+((255.0*min_val)/(max_val-min_val)));

			// Find projector chessboard corners.
			CvPoint2D32f* proj_corners = new CvPoint2D32f[proj_board_n];
			int proj_corner_count;
			int proj_found = detectChessboard(cam_frame_2_gray, proj_board_size, proj_corners, &proj_corner_count);



            //printPoints(proj_corners, proj_board_n, "proj_corners");

			// Display current projector tracking results.
            //cvMerge(cam_frame_2_gray, cam_frame_2_gray, cam_frame_2_gray, NULL, cam_frame_2);
            IplImage* cam_frame_BGR = Gray2BGR(cam_frame_2_gray);

			cvDrawChessboardCorners(cam_frame_BGR, proj_board_size, proj_corners, proj_corner_count, proj_found);
			ShowImageResampled("Projector Correspondences", cam_frame_BGR, sl_params->window_w, sl_params->window_h);
            cvReleaseImage(&cam_frame_BGR);

            os.str("");
            os << "CameraImage" << 5*successes+4 << ".png";
            cvSaveImage(os.str().c_str(), cam_frame_2_gray);

            //if(proj_corner_count == proj_board_n)
            //{
            //    successTimer++;
            //    printf("+\n");
            //}
            //else
            //{
            //    successTimer = 0;
            //    printf("-\n");
            //}

			// If chessboard is detected, then update calibration lists.
			//if(captureFrame & (proj_corner_count == proj_board_n)){
			//if(successTimer > numSuccessTimerMax)

            if(proj_corner_count == proj_board_n)
            //if(0)
            {
                printf("Press any key to save results or c to cancel this round\n");
	            int key = cvWaitKey(0);
	            if(key=='c')
                {
		            // Display red image for next camera capture frame.
		            cvSet(proj_frame, cvScalar(0.0, 0.0, 255.0));
		            cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
		            cvShowImage("projWindow", proj_frame);

		            cvKey_temp = cvWaitKey(sl_params->delay);
		            if(cvKey_temp != -1) 
			            cvKey = cvKey_temp;

		            continue;
                }
				// Add camera calibration data.
				for(int i=successes*cam_board_n, j=0; j<cam_board_n; ++i,++j){
					CV_MAT_ELEM(*cam_image_points,  float, i, 0) = cam_corners[j].x;
					CV_MAT_ELEM(*cam_image_points,  float, i, 1) = cam_corners[j].y;
					CV_MAT_ELEM(*cam_object_points, float, i, 0) = sl_params->cam_board_w_mm*float(j/sl_params->cam_board_w);
					CV_MAT_ELEM(*cam_object_points, float, i, 1) = sl_params->cam_board_h_mm*float(j%sl_params->cam_board_w);
					CV_MAT_ELEM(*cam_object_points, float, i, 2) = 0.0f;
				}
				CV_MAT_ELEM(*cam_point_counts, int, successes, 0) = cam_board_n;
				cvCopyImage(cam_frame_1, cam_calibImages[successes]);

				// Add projector calibration data.
				//for(int i=successes*proj_board_n, j=0; j<proj_board_n; ++i,++j){
				//	CV_MAT_ELEM(*proj_image_points, float, i, 0) = proj_corners[j].x;
				//	CV_MAT_ELEM(*proj_image_points, float, i, 1) = proj_corners[j].y;
				//}
				//CV_MAT_ELEM(*proj_point_counts, int, successes, 0) = proj_board_n;

                // define projector points
                for(int j=0; j<proj_board_n; ++j){
                    CvMat* p_x = cvCreateMat(3, 1, CV_32FC1);
                    CvMat* p_x_h = cvCreateMat(3, 1, CV_32FC1);

                    CV_MAT_ELEM(*p_x, float, 0, 0) = CV_MAT_ELEM(*proj_points, float, j, 0);
                    CV_MAT_ELEM(*p_x, float, 1, 0) = CV_MAT_ELEM(*proj_points, float, j, 1);
                    CV_MAT_ELEM(*p_x, float, 2, 0) = 1.0;

                    //printf("p_x: %f %f\n", CV_MAT_ELEM(*p_x, float, 0, 0), CV_MAT_ELEM(*p_x, float, 1, 0));

                    cvMatMul(projToProjHomography, p_x, p_x_h);


			        //		CV_MAT_ELEM(*proj_image_points2, float, proj_board_n*i+j, 0) = 
			        //			sl_params->proj_board_w_pixels*float(j%sl_params->proj_board_w) + (float)proj_border_cols + (float)sl_params->proj_board_w_pixels - (float)0.5;
			        //		CV_MAT_ELEM(*proj_image_points2, float, proj_board_n*i+j, 1) = 
			        //			sl_params->proj_board_h_pixels*float(j/sl_params->proj_board_w) + (float)proj_border_rows + (float)sl_params->proj_board_h_pixels - (float)0.5;

                    float p_x_h_x = CV_MAT_ELEM(*p_x_h, float, 0, 0) / CV_MAT_ELEM(*p_x_h, float, 2, 0);
                    float p_x_h_y = CV_MAT_ELEM(*p_x_h, float, 1, 0) / CV_MAT_ELEM(*p_x_h, float, 2, 0);

                    //printf("p_x_h: %f %f\n", p_x_h_x, p_x_h_y);

                    CV_MAT_ELEM(*proj_image_points2, float, proj_board_n*successes+j, 0) = p_x_h_x;
                    CV_MAT_ELEM(*proj_image_points2, float, proj_board_n*successes+j, 1) = p_x_h_y;

                    cvReleaseMat(&p_x);
                    cvReleaseMat(&p_x_h);
			    }
				// Add projector calibration data.
				for(int i=successes*proj_board_n, j=0; j<proj_board_n; ++i,++j){
					CV_MAT_ELEM(*proj_image_points, float, i, 0) = proj_corners[j].x;
					CV_MAT_ELEM(*proj_image_points, float, i, 1) = proj_corners[j].y;
				}
                CV_MAT_ELEM(*proj_point_counts, int, successes, 0) = proj_board_n;

				cvCopyImage(cam_frame_2, proj_calibImages[successes]);

				// Update display.
				successes++;
				printf("*%d Captured frame %d of %d.\n",successes,successes,n_boards);
				captureFrame = false;

                successTimer = 0;

				cvWaitKey(sl_params->delay);
			}

			// Free allocated resources.
			delete[] proj_corners;

			// Display red image for next camera capture frame.
			cvSet(proj_frame, cvScalar(0.0, 0.0, 255.0));
			cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
			cvShowImage("projWindow", proj_frame);  
			cvKey_temp = cvWaitKey(sl_params->delay);
			if(cvKey_temp != -1) 
				cvKey = cvKey_temp;
		}
		else{
			// Display red image for next camera capture frame.
			cvSet(proj_frame, cvScalar(0.0, 0.0, 255.0));
			cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
			cvShowImage("projWindow", proj_frame);

         //   // determine projector-camera homography to project the projector checkerboard 
         //   cvZero(proj_frame);
         //   cvMerge(proj_chessboard, proj_chessboard, proj_chessboard, NULL, proj_frame);
         //   cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
	        //cvShowImage("projWindow", proj_frame);

            cvKey_temp = cvWaitKey(sl_params->delay);
			if(cvKey_temp != -1) 
				cvKey = cvKey_temp;
		}

		// Free allocated resources.
		delete[] cam_corners;

		// Process user input.
        //printf("Press any key to capture\n");
        cvKey_temp = cvWaitKey(sl_params->delay);
		if(cvKey_temp != -1)
			cvKey = cvKey_temp;
		if(cvKey==27)
			break;
		else if(cvKey=='n')
			captureFrame = true;
		cvKey_temp = -1;
		cvKey = -1;
	}

	// Close the display window.
	cvDestroyWindow("Camera Correspondences");

	// Calibrate projector, if minimum number of frames are available.
	if(successes >= 2){
		
		// Allocate calibration matrices.
		CvMat* cam_object_points2       = cvCreateMat(successes*cam_board_n, 3, CV_32FC1);
		CvMat* cam_image_points2        = cvCreateMat(successes*cam_board_n, 2, CV_32FC1);
		CvMat* cam_point_counts2        = cvCreateMat(successes, 1, CV_32SC1);
	    CvMat* cam_rotation_vectors     = cvCreateMat(successes, 3, CV_32FC1);
  	    CvMat* cam_translation_vectors  = cvCreateMat(successes, 3, CV_32FC1);
		CvMat* proj_object_points2      = cvCreateMat(successes*proj_board_n, 3, CV_32FC1);
		//CvMat* proj_image_points2       = cvCreateMat(successes*proj_board_n, 2, CV_32FC1);
		CvMat* proj_point_counts2       = cvCreateMat(successes, 1, CV_32SC1);
	    CvMat* proj_rotation_vectors    = cvCreateMat(successes, 3, CV_32FC1);
  	    CvMat* proj_translation_vectors = cvCreateMat(successes, 3, CV_32FC1);

		// Transfer camera calibration data from captured values.
		for(int i=0; i<successes*cam_board_n; ++i){
			CV_MAT_ELEM(*cam_image_points2,  float, i, 0) = CV_MAT_ELEM(*cam_image_points,  float, i, 0);
			CV_MAT_ELEM(*cam_image_points2,  float, i, 1) = CV_MAT_ELEM(*cam_image_points,  float, i, 1);
			CV_MAT_ELEM(*cam_object_points2, float, i, 0) =	CV_MAT_ELEM(*cam_object_points, float, i, 0);
			CV_MAT_ELEM(*cam_object_points2, float, i, 1) =	CV_MAT_ELEM(*cam_object_points, float, i, 1);
			CV_MAT_ELEM(*cam_object_points2, float, i, 2) = CV_MAT_ELEM(*cam_object_points, float, i, 2);
		}
		for(int i=0; i<successes; ++i)
			CV_MAT_ELEM(*cam_point_counts2, int, i, 0) = CV_MAT_ELEM(*cam_point_counts, int, i, 0);

		// Calibrate the camera and save calibration parameters (if camera calibration is enabled).
		if(calibrate_both){
			printf("Calibrating camera...\n");
			int calib_flags = 0;
			if(!sl_params->cam_dist_model[0])
				calib_flags |= CV_CALIB_ZERO_TANGENT_DIST;
			if(!sl_params->cam_dist_model[1]){
				cvmSet(sl_calib->cam_distortion, 4, 0, 0);
				calib_flags |= CV_CALIB_FIX_K3;
			}
			double camCalibrationError = cvCalibrateCamera2(cam_object_points2, cam_image_points2, cam_point_counts2, 
				cvSize(sl_params->cam_w, sl_params->cam_h), 
				sl_calib->cam_intrinsic, sl_calib->cam_distortion,
				cam_rotation_vectors, cam_translation_vectors, calib_flags);
            printf("***Camera Calibration succeeded with error: %f\n", camCalibrationError);

            CvMat* camCalibrationErrorMat = cvCreateMat(1, 1, CV_32FC1);
            camCalibrationErrorMat->data.fl[0] = camCalibrationError;
            sprintf(str, "%s\\calibrationError.xml", calibDir);
            cvSave(str, camCalibrationErrorMat);
            cvReleaseMat(&camCalibrationErrorMat);

			printf("Saving calibration images and parameters...\n");
			sprintf(calibDir, "%s\\calib\\cam", sl_params->outdir);
			CvMat* r = cvCreateMat(1, 3, CV_32FC1);
			for(int i=0; i<successes; ++i){
				sprintf(str,"%s\\%0.2d.png", calibDir, i);
				cvSaveImage(str, cam_calibImages[i]);
			}
			cvGetRow(cam_rotation_vectors, sl_calib->cam_rot_vec, successes-1);
            cvGetRow(cam_translation_vectors, sl_calib->cam_trans, successes-1);
			cvRodrigues2(sl_calib->cam_rot_vec, sl_calib->cam_rot_mat, NULL);

            sprintf(str,"%s\\cam_object_points2.xml", calibDir);	
			cvSave(str, cam_object_points2);
            sprintf(str,"%s\\cam_image_points2.xml", calibDir);	
			cvSave(str, cam_image_points2);
			sprintf(str,"%s\\cam_intrinsic.xml", calibDir);	
			cvSave(str, sl_calib->cam_intrinsic);
			sprintf(str,"%s\\cam_distortion.xml", calibDir);
			cvSave(str, sl_calib->cam_distortion);
			sprintf(str,"%s\\cam_rotation_vectors.xml", calibDir);
			cvSave(str, sl_calib->cam_rot_vec);
			sprintf(str,"%s\\cam_translation_vectors.xml", calibDir);
			cvSave(str, sl_calib->cam_trans);
			cvReleaseMat(&r);
			sl_calib->cam_intrinsic_calib = true;
		}

		// Transfer projector calibration data from captured values.
		for(int i=0; i<successes; ++i){
 
			//// Define image points corresponding to projector chessboard (i.e., considering projector as an inverse camera).
			//if(!sl_params->proj_invert){
			//	for(int j=0; j<proj_board_n; ++j){
			//		CV_MAT_ELEM(*proj_image_points2, float, proj_board_n*i+j, 0) = 
			//			sl_params->proj_board_w_pixels*float(j%sl_params->proj_board_w) + (float)proj_border_cols + (float)sl_params->proj_board_w_pixels - (float)0.5;
			//		CV_MAT_ELEM(*proj_image_points2, float, proj_board_n*i+j, 1) = 
			//			sl_params->proj_board_h_pixels*float(j/sl_params->proj_board_w) + (float)proj_border_rows + (float)sl_params->proj_board_h_pixels - (float)0.5;
			//	}
			//}
			//else{
			//	for(int j=0; j<proj_board_n; ++j){
			//		CV_MAT_ELEM(*proj_image_points2, float, proj_board_n*i+j, 0) = 
			//			sl_params->proj_board_w_pixels*float((proj_board_n-j-1)%sl_params->proj_board_w) + (float)proj_border_cols + (float)sl_params->proj_board_w_pixels - (float)0.5;
			//		CV_MAT_ELEM(*proj_image_points2, float, proj_board_n*i+j, 1) = 
			//			sl_params->proj_board_h_pixels*float((proj_board_n-j-1)/sl_params->proj_board_w) + (float)proj_border_rows + (float)sl_params->proj_board_h_pixels - (float)0.5;
			//	}
			//}

            //printMatrix(proj_image_points2, "proj_image_points2");

			// Evaluate undistorted image pixels for both the camera and the projector chessboard corners.
			CvMat* cam_dist_image_points    = cvCreateMat(cam_board_n,  1, CV_32FC2);
			CvMat* cam_undist_image_points  = cvCreateMat(cam_board_n,  1, CV_32FC2);
			CvMat* proj_dist_image_points   = cvCreateMat(proj_board_n, 1, CV_32FC2);
			CvMat* proj_undist_image_points = cvCreateMat(proj_board_n, 1, CV_32FC2);
			for(int j=0; j<cam_board_n; ++j)
				cvSet1D(cam_dist_image_points, j, 
					cvScalar(CV_MAT_ELEM(*cam_image_points, float, cam_board_n*i+j, 0), 
					         CV_MAT_ELEM(*cam_image_points, float, cam_board_n*i+j, 1)));
			for(int j=0; j<proj_board_n; ++j)
				cvSet1D(proj_dist_image_points, j, 
					cvScalar(CV_MAT_ELEM(*proj_image_points, float, proj_board_n*i+j, 0), 
					         CV_MAT_ELEM(*proj_image_points, float, proj_board_n*i+j, 1)));
			cvUndistortPoints(cam_dist_image_points, cam_undist_image_points, 
				sl_calib->cam_intrinsic, sl_calib->cam_distortion, NULL, NULL);
			cvUndistortPoints(proj_dist_image_points, proj_undist_image_points, 
				sl_calib->cam_intrinsic, sl_calib->cam_distortion, NULL, NULL);
			cvReleaseMat(&cam_dist_image_points);
			cvReleaseMat(&proj_dist_image_points);

            //for(int i = 0; i < cam_board_n; i++)
            //{
            //    printf("cam_undist_image_points[%i] = %f, %f\n", i, cvGet2D(cam_undist_image_points, i, 0).val[0], cvGet2D(cam_undist_image_points, i, 0).val[1]);
            //}

			// Estimate homography that maps undistorted image pixels to positions on the chessboard.
			CvMat* homography = cvCreateMat(3, 3, CV_32FC1);
			CvMat* cam_src    = cvCreateMat(cam_board_n, 3, CV_32FC1);
			CvMat* cam_dst    = cvCreateMat(cam_board_n, 3, CV_32FC1);
			for(int j=0; j<cam_board_n; ++j){
				CvScalar pd = cvGet1D(cam_undist_image_points, j);
				CV_MAT_ELEM(*cam_src, float, j, 0) = (float)pd.val[0];
				CV_MAT_ELEM(*cam_src, float, j, 1) = (float)pd.val[1];
				CV_MAT_ELEM(*cam_src, float, j, 2) = 1.0;
				CV_MAT_ELEM(*cam_dst, float, j, 0) = CV_MAT_ELEM(*cam_object_points, float, cam_board_n*i+j, 0);
				CV_MAT_ELEM(*cam_dst, float, j, 1) = CV_MAT_ELEM(*cam_object_points, float, cam_board_n*i+j, 1);
				CV_MAT_ELEM(*cam_dst, float, j, 2) = 1.0;
			}
			cvReleaseMat(&cam_undist_image_points);
			cvFindHomography(cam_src, cam_dst, homography);
			cvReleaseMat(&cam_src);
			cvReleaseMat(&cam_dst);

			// Map undistorted projector image corners to positions on the chessboard plane.
			CvMat* proj_src = cvCreateMat(proj_board_n, 1, CV_32FC2);
			CvMat* proj_dst = cvCreateMat(proj_board_n, 1, CV_32FC2);
			for(int j=0; j<proj_board_n; j++)
				cvSet1D(proj_src, j, cvGet1D(proj_undist_image_points, j));
			cvReleaseMat(&proj_undist_image_points);
			cvPerspectiveTransform(proj_src, proj_dst, homography);
			cvReleaseMat(&homography);
			cvReleaseMat(&proj_src);
			
			// Define object points corresponding to projector chessboard.
			for(int j=0; j<proj_board_n; j++){
				CvScalar pd = cvGet1D(proj_dst, j);
				CV_MAT_ELEM(*proj_object_points2, float, proj_board_n*i+j, 0) = (float)pd.val[0];
				CV_MAT_ELEM(*proj_object_points2, float, proj_board_n*i+j, 1) = (float)pd.val[1];
				CV_MAT_ELEM(*proj_object_points2, float, proj_board_n*i+j, 2) = 0.0f;
			}
			cvReleaseMat(&proj_dst); 

            //printMatrix(proj_object_points2, "proj_object_points2");
		}
		for(int i=0; i<successes; ++i)
			CV_MAT_ELEM(*proj_point_counts2, int, i, 0) = CV_MAT_ELEM(*proj_point_counts, int, i, 0);

		// Calibrate the projector and save calibration parameters (if camera calibration is enabled).
		printf("Calibrating projector...\n");
		int calib_flags = 0;
		if(!sl_params->proj_dist_model[0])
			calib_flags |= CV_CALIB_ZERO_TANGENT_DIST;
		if(!sl_params->proj_dist_model[1]){
			cvmSet(sl_calib->proj_distortion, 4, 0, 0);
			calib_flags |= CV_CALIB_FIX_K3;
		}
		double projCalibrationError = cvCalibrateCamera2(
			proj_object_points2, proj_image_points2, proj_point_counts2, 
			cvSize(sl_params->proj_w, sl_params->proj_h), 
			sl_calib->proj_intrinsic, sl_calib->proj_distortion,
			proj_rotation_vectors, proj_translation_vectors, calib_flags);

        // Create projector extrinsics with the camera as the origin
        //  instead of the final calibration target as the origin
        //CvMat* proj_extrinsic_cam_ref = cvCreateMat(


        printf("***Projector Calibration succeeded with error: %f\n", projCalibrationError);
        sprintf(str, "%s\\projCalibrationError.xml", calibDir);
        CvMat* projCalibrationErrorMat = cvCreateMat(1, 1, CV_32FC1);
        projCalibrationErrorMat->data.fl[0] = projCalibrationError;
		cvSave(str, projCalibrationErrorMat);
        cvReleaseMat(&projCalibrationErrorMat);

		printf("Saving calibration images and parameters...\n");
		sprintf(calibDir, "%s\\calib\\proj", sl_params->outdir);
		CvMat* r = cvCreateMat(1, 3, CV_32FC1);
		for(int i=0; i<successes; ++i){
			sprintf(str,"%s\\%0.2d.png", calibDir, i);
			cvSaveImage(str, proj_calibImages[i]);
			sprintf(str,"%s\\%0.2db.png", calibDir, i);
			cvSaveImage(str, cam_calibImages[i]);
			//cvSave(str, R);
		}
		cvGetRow(proj_rotation_vectors, sl_calib->proj_rot_vec, successes-1);
		cvGetRow(proj_translation_vectors, sl_calib->proj_trans, successes-1);
		cvRodrigues2(sl_calib->proj_rot_vec, sl_calib->proj_rot_mat, NULL);

        sprintf(str,"%s\\proj_intrinsic.xml", calibDir);	
		cvSave(str, sl_calib->proj_intrinsic);
		sprintf(str,"%s\\proj_distortion.xml", calibDir);
		cvSave(str, sl_calib->proj_distortion);
		sprintf(str,"%s\\proj_rotation_vectors.xml", calibDir);
		cvSave(str, proj_rotation_vectors);
		sprintf(str,"%s\\proj_translation_vectors.xml", calibDir);
		cvSave(str, proj_translation_vectors);

		// Save the camera calibration parameters (in case camera is recalibrated).
		//sprintf(calibDir, "%s\\calib\\proj", sl_params->outdir);
		//for(int i=0; i<successes; ++i){
		//	cvGetRow(cam_rotation_vectors, r, i);
		//	cvRodrigues2(r, R, NULL);
		//	sprintf(str,"%s\\cam_rotation_matrix_%0.2d.xml", calibDir, i);
		//	//cvSave(str, R);
		//}
		sprintf(str,"%s\\cam_intrinsic.xml", calibDir);	
		cvSave(str, sl_calib->cam_intrinsic);
		sprintf(str,"%s\\cam_distortion.xml", calibDir);
		cvSave(str, sl_calib->cam_distortion);
		sprintf(str,"%s\\cam_rotation_vectors.xml", calibDir);
		cvSave(str, cam_rotation_vectors);
		sprintf(str,"%s\\cam_translation_vectors.xml", calibDir);
		cvSave(str, cam_translation_vectors);
        CvMat* cam_dim = cvCreateMat(2, 1, CV_32FC1);
        cvmSet(cam_dim, 0, 0, sl_params->cam_w);
        cvmSet(cam_dim, 1, 0, sl_params->cam_h);
		sprintf(str,"%s\\cam_dimensions.xml", calibDir);
        cvSave(str, cam_dim);
        cvReleaseMat(&cam_dim);

		//// Calculate the fundamental matrix between the two elements.
		//sl_calib->fundMatrx->ComputeFundamentalMatrix();
		//CvMat* fundMat = sl_calib->fundMatrx->GetMatrix();
		//printf("Fund Mat: ");
		//for(int row = 0; row < 3; row++)
		//{
		//	for(int col = 0; col < 3; col++)
		//	{
		//		printf("%f ", cvmGet(fundMat, row, col));
		//	}
		//}
		//printf("\n");

		// Save extrinsic calibration of projector-camera system.
		// Note: First calibration image is used to define extrinsic calibration.
		CvMat* cam_object_points_00      = cvCreateMat(cam_board_n, 3, CV_32FC1);
		CvMat* cam_image_points_00       = cvCreateMat(cam_board_n, 2, CV_32FC1);
		CvMat* cam_rotation_vector_00    = cvCreateMat(1, 3, CV_32FC1);
  	    CvMat* cam_translation_vector_00 = cvCreateMat(1, 3, CV_32FC1);
		if(!calibrate_both){
			for(int i=0; i<cam_board_n; ++i){
				CV_MAT_ELEM(*cam_image_points_00,  float, i, 0) = CV_MAT_ELEM(*cam_image_points2,  float, i, 0);
				CV_MAT_ELEM(*cam_image_points_00,  float, i, 1) = CV_MAT_ELEM(*cam_image_points2,  float, i, 1);
				CV_MAT_ELEM(*cam_object_points_00, float, i, 0) = CV_MAT_ELEM(*cam_object_points2, float, i, 0);
				CV_MAT_ELEM(*cam_object_points_00, float, i, 1) = CV_MAT_ELEM(*cam_object_points2, float, i, 1);
				CV_MAT_ELEM(*cam_object_points_00, float, i, 2) = CV_MAT_ELEM(*cam_object_points2, float, i, 2);
			}
			cvFindExtrinsicCameraParams2(
				cam_object_points_00, cam_image_points_00, 
				sl_calib->cam_intrinsic, sl_calib->cam_distortion,
				cam_rotation_vector_00, cam_translation_vector_00);
			for(int i=0; i<3; i++)
				CV_MAT_ELEM(*sl_calib->cam_extrinsic, float, 0, i) = (float)cvmGet(cam_rotation_vector_00, 0, i);
			for(int i=0; i<3; i++)
				CV_MAT_ELEM(*sl_calib->cam_extrinsic, float, 1, i) = (float)cvmGet(cam_translation_vector_00, 0, i);
		}
		else{
			for(int i=0; i<3; i++)
				CV_MAT_ELEM(*sl_calib->cam_extrinsic, float, 0, i) = (float)cvmGet(cam_rotation_vectors, 0, i);
			for(int i=0; i<3; i++)
				CV_MAT_ELEM(*sl_calib->cam_extrinsic, float, 1, i) = (float)cvmGet(cam_translation_vectors, 0, i);
		}
		sprintf(str, "%s\\cam_extrinsic.xml", calibDir);
		cvSave(str, sl_calib->cam_extrinsic);
		//sprintf(str, "%s\\fundamental_matrix.xml", calibDir);
		//cvSave(str, sl_calib->fundMatrx->GetMatrix());

		for(int i=0; i<3; i++)
			CV_MAT_ELEM(*sl_calib->proj_extrinsic, float, 0, i) = (float)cvmGet(proj_rotation_vectors, 0, i);
		for(int i=0; i<3; i++)
			CV_MAT_ELEM(*sl_calib->proj_extrinsic, float, 1, i) = (float)cvmGet(proj_translation_vectors, 0, i);
		sprintf(str, "%s\\proj_extrinsic.xml", calibDir);
		cvSave(str, sl_calib->proj_extrinsic);

        // get the projector in the coordinate system of the camera

        // Camera extrinsic matrix
    
        // convert to the new matrix representation
        Mat cam_rot_mat2(sl_calib->cam_rot_mat);
        PrintMatrix("cam_rot_mat2", cam_rot_mat2);
        Mat cam_trans2(sl_calib->cam_trans);
        PrintMatrix("cam_trans2", cam_trans2);

        // Mat cam_ext_mat
        // r11 r12 r13  t1
        // r21 r22 r23  t2
        // r31 r32 r33  t3
        //   0   0   0   1   
        Mat cam_ext_mat = Mat::zeros(4, 4, CV_32F);
        cam_ext_mat.at<float>(0,0) = cam_rot_mat2.at<float>(0,0);
        cam_ext_mat.at<float>(0,1) = cam_rot_mat2.at<float>(0,1);
        cam_ext_mat.at<float>(0,2) = cam_rot_mat2.at<float>(0,2);
        cam_ext_mat.at<float>(1,0) = cam_rot_mat2.at<float>(1,0);
        cam_ext_mat.at<float>(1,1) = cam_rot_mat2.at<float>(1,1);
        cam_ext_mat.at<float>(1,2) = cam_rot_mat2.at<float>(1,2);
        cam_ext_mat.at<float>(2,0) = cam_rot_mat2.at<float>(2,0);
        cam_ext_mat.at<float>(2,1) = cam_rot_mat2.at<float>(2,1);
        cam_ext_mat.at<float>(2,2) = cam_rot_mat2.at<float>(2,2);

        cam_ext_mat.at<float>(0,3) = cam_trans2.at<float>(0,0);
        cam_ext_mat.at<float>(1,3) = cam_trans2.at<float>(1,0);
        cam_ext_mat.at<float>(2,3) = cam_trans2.at<float>(2,0);

        cam_ext_mat.at<float>(3,0) = 0;
        cam_ext_mat.at<float>(3,1) = 0;
        cam_ext_mat.at<float>(3,2) = 0;
        cam_ext_mat.at<float>(3,3) = 1;
        PrintMatrix("cam_ext_mat", cam_ext_mat);

        Mat cam_ext_mat_inv(4, 4, CV_32F);
        invert(cam_ext_mat, cam_ext_mat_inv);
        PrintMatrix("cam_ext_mat_inv", cam_ext_mat_inv);

        // projector extrinsic matrix

        // convert to the new matrix representation
        Mat proj_rot_mat2(sl_calib->proj_rot_mat);
        Mat proj_trans2(sl_calib->proj_trans);

        // Mat proj_ext_mat
        // r11 r12 r13  t1
        // r21 r22 r23  t2
        // r31 r32 r33  t3
        //   0   0   0   1   
        Mat proj_ext_mat = Mat::zeros(4, 4, CV_32F);
        proj_ext_mat.at<float>(0,0) = proj_rot_mat2.at<float>(0,0);
        proj_ext_mat.at<float>(0,1) = proj_rot_mat2.at<float>(0,1);
        proj_ext_mat.at<float>(0,2) = proj_rot_mat2.at<float>(0,2);
        proj_ext_mat.at<float>(1,0) = proj_rot_mat2.at<float>(1,0);
        proj_ext_mat.at<float>(1,1) = proj_rot_mat2.at<float>(1,1);
        proj_ext_mat.at<float>(1,2) = proj_rot_mat2.at<float>(1,2);
        proj_ext_mat.at<float>(2,0) = proj_rot_mat2.at<float>(2,0);
        proj_ext_mat.at<float>(2,1) = proj_rot_mat2.at<float>(2,1);
        proj_ext_mat.at<float>(2,2) = proj_rot_mat2.at<float>(2,2);
        
        proj_ext_mat.at<float>(0,3) = proj_trans2.at<float>(0,0);
        proj_ext_mat.at<float>(1,3) = proj_trans2.at<float>(1,0);
        proj_ext_mat.at<float>(2,3) = proj_trans2.at<float>(2,0);
        
        proj_ext_mat.at<float>(3,0) = 0;
        proj_ext_mat.at<float>(3,1) = 0;
        proj_ext_mat.at<float>(3,2) = 0;
        proj_ext_mat.at<float>(3,3) = 1;
        PrintMatrix("cam_ext_mat", proj_ext_mat);
        //proj_ext_mat( Range(0,2), Range(0,2) ) = proj_rot_mat2( Range::all(), Range::all() );
        //proj_ext_mat( Range(0,0), Range(0,2) ) = proj_trans2( Range::all(), Range(0,0) );
        //proj_ext_mat.at<float>(3,3) = 1;
        
        Mat proj_ext_mat_new(4, 4, CV_32F);
        multiply(proj_ext_mat, cam_ext_mat_inv, proj_ext_mat_new);
        PrintMatrix("proj_ext_mat_new", proj_ext_mat_new);
        
		sprintf(str, "%s\\proj_extrinsic_4x4.xml", calibDir);
        CvMat proj_ext_mat_new_cv(proj_ext_mat_new);
		cvSave(str, &proj_ext_mat_new_cv);

        // create the new rot mat 
        Mat proj_rot_mat_new(3, 3, CV_32F);
        proj_rot_mat_new = proj_ext_mat_new( Range(0,3), Range(0,3) );
        
        // create the rotation vectors
        Mat proj_rot_vec_new(1, 3, CV_32F);
        Rodrigues(proj_rot_mat_new, proj_rot_vec_new);

        // create the new translation vectors
        Mat proj_trans_new(3, 1, CV_32F);
        proj_trans_new = proj_ext_mat_new( Range(0,3), Range(3,4) );

        Mat proj_ext_mat_orig_format(2, 3, CV_32F);
        proj_ext_mat_orig_format.at<float>(0,0) = proj_rot_vec_new.at<float>(0,0);
        proj_ext_mat_orig_format.at<float>(0,1) = proj_rot_vec_new.at<float>(0,1);
        proj_ext_mat_orig_format.at<float>(0,2) = proj_rot_vec_new.at<float>(0,2);
        proj_ext_mat_orig_format.at<float>(1,0) = proj_trans_new.at<float>(0,0);
        proj_ext_mat_orig_format.at<float>(1,1) = proj_trans_new.at<float>(0,1);
        proj_ext_mat_orig_format.at<float>(1,2) = proj_trans_new.at<float>(0,2);
        //proj_ext_mat_orig_format( Range(0,0), Range(0,2) ) = proj_rot_vec_new;
        //proj_ext_mat_orig_format( Range(1,1), Range(0,2) ) = proj_trans_new;

		sprintf(str, "%s\\proj_extrinsic_new.xml", calibDir);
        CvMat proj_ext_mat_orig_format_cv(proj_ext_mat_orig_format);
        cvSave(str, &proj_ext_mat_orig_format_cv);

		// Free allocated resources.
		cvReleaseMat(&cam_object_points2);
		cvReleaseMat(&cam_image_points2);
		cvReleaseMat(&cam_point_counts2);
	    cvReleaseMat(&cam_rotation_vectors);
  	    cvReleaseMat(&cam_translation_vectors);
		cvReleaseMat(&proj_object_points2);
		cvReleaseMat(&proj_image_points2);
		cvReleaseMat(&proj_point_counts2);
	    cvReleaseMat(&proj_rotation_vectors);
  	    cvReleaseMat(&proj_translation_vectors);
		//cvReleaseMat(&R);
		cvReleaseMat(&r);
		cvReleaseMat(&cam_object_points_00);
		cvReleaseMat(&cam_image_points_00);
		cvReleaseMat(&cam_rotation_vector_00);
  	    cvReleaseMat(&cam_translation_vector_00);
	}
	else{
		printf("ERROR: At least two detected chessboards are required!\n");
	    if(calibrate_both)
			printf("Projector-camera calibration was not successful and must be repeated.\n");
		else
			printf("Projector calibration was not successful and must be repeated.\n");
		return -1;
	}

	// Update calibration status.
	sl_calib->proj_intrinsic_calib   = true;
	sl_calib->procam_extrinsic_calib = true;

	// Evaluate projector-camera geometry.
	//evaluateProCamGeometry(sl_params, sl_calib);

	// Free allocated resources.
	cvReleaseMat(&proj_points);
	cvReleaseMat(&cam_image_points);
    cvReleaseMat(&cam_object_points);
    cvReleaseMat(&cam_point_counts);
	cvReleaseMat(&proj_image_points);
    cvReleaseMat(&proj_point_counts);
	cvReleaseImage(&proj_chessboard);
	cvReleaseImage(&cam_frame_1);
	cvReleaseImage(&cam_frame_2);
	cvReleaseImage(&cam_frame_3);
	cvReleaseImage(&proj_frame);
    cvReleaseImage(&proj_fram_gray);
	cvReleaseImage(&cam_frame_1_gray);
	cvReleaseImage(&cam_frame_2_gray);
    cvReleaseImage(&cam_frame_red);
    cvReleaseMat(&projToCamHomography);
	for(int i=0; i<n_boards; i++){
		cvReleaseImage(&cam_calibImages[i]);
		cvReleaseImage(&proj_calibImages[i]);
	}
	delete[] cam_calibImages;
	delete[] proj_calibImages;

	// Return without errors.
	if(calibrate_both){
		printf("Projector-camera calibration was successful.\n");
		displayCamCalib(sl_calib);
	}
	else
		printf("Projector calibration was successful.\n");
	displayProjCalib(sl_calib);
	return 0;
}