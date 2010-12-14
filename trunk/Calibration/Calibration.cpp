////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file   Calibration\Calibration.cpp
///
/// @brief  Implements the calibration class. 
///
/// @defgroup Calibration Calibration
///       Main executable for projector-camera calibration.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Common.h"
#include "Calibration.h"
#include "CalibrationExceptions.h"
#include "CalibrateProCam.h"
#include "CameraConfigParams.h"
#include "Configuration.h"
#include "KinectCameraManager.h"
#include "UtilProCam.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @fn int main(int argc, char* argv[])
///
/// @brief  Main entry-point for this application. 
///
/// @author Brett Jones
/// @date   12/12/2010
///
/// @param  argc    Number of command-line arguments. 
/// @param  argv    Array of command-line argument strings. 
///
/// @return Exit-code for the process - 0 for success, else an error code. 
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
    // ***************************************************
    // Load the configuration file
    // ***************************************************
	printf("[Projector-Camera Calibration]\n");
	char configFile[1024];
	if(argc == 1)
		strcpy(configFile, "../../config.xml");
	else
		strcpy(configFile, argv[1]);

	// Read parameters from configuration file.
	struct slParams sl_params;

    Configuration config(std::string(configFile), &sl_params);
    try{
        config.Load();
    }
    catch(FileNotFound &e)
    {
        printf("%s", e.what());
        _getch();
        return -1;
    }

    // ***************************************************
    // Intialize the hardware
    // ***************************************************
    
    // To do: implement camera configuration parameter loading
    // Create blank camera parameters
    CameraConfigParams cameraConfigParams;
    
    KinectCameraManager kinectCameraManager;
    std::vector<Camera*> cameras;
    Camera* camera;
    
    // Initialize cameras
    try
    {
        kinectCameraManager.Init(&cameraConfigParams);

        std::vector<Camera*> cameras = kinectCameraManager.GetCameras();
        if(cameras.size() < 1)
        {
            printf("Camera not found\n");
            return -1;
        }   

        camera = cameras[0];

        // Start Camera Capture
        camera->StartCapture();

        // Get 1st Frame
        IplImage* cam_frame = camera->QueryFrame();
    }
    catch(...)
    {
        return -1;
    }

    CalibrateProCam cvCalibrateProCam(camera);

	// Create fullscreen window (for controlling projector display).
	cvNamedWindow("projWindow", CV_WINDOW_AUTOSIZE);
	IplImage* proj_frame = cvCreateImage(cvSize(sl_params.proj_w, sl_params.proj_h), IPL_DEPTH_8U, 3);
	cvSet(proj_frame, cvScalar(0, 0, 255));
	cvShowImage("projWindow", proj_frame);
	cvMoveWindow("projWindow", -sl_params.proj_w+sl_params.window_offset_x, sl_params.window_offset_y);
	cvWaitKey(1);
	
	// Create output directory (clear previous scan first).
	printf("Creating output directory (overwrites existing object data)...\n");
	char str[1024];
	_mkdir(sl_params.outdir);
	sprintf(str, "%s\\%s", sl_params.outdir, sl_params.object);
	_mkdir(str);
	sprintf(str, "rd /s /q \"%s\\%s\"", sl_params.outdir, sl_params.object);
	system(str);
	sprintf(str, "%s\\%s", sl_params.outdir, sl_params.object);
	if(_mkdir(str) != 0){
		printf("ERROR: Cannot open output directory!\n");
		printf("Press any key to exit.\n");
		_getch();
		return -1;
	}
	
	// Allocate storage for calibration parameters.
	struct slCalib sl_calib;
	int cam_nelems                  = sl_params.cam_w*sl_params.cam_h;
	int proj_nelems                 = sl_params.proj_w*sl_params.proj_h;
    sl_calib.cam_intrinsic_calib    = false;
	sl_calib.proj_intrinsic_calib   = false;
	sl_calib.procam_extrinsic_calib = false;
	sl_calib.cam_intrinsic          = cvCreateMat(3,3,CV_32FC1);
	sl_calib.cam_distortion         = cvCreateMat(5,1,CV_32FC1);
	sl_calib.cam_extrinsic          = cvCreateMat(2, 3, CV_32FC1);
    sl_calib.cam_rot_vec            = cvCreateMat(3, 1, CV_32FC1);
    sl_calib.cam_rot_mat            = cvCreateMat(3, 3, CV_32FC1);
    sl_calib.cam_trans              = cvCreateMat(3, 1, CV_32FC1);
	sl_calib.proj_intrinsic         = cvCreateMat(3, 3, CV_32FC1);
	sl_calib.proj_distortion        = cvCreateMat(5, 1, CV_32FC1);
	sl_calib.proj_extrinsic         = cvCreateMat(2, 3, CV_32FC1);
    sl_calib.proj_rot_vec            = cvCreateMat(3, 1, CV_32FC1);
    sl_calib.proj_rot_mat            = cvCreateMat(3, 3, CV_32FC1);
    sl_calib.proj_trans              = cvCreateMat(3, 1, CV_32FC1);
	sl_calib.cam_center             = cvCreateMat(3, 1, CV_32FC1);
	sl_calib.proj_center            = cvCreateMat(3, 1, CV_32FC1);
	sl_calib.cam_rays               = cvCreateMat(3, cam_nelems, CV_32FC1);
	sl_calib.proj_rays              = cvCreateMat(3, cam_nelems, CV_32FC1);
	//sl_calib.proj_rays              = cvCreateMat(3, proj_nelems, CV_32FC1);
	sl_calib.proj_column_planes     = cvCreateMat(sl_params.proj_w, 4, CV_32FC1);
	sl_calib.proj_row_planes        = cvCreateMat(sl_params.proj_h, 4, CV_32FC1);
	//sl_calib.fundMatrx				= new FundamentalMatrix();

	
	// Load intrinsic camera calibration parameters (if found).
	char str1[1024], str2[1024];
	sprintf(str1, "%s\\calib\\cam\\cam_intrinsic.xml",  sl_params.outdir);
	sprintf(str2, "%s\\calib\\cam\\cam_distortion.xml", sl_params.outdir);
	if( ((CvMat*)cvLoad(str1) != 0) && ((CvMat*)cvLoad(str2) != 0) ){
		sl_calib.cam_intrinsic  = (CvMat*)cvLoad(str1);
		sl_calib.cam_distortion = (CvMat*)cvLoad(str2);
		sl_calib.cam_intrinsic_calib = true;
		printf("Loaded previous intrinsic camera calibration.\n");
	}
	else
		printf("Camera has not been intrinsically calibrated!\n");

	//sprintf(str1, "%s\\calib\\proj\\fundamental_matrix.xml",  sl_params.outdir);
	//if( (CvMat*)cvLoad(str1) != 0 )
	//{
	//	//sl_calib.fundMatrx->SetMatrix((CvMat*)cvLoad(str1));
	//	printf("Loaded previous fundamental matrix.\n");
	//}

	// Load intrinsic projector calibration parameters (if found);
	sprintf(str1, "%s\\calib\\proj\\proj_intrinsic.xml",  sl_params.outdir);
	sprintf(str2, "%s\\calib\\proj\\proj_distortion.xml", sl_params.outdir);
	if( ((CvMat*)cvLoad(str1) != 0) && ((CvMat*)cvLoad(str2) != 0) ){
		sl_calib.proj_intrinsic  = (CvMat*)cvLoad(str1);
		sl_calib.proj_distortion = (CvMat*)cvLoad(str2);
		sl_calib.proj_intrinsic_calib = true;
		printf("Loaded previous intrinsic projector calibration.\n");
	}
	else
		printf("Projector has not been intrinsically calibrated!\n");

	// Load extrinsic projector-camera parameters (if found).
	sprintf(str1, "%s\\calib\\proj\\cam_extrinsic.xml",  sl_params.outdir);
	sprintf(str2, "%s\\calib\\proj\\proj_extrinsic.xml", sl_params.outdir);
	if( (sl_calib.cam_intrinsic_calib && sl_calib.proj_intrinsic_calib) &&
		( ((CvMat*)cvLoad(str1) != 0) && ((CvMat*)cvLoad(str2) != 0) ) ){
		sl_calib.cam_extrinsic  = (CvMat*)cvLoad(str1);
		sl_calib.proj_extrinsic = (CvMat*)cvLoad(str2);
		sl_calib.procam_extrinsic_calib = true;
		//cvCalibrateProCam.evaluateProCamGeometry(&sl_params, &sl_calib);
		printf("Loaded previous extrinsic projector-camera calibration.\n");
	}
	else
		printf("Projector-camera system has not been extrinsically calibrated!\n");

	// Initialize background model.
	sl_calib.background_depth_map = cvCreateMat(sl_params.cam_h, sl_params.cam_w, CV_32FC1);
	sl_calib.background_image     = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U, 3);
	sl_calib.background_mask      = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U, 1);
	cvSet(sl_calib.background_depth_map, cvScalar(FLT_MAX));
	cvZero(sl_calib.background_image);
	cvSet(sl_calib.background_mask, cvScalar(255));

	// Initialize scan counter (used to index each scan iteration).
	int scan_index = 0;

	// Process user input, until 'ESC' is pressed.
	int cvKey = NULL;
	while(1){

		// Display a black projector image by default.
		cvSet(proj_frame, cvScalar(0, 0, 255));
		cvShowImage("projWindow", proj_frame);
		cvWaitKey(1);

		// Parse keystroke.
		if(cvKey == 27){
			printf("\n> Writing configuration file \"%s\"...\n", configFile);
			config.Save();
			printf("> Exiting application...\n");
			break;
		}
		else if(cvKey == 'c'){
			printf("\n> Calibrating camera and projector simultaneously...\n");
			cvCalibrateProCam.runProjectorCalibration(&sl_params, &sl_calib, true);
			config.Save();

            cvKey = NULL;
		}

		// Display prompt.
		if(cvKey == NULL){
			printf("\nPress the following keys for the corresponding functions.\n");
			printf("'C': Calibrate camera and projector simultaneously\n");
			//printf("'E': Calibrate projector-camera alignment\n");
			printf("'ESC': Exit application\n");
		}

		// Get keystroke.
		cvKey = _getch();
	}

    // Destory camera
    camera->EndCapture();
    if(camera)
	{
        delete camera;
	}

	delete sl_calib.fundMatrx;

	// Release allocated resources.
	cvReleaseMat(&sl_calib.cam_intrinsic);
	cvReleaseMat(&sl_calib.cam_distortion);
	cvReleaseMat(&sl_calib.cam_extrinsic);
	cvReleaseMat(&sl_calib.cam_rot_vec);
	cvReleaseMat(&sl_calib.cam_rot_mat);
	cvReleaseMat(&sl_calib.cam_trans);
	cvReleaseMat(&sl_calib.proj_intrinsic);
	cvReleaseMat(&sl_calib.proj_distortion);
	cvReleaseMat(&sl_calib.proj_extrinsic);
	cvReleaseMat(&sl_calib.proj_rot_vec);
	cvReleaseMat(&sl_calib.proj_rot_mat);
	cvReleaseMat(&sl_calib.proj_trans);
	cvReleaseMat(&sl_calib.cam_center);
	cvReleaseMat(&sl_calib.proj_center);
	cvReleaseMat(&sl_calib.cam_rays);
	cvReleaseMat(&sl_calib.proj_rays);
	cvReleaseMat(&sl_calib.proj_column_planes);
	cvReleaseMat(&sl_calib.proj_row_planes);
	cvReleaseImage(&proj_frame);
	cvReleaseMat(&sl_calib.background_depth_map);
	cvReleaseImage(&sl_calib.background_image);
	cvReleaseImage(&sl_calib.background_mask);

	// Exit without errors.
	cvDestroyWindow("projWindow");

    return 0;
}