////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file   Calibration\Configuration.cpp
///
/// @brief  Implements the configuration class. 
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Configuration.h"

#include "Calibration.h"
#include "CalibrationExceptions.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
/// @fn public: Configuration(const char* filename, struct stParams* sl_params)
///
/// @brief  Constructor
///
/// @author Brett Jones
/// @date   12/12/2010
///
/// @param  filename            Configuration filename.
/// @param [in,out] sl_params   Paramaters.
////////////////////////////////////////////////////////////////////////////////////////////////////
Configuration::Configuration(std::string filename, struct slParams* sl_params)
{
    mFilename = filename;
    mSlParams = sl_params;
}

void Configuration::Load()
{
    FILE* pFile = fopen(mFilename.c_str(), "r");
	if(pFile != NULL){
		fclose(pFile);
		printf("Reading configuration file \"%s\"...\n", mFilename);
	}
	else{
        throw new FileNotFound("configuration file");
    }

    // done to reuse existing code
	struct slParams* sl_params = mSlParams;
    
    // Open file storage for XML-formatted configuration file.
	CvFileStorage* fs = cvOpenFileStorage(mFilename.c_str(), 0, CV_STORAGE_READ);

	// Read output directory and object (or sequence) name.
	CvFileNode* m = cvGetFileNodeByName(fs, 0, "output");
	strcpy(sl_params->outdir, cvReadStringByName(fs, m, "output_directory", "./output"));
	strcpy(sl_params->object, cvReadStringByName(fs, m, "object_name", "./output"));
	sl_params->save = (cvReadIntByName(fs, m, "save_intermediate_results", 0) != 0);

	// Read camera parameters.
	m = cvGetFileNodeByName(fs, 0, "camera");
	sl_params->cam_w         =  cvReadIntByName(fs, m, "width",                          960);
	sl_params->cam_h         =  cvReadIntByName(fs, m, "height",                         720);
	sl_params->Logitech_9000 = (cvReadIntByName(fs, m, "Logitech_Quickcam_9000_raw_mode",  0) != 0);

	// Read projector parameters.
	m = cvGetFileNodeByName(fs, 0, "projector");
	sl_params->proj_w      =  cvReadIntByName(fs, m, "width",            1024);
	sl_params->proj_h      =  cvReadIntByName(fs, m, "height",            768);
	sl_params->proj_invert = (cvReadIntByName(fs, m, "invert_projector",    0) != 0);

	// Read camera and projector gain parameters.
	m = cvGetFileNodeByName(fs, 0, "gain");
	sl_params->cam_gain  = cvReadIntByName(fs, m, "camera_gain",     50);
	sl_params->proj_gain = cvReadIntByName(fs, m, "projector_gain",  50);
	
	// Read distortion model parameters.
	m = cvGetFileNodeByName(fs, 0, "distortion_model");
	sl_params->cam_dist_model[0]  = (cvReadIntByName(fs, m, "enable_tangential_camera",          0) != 0);
	sl_params->cam_dist_model[1]  = (cvReadIntByName(fs, m, "enable_6th_order_radial_camera",    0) != 0);
	sl_params->proj_dist_model[0] = (cvReadIntByName(fs, m, "enable_tangential_projector",       0) != 0);
	sl_params->proj_dist_model[1] = (cvReadIntByName(fs, m, "enable_6th_order_radial_projector", 0) != 0);

	// Read camera calibration chessboard parameters.
	m = cvGetFileNodeByName(fs, 0, "camera_chessboard");
	sl_params->cam_board_w    =        cvReadIntByName(fs,  m, "interior_horizontal_corners",    8);
	sl_params->cam_board_h    =        cvReadIntByName(fs,  m, "interior_vertical_corners",      6);
	sl_params->cam_board_w_mm = (float)cvReadRealByName(fs, m, "square_width_mm",             30.0);
	sl_params->cam_board_h_mm = (float)cvReadRealByName(fs, m, "square_height_mm",            30.0);

	// Read projector calibration chessboard parameters.
	m = cvGetFileNodeByName(fs, 0, "projector_chessboard");
	sl_params->proj_board_w        = cvReadIntByName(fs,  m, "interior_horizontal_corners",  8);
	sl_params->proj_board_h        = cvReadIntByName(fs,  m, "interior_vertical_corners",    6);
	sl_params->proj_board_w_pixels = cvReadIntByName(fs, m, "square_width_pixels",          75);
	sl_params->proj_board_h_pixels = cvReadIntByName(fs, m, "square_height_pixels",         75);
	
	// Read scanning and reconstruction parameters.
	m = cvGetFileNodeByName(fs, 0, "scanning_and_reconstruction");
	sl_params->mode                    =         cvReadIntByName(fs,  m, "mode",                               2);
	sl_params->scan_cols               =        (cvReadIntByName(fs,  m, "reconstruct_columns",                1) != 0);
	sl_params->scan_rows               =        (cvReadIntByName(fs,  m, "reconstruct_rows",                   1) != 0);
	sl_params->delay                   =         cvReadIntByName(fs,  m, "frame_delay_ms",                   200);
	sl_params->thresh                  =         cvReadIntByName(fs,  m, "minimum_contrast_threshold",        32);
	sl_params->dist_range[0]           = (float) cvReadRealByName(fs, m, "minimum_distance_mm",              0.0);
	sl_params->dist_range[1]           = (float) cvReadRealByName(fs, m, "maximum_distance_mm",            1.0e4);
	sl_params->dist_reject             = (float) cvReadRealByName(fs, m, "maximum_distance_variation_mm",   10.0);
	sl_params->background_depth_thresh = (float) cvReadRealByName(fs, m, "minimum_background_distance_mm",  20.0);
    sl_params->generate_normals        =        (cvReadIntByName(fs,  m, "generate_normals",                   1) != 0);

	// Read visualization options.
	m = cvGetFileNodeByName(fs, 0, "visualization");
	sl_params->display  = (cvReadIntByName(fs, m, "display_intermediate_results",   1) != 0);
	sl_params->window_w =  cvReadIntByName(fs, m, "display_window_width_pixels",  640);
    sl_params->window_offset_x =  cvReadIntByName(fs, m, "window_offset_x",  -13);
    sl_params->window_offset_y =  cvReadIntByName(fs, m, "window_offset_y",  -23);

	// Enable both row and column scanning, if "ray-ray" reconstruction mode is enabled.
	if(sl_params->mode == 2){
		sl_params->scan_cols = true;
		sl_params->scan_rows = true;
	}

	// Set camera visualization window dimensions.
	sl_params->window_h = (int)ceil((float)sl_params->window_w*((float)sl_params->cam_h/(float)sl_params->cam_w));

	// Close file storage for XML-formatted configuration file.
	cvReleaseFileStorage(&fs);
}

void Configuration::Save()
{
    FILE* pFile = fopen(mFilename.c_str(), "r");
	if(pFile != NULL){
		fclose(pFile);
		printf("Reading configuration file \"%s\"...\n", mFilename);
	}
	else{
        throw new FileNotFound("configuration file");
    }

    // done to reuse existing code
	struct slParams* sl_params = mSlParams;

	// Create file storage for XML-formatted configuration file.
    CvFileStorage* fs = cvOpenFileStorage(mFilename.c_str(), 0, CV_STORAGE_WRITE);
	
	// Write output directory and object (or sequence) name.
	cvStartWriteStruct(fs, "output", CV_NODE_MAP);
	cvWriteString(fs, "output_directory",          sl_params->outdir, 1);
	cvWriteString(fs, "object_name",               sl_params->object, 1);
	cvWriteInt(fs,    "save_intermediate_results", sl_params->save);
	cvEndWriteStruct(fs);
	
	// Write camera parameters.
	cvStartWriteStruct(fs, "camera", CV_NODE_MAP);
	cvWriteInt(fs, "width",                           sl_params->cam_w);
	cvWriteInt(fs, "height",                          sl_params->cam_h);
	cvWriteInt(fs, "Logitech_Quickcam_9000_raw_mode", sl_params->Logitech_9000);
	cvEndWriteStruct(fs);

	// Write projector parameters.
	cvStartWriteStruct(fs, "projector", CV_NODE_MAP);
	cvWriteInt(fs, "width",            sl_params->proj_w);
	cvWriteInt(fs, "height",           sl_params->proj_h);
	cvWriteInt(fs, "invert_projector", sl_params->proj_invert);
	cvEndWriteStruct(fs);

	// Write camera and projector gain parameters.
	cvStartWriteStruct(fs, "gain", CV_NODE_MAP);
	cvWriteInt(fs, "camera_gain",    sl_params->cam_gain);
	cvWriteInt(fs, "projector_gain", sl_params->proj_gain);
	cvEndWriteStruct(fs);

	// Write distortion model parameters.
	cvStartWriteStruct(fs, "distortion_model", CV_NODE_MAP);
	cvWriteInt(fs, "enable_tangential_camera",          sl_params->cam_dist_model[0]);
	cvWriteInt(fs, "enable_6th_order_radial_camera",    sl_params->cam_dist_model[1]);
	cvWriteInt(fs, "enable_tangential_projector",       sl_params->proj_dist_model[0]);
	cvWriteInt(fs, "enable_6th_order_radial_projector", sl_params->proj_dist_model[1]);
	cvEndWriteStruct(fs);

	// Write camera calibration chessboard parameters.
	cvStartWriteStruct(fs, "camera_chessboard", CV_NODE_MAP);
	cvWriteInt(fs,  "interior_horizontal_corners",  sl_params->cam_board_w);
	cvWriteInt(fs,  "interior_vertical_corners",    sl_params->cam_board_h);
	cvWriteReal(fs, "square_width_mm",              sl_params->cam_board_w_mm);
	cvWriteReal(fs, "square_height_mm",             sl_params->cam_board_h_mm);
	cvEndWriteStruct(fs);

	// Write projector calibration chessboard parameters.
	cvStartWriteStruct(fs, "projector_chessboard", CV_NODE_MAP);
	cvWriteInt(fs, "interior_horizontal_corners",  sl_params->proj_board_w);
	cvWriteInt(fs, "interior_vertical_corners",    sl_params->proj_board_h);
	cvWriteInt(fs, "square_width_pixels",          sl_params->proj_board_w_pixels);
	cvWriteInt(fs, "square_height_pixels",         sl_params->proj_board_h_pixels);
	cvEndWriteStruct(fs);

	// Write scanning and reconstruction parameters.
	cvStartWriteStruct(fs, "scanning_and_reconstruction", CV_NODE_MAP);
	cvWriteInt(fs,  "mode",                           sl_params->mode);
	cvWriteInt(fs,  "reconstruct_columns",            sl_params->scan_cols);
	cvWriteInt(fs,  "reconstruct_rows",               sl_params->scan_rows);
	cvWriteInt(fs,  "frame_delay_ms",                 sl_params->delay);
	cvWriteInt(fs,  "minimum_contrast_threshold",     sl_params->thresh);
	cvWriteReal(fs, "minimum_distance_mm",            sl_params->dist_range[0]);
	cvWriteReal(fs, "maximum_distance_mm",            sl_params->dist_range[1]);
	cvWriteReal(fs, "maximum_distance_variation_mm",  sl_params->dist_reject);
	cvWriteReal(fs, "minimum_background_distance_mm", sl_params->background_depth_thresh);
    cvWriteInt(fs,  "generate_normals",               sl_params->generate_normals);
	cvEndWriteStruct(fs);

	// Write visualization options.
	cvStartWriteStruct(fs, "visualization", CV_NODE_MAP);
	cvWriteInt(fs, "display_intermediate_results", sl_params->display);
	cvWriteInt(fs, "display_window_width_pixels",  sl_params->window_w);
	cvWriteInt(fs, "window_offset_x",  sl_params->window_offset_x);
    cvWriteInt(fs, "window_offset_y",  sl_params->window_offset_y);
	cvEndWriteStruct(fs);

	// Close file storage for XML-formatted configuration file.
	cvReleaseFileStorage(&fs);
}



