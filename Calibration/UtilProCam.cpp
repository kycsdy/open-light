////////////////////////////////////////////////////////////////////////////////////////////////////
// file:	Calibration\UtilProCam.cpp
//
// summary:	Implements the util pro camera class
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Common.h"
#include "Calibration.h"
#include "UtilProCam.h"
#include "Camera.h"

#include <stdlib.h>
#include <time.h>

// Calculate the base 2 logarithm.
double log2(double x)
{
    return log(x)/log(2.0);
}

// Fit a hyperplane to a set of ND points.
// Note: Input points must be in the form of an NxM matrix, where M is the dimensionality.
//       This function finds the best-fit plane P, in the least-squares
//       sense, between the points (X,Y,Z). The resulting plane P is described
//       by the coefficient vector W, where W(1)*X + W(2)*Y +W(3)*Z = W(3), for
//       (X,Y,Z) on the plane P.
void FitPlane(const CvMat* points, float* plane){
	// Estimate geometric centroid.
	int nrows = points->rows;
	int ncols = points->cols;
	int type  = points->type;
	CvMat* centroid = cvCreateMat(1, ncols, type);
	cvSet(centroid, cvScalar(0));
	for(int c=0; c<ncols; c++){
		for(int r=0; r<nrows; r++)
			centroid->data.fl[c] += points->data.fl[ncols*r+c];
		centroid->data.fl[c] /= nrows;
	}
	
	// Subtract geometric centroid from each point.
	CvMat* points2 = cvCreateMat(nrows, ncols, type);
	for(int r=0; r<nrows; r++)
		for(int c=0; c<ncols; c++)
			points2->data.fl[ncols*r+c] = points->data.fl[ncols*r+c] - centroid->data.fl[c];
	
	// Evaluate SVD of covariance matrix.
	CvMat* A = cvCreateMat(ncols, ncols, type);
	CvMat* W = cvCreateMat(ncols, ncols, type);
	CvMat* V = cvCreateMat(ncols, ncols, type);
	cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T); 
	cvSVD(A, W, NULL, V, CV_SVD_V_T);

	// Assign plane coefficients by singular vector corresponding to smallest singular value.
	plane[ncols] = 0;
	for(int c=0; c<ncols; c++){
		plane[c] = V->data.fl[ncols*(ncols-1)+c];
		plane[ncols] += plane[c]*centroid->data.fl[c];
	}

	// Release allocated resources.
	cvReleaseMat(&centroid);
	cvReleaseMat(&points2);
	cvReleaseMat(&A);
	cvReleaseMat(&W);
	cvReleaseMat(&V);
}

// Find intersection between a 3D plane and a 3D line.
// Note: Finds the point of intersection of a line in parametric form 
//       (i.e., containing a point Q and spanned by the vector V, with 
//       a plane W defined in implicit form. Note, this function does 
//       not handle certain "degenerate" cases, since they do not occur
//       in practice with the structured lighting configuration.
void intersectLineWithPlane3D(const float* q, 
							  const float* v, 
							  const float* w,
							  float* p, 
							  float& depth){

	// Evaluate inner products.
	float n_dot_q = 0, n_dot_v = 0;
	for(int i=0; i<3; i++){
		n_dot_q += w[i]*q[i];
		n_dot_v += w[i]*v[i];
	}

	// Evaluate point of intersection P.
	depth = (w[3]-n_dot_q)/n_dot_v;
	for(int i=0; i<3; i++)
		p[i] = q[i] + depth*v[i];
}

// Find closest point to two 3D lines.
// Note: Finds the closest 3D point between two 3D lines defined in parametric
///      form (i.e., containing a point Q and spanned by the vector V). Note, 
//       this function does not handle certain "degenerate" cases, since they 
//       do not occur in practice with the structured lighting configuration.
void intersectLineWithLine3D(const float* q1, 
							 const float* v1, 
							 const float* q2,
							 const float* v2,
							 float* p){

	// Define intermediate quantities.
	float q12[3], v1_dot_v1 = 0, v2_dot_v2 = 0, v1_dot_v2 = 0, q12_dot_v1 = 0, q12_dot_v2 = 0;
	for(int i=0; i<3; i++){
		q12[i]      =  q1[i]-q2[i];
		v1_dot_v1  +=  v1[i]*v1[i];
		v2_dot_v2  +=  v2[i]*v2[i];
		v1_dot_v2  +=  v1[i]*v2[i];
		q12_dot_v1 += q12[i]*v1[i];
		q12_dot_v2 += q12[i]*v2[i];
	}

	// Calculate scale factors.
	float s, t, denom;
	denom = v1_dot_v1*v2_dot_v2 - v1_dot_v2*v1_dot_v2;
	s =  (v1_dot_v2/denom)*q12_dot_v2 - (v2_dot_v2/denom)*q12_dot_v1;
	t = -(v1_dot_v2/denom)*q12_dot_v1 + (v1_dot_v1/denom)*q12_dot_v2;

	// Evaluate closest point.
	for(int i=0; i<3; i++)
		p[i] = ( (q1[i]+s*v1[i]) + (q2[i]+t*v2[i]) )/2;
}

// Capture live image stream (e.g., for adjusting object placement).
int camPreview(Camera* camera, struct slParams* sl_params, struct slCalib* sl_calib){

	// Create a window to display captured frames.
	IplImage* cam_frame  = camera->QueryFrame();
	IplImage* proj_frame = cvCreateImage(cvSize(sl_params->proj_w, sl_params->proj_h), IPL_DEPTH_8U, 1);
	cvNamedWindow("camWindow", CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar("Cam. Gain",  "camWindow", &sl_params->cam_gain,  100, NULL);
	cvCreateTrackbar("Proj. Gain", "camWindow", &sl_params->proj_gain, 100, NULL);
	HWND camWindow = (HWND)cvGetWindowHandle("camWindow");
	BringWindowToTop(camWindow);
	cvWaitKey(1);

	// Capture live image stream.
	int cvKey = -1, cvKey_temp = -1;
	while(1){

		// Project white image.
		cvSet(proj_frame, cvScalar(255));
		cvScale(proj_frame, proj_frame, 2.*(sl_params->proj_gain/100.), 0);
		cvShowImage("projWindow", proj_frame);
		cvKey_temp = cvWaitKey(1);
		if(cvKey_temp != -1) 
			cvKey = cvKey_temp;

		// Capture next frame and update display window.
		cam_frame = camera->QueryFrame();
		cvScale(cam_frame, cam_frame, 2.*(sl_params->cam_gain/100.), 0);
		ShowImageResampled("camWindow", cam_frame, sl_params->window_w, sl_params->window_h);
		cvKey_temp = cvWaitKey(10);
		if(cvKey_temp != -1) 
			cvKey = cvKey_temp;
		
		// Exit on user interaction.
		if(cvKey != -1)
			break;
	}

	// Project black image.
	cvZero(proj_frame);
	cvShowImage("projWindow", proj_frame);
	cvKey_temp = cvWaitKey(1);

	// Return without errors.
	cvDestroyWindow("camWindow");
	cvReleaseImage(&proj_frame);
	return 0;
}

// Shade a grayscale image using the "winter" colormap (similar to Matlab's). 
void colorizeWinter(IplImage* src, IplImage*& dst, IplImage* mask){

	//// Create an increasing linear-ramp in the green channel.
	//cvMerge(NULL, src, NULL, NULL, dst);

	//// Create a decreasing linear-ramp in the blue channel.
	//IplImage* blue = cvCloneImage(src);
	//cvSubRS(src, cvScalar(255.0), blue, mask);
	//cvMerge(blue, NULL, NULL, NULL, dst);
	//
	//// Release allocated resources.
	//cvReleaseImage(&blue);

	// Create an increasing linear-ramp in the green channel.
	cvMerge(src, src, src, NULL, dst);

	//// Create a decreasing linear-ramp in the blue channel.
	//IplImage* blue = cvCloneImage(src);
	//cvSubRS(src, cvScalar(255.0), blue, mask);
	//cvMerge(blue, NULL, NULL, NULL, dst);
	
	// Release allocated resources.
	//cvReleaseImage(&blue);
}

// Show an image, resampled to desired size.
void ShowImageResampled(char* name, 
						  IplImage* image, 
						  int width, 
						  int height){

	// Allocate resampled image.
	IplImage* resampled_image = 
		cvCreateImage(cvSize(width, height), image->depth, image->nChannels);

	// Resize image.
	cvResize(image, resampled_image, CV_INTER_LINEAR);

	// Display resampled image.
	cvShowImage(name, resampled_image);

	// Release allocated resources.
	cvReleaseImage(&resampled_image);
}

// Save a VRML-formatted point cloud.
int savePointsVRML(char* filename, 
				   CvMat* points,
				   CvMat* normals,
				   CvMat* colors,
				   CvMat* mask){

	// Open output file and create header.
	FILE* pFile = fopen(filename, "w");
	if(pFile == NULL){
		fprintf(stderr,"ERROR: Cannot open VRML file!\n");
		return -1;
	}
	fprintf(pFile, "#VRML V2.0 utf8\n");
	fprintf(pFile, "Shape {\n");
	fprintf(pFile, " geometry IndexedFaceSet {\n");

	// Output points (i.e., indexed face set vertices).
	// Note: Flip y-component for compatibility with Java-based viewer.
	if(points != NULL){
		fprintf(pFile, "  coord Coordinate {\n");
		fprintf(pFile, "   point [\n");
		for(int c=0; c<points->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
				for(int r=0; r<points->rows; r++){
					if(r != 1)
						fprintf(pFile, "    %f ",  points->data.fl[c + points->cols*r]);
					else
						fprintf(pFile, "    %f ", -points->data.fl[c + points->cols*r]);
				}
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "   ]\n");
		fprintf(pFile, "  }\n");
	}

	// Output normals (if provided).
	// Note: Flips normals, for compatibility with Java-based viewer.
	if(normals != NULL){
		fprintf(pFile, "  normalPerVertex TRUE\n");
		fprintf(pFile, "  normal Normal {\n");
		fprintf(pFile, "   vector [\n");
		for(int c=0; c<normals->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
				for(int r=0; r<normals->rows; r++)
					fprintf(pFile, "    %f ", -normals->data.fl[c + normals->cols*r]);
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "   ]\n");
		fprintf(pFile, "  }\n");
	}

	// Output colors (if provided).
	// Note: Assumes input is an 8-bit RGB color array.
	if(colors != NULL){
		fprintf(pFile, "  colorPerVertex TRUE\n");
		fprintf(pFile, "  color Color {\n");
		fprintf(pFile, "   color [\n");
		for(int c=0; c<colors->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
				for(int r=0; r<colors->rows; r++)
					fprintf(pFile, "    %f ", colors->data.fl[c + colors->cols*r]);
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "   ]\n");
		fprintf(pFile, "  }\n");
	}

	// Create footer and close file.
	fprintf(pFile, " }\n");
	fprintf(pFile, "}\n");
	if(fclose(pFile) != 0){
		printf("ERROR: Cannot close VRML file!\n");
		return -1;
	}

	// Return without errors.
	return 0;
}

// Save a text file of 3D world points and camera and projector image points
int savePointsTxt(char* filename, 
					CvMat* points,
					IplImage*& gray_decoded_cols, 
					IplImage*& gray_decoded_rows, 
					CvMat* mask, struct slParams* sl_params)
{
	FILE* pFile = fopen(filename, "w");
	
	if(pFile == NULL)
	{
		fprintf(stderr, "ERROR: Cannot open text file!\n");
		return -1;
	}

	float*	gray_decoded_cols_data = (float*)gray_decoded_cols->imageData;
	int     gray_decoded_cols_step = gray_decoded_cols->widthStep/sizeof(float);
	float*	gray_decoded_rows_data = (float*)gray_decoded_rows->imageData;
	int     gray_decoded_rows_step = gray_decoded_rows->widthStep/sizeof(float);
	int		cam_nelems				= sl_params->cam_w*sl_params->cam_h;
	int		num_frames				= 2;
	float	point[3];
	float   percentSubSample = 0.01;

	// Output point format: X Y Z  nframes  frame0 x0 y0  frame1 x1 y1
	// This format is for a 1 camera, 1 projector set up

	// to do:
	// change to use rand with ms
	srand(time(NULL));
	
	for(int r=0; r<sl_params->cam_h; r++)
	{
		for(int c=0; c<sl_params->cam_w; c++)
		{	
			if(mask->data.fl[sl_params->cam_w*r+c] == 1)
			{
				// randomly subsample
				float randVal = rand() % 1000 / 1000.0;
				if(randVal < percentSubSample)
				{
                    fprintf(pFile, "\n");

					float corresponding_column = gray_decoded_cols_data[r*gray_decoded_cols_step+c];
					float corresponding_row    = gray_decoded_rows_data[r*gray_decoded_rows_step+c];

					int rc_cam  = (sl_params->cam_w)*r+c;
					
					for(int i = 0; i < 3; i++)
						point[i] = points->data.fl[rc_cam+cam_nelems*i];

					for(int i = 0; i < 3; i++)
					{
						fprintf(pFile, "%f ", point[i]);
					}

					float fRow = r;
					float fCol = c;

					fprintf(pFile, "%d ", num_frames);
					
					fprintf(pFile, "0 ");
					fprintf(pFile, "%f %f ", fCol, fRow);
					fprintf(pFile, "1 ");
					fprintf(pFile, "%f %f", corresponding_column, corresponding_row);
				}
			}
		}
	}

	// Create footer and close file.
	if(fclose(pFile) != 0){
		printf("ERROR: Cannot close Txt file!\n");
		return -1;
	}

    return 0;
}

// Save a VRML-formatted point cloud.
int savePointsOBJ(char* filename, 
				   CvMat* points,
                   CvMat* faces,
				   CvMat* normals,
                   CvMat* uvCoords,
				   CvMat* colors,
				   CvMat* mask){

	// Open output file and create header.
	FILE* pFile = fopen(filename, "w");
	if(pFile == NULL){
		fprintf(stderr,"ERROR: Cannot open VRML file!\n");
		return -1;
	}
	fprintf(pFile, "#OBJ File\n");
	fprintf(pFile, "\n");

	fprintf(pFile, "#Begin Vertices\n");

	// Output points (i.e., indexed face set vertices).
	// Note: Flip y-component for compatibility with Java-based viewer.
	if(points != NULL){
		for(int c=0; c<points->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
		        fprintf(pFile, "v ");
				for(int r=0; r<points->rows; r++){
					if(r != 1)
                        fprintf(pFile, "%f ",  points->data.fl[c + points->step/sizeof(float)*r]);
					else
                        fprintf(pFile, "%f ", -points->data.fl[c + points->step/sizeof(float)*r]);
				}
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "\n");
	}

	// Output faces
	if(faces != NULL){
		for(int c=0; c<faces->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
                if( faces->data.i[c] > 0 ){
		            fprintf(pFile, "f ");
				    for(int r=0; r<faces->rows; r++){
                        fprintf(pFile, "%i ",  faces->data.i[c + faces->step/sizeof(int)*r]);
				    }
				    fprintf(pFile, "\n");
                }
			}
		}
		fprintf(pFile, "\n");
	}

	// Output normals (if provided).
	// Note: Flips normals, for compatibility with Java-based viewer.
	if(normals != NULL){
		for(int c=0; c<normals->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
                fprintf(pFile, "vn ");
				for(int r=0; r<normals->rows; r++)
                    fprintf(pFile, "%f ", -normals->data.fl[c + normals->step/sizeof(float)*r]);
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "   ]\n");
	}

	// Output uv coords
	if(uvCoords != NULL){
		for(int c=0; c<uvCoords->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
		        fprintf(pFile, "vt ");
				for(int r=0; r<uvCoords->rows; r++){
                    fprintf(pFile, "%f ", uvCoords->data.fl[c + uvCoords->step/sizeof(float)*r]);
				}
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "\n");
	}

	// Create footer and close file.
	fprintf(pFile, "\n");
	if(fclose(pFile) != 0){
		printf("ERROR: Cannot close OBJ file!\n");
		return -1;
	}

	// Return without errors.
	return 0;
}

// In-place conversion of a 10-bit raw image to an 8-bit BGR image.
// Note: Only works with Logitech QuickCam 9000 in 10-bit raw-mode (with a Bayer BGGR mosaic).
//       See: http://www.quickcamteam.net/documentation/how-to/how-to-enable-raw-streaming-on-logitech-webcams
void CvtLogitech9000Raw(IplImage* image, bool return_raw){
	IplImage* raw_image = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
	for(int r=0; r<image->height; r++){
		uchar* image_data     = (uchar*)(image->imageData     + r*image->widthStep);
		uchar* raw_image_data = (uchar*)(raw_image->imageData + r*raw_image->widthStep);
		for(int c=0; c<image->width; c++)
			raw_image_data[c] = uchar((255./1023.)*(image_data[3*c] + 256*image_data[3*c+1]));
	}
	if(return_raw)
		cvMerge(raw_image, raw_image, raw_image, NULL, image);
	else
		cvCvtColor(raw_image, image, CV_BayerBG2BGR);
	cvReleaseImage(&raw_image);
}

IplImage* Gray2BGR(IplImage* frame)
{
    IplImage* frameRGB;

    // if color image
    if(frame->nChannels == 1)
    {
        frameRGB = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, 3);
        cvCvtColor(frame, frameRGB, CV_GRAY2BGR);
    }
    else
    {
        frameRGB = cvCloneImage(frame);
    }

    return frameRGB;
}

void PrintMatrix(std::string name, cv::Mat &mat)
{
    printf("%s:\n", name.c_str());
    for(int row = 0; row < mat.rows; row++)
    {
        for(int col = 0; col < mat.cols; col++)
        {
            printf("%f ", mat.at<float>(row, col));
        }
        printf("\n");
    }
}