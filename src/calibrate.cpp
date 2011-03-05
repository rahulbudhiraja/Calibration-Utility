#include<cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include<vector>
#include<stdio.h>
#include<cstdlib>
#include<sstream> // for stringstream
#include <algorithm>
#include <functional>
#include <cv.h>
#include <highgui.h>
#include <ctype.h>.

//#define POINTGRAY

#ifdef POINTGRAY

#include <flycapture/FlyCapture2.h>
#define  FLYCAPTURE2_API __declspec( dllimport )
using namespace FlyCapture2;

#endif

int n_boards = 0;
const int board_dt = 20;
int board_w;
int board_h;

IplImage *image=0,*im_colored=0;

using namespace std;

#ifdef POINTGRAY
	
	    IplImage *frame = NULL;
    
	    FlyCapture2::Error error;
	    FlyCapture2::PGRGuid guid;
	    FlyCapture2::BusManager busMgr;
	    FlyCapture2::Camera cam;
	    //Get one raw image to be able to calculate the OpenCV window size
	    FlyCapture2::Image rawImage;
	    cv::Mat Image_Mat,Gray_Mat;	   	    
#endif



#ifdef POINTGRAY

IplImage *PointGrayQueryFrame()
{
        cam.RetrieveBuffer(&rawImage);


        // Get the raw image dimensions
        FlyCapture2::PixelFormat pixFormat;
        unsigned int rows, cols, stride;
        rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );
	//std::cout<<rows<<" "<<cols<<"\n";
        // Create a converted image
        FlyCapture2::Image convertedImage;

	// Convert the raw image
        error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_RGB8,&convertedImage );
     
     
     
        if (error != FlyCapture2::PGRERROR_OK)
        {
            error.PrintErrorTrace();
            exit(0);
        }

	
        //Copy the image into the IplImage of OpenCV
        
        memcpy(image->imageData, convertedImage.GetData(), convertedImage.GetDataSize());
 	cv::cvtColor(image, Image_Mat, CV_BGR2RGB);
 	
 	//IplImage *im_rgb  = cvLoadImage("image.jpg");
	cvCvtColor(image,im_colored,CV_BGR2RGB);
 	
	//cv::cvtColor(image,image_recolored,CV_BGR2RGB);
	
	//image_recolored=cvCloneImage(image);
	cvShowImage("Input",image);


	cv::imshow( "Original", Image_Mat);
	
	
	
	return im_colored;
        
	//return image;  
}



#endif

int main(int argc, char *argv[])
{

	board_w = 5; // Board width in squares
	board_h = 8; // Board height 
	n_boards =20; // Number of boards
	int board_n = board_w * board_h;
	CvSize board_sz = cvSize( board_w, board_h );
	
	
	#ifdef POINTGRAY
	//Getting the GUID of the cam
	    
	    error = busMgr.GetCameraFromIndex(0, &guid);
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
		error.PrintErrorTrace();
		return -1;
	    }
	    
	

	    // Connect to a camera
	    error = cam.Connect(&guid);
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
		error.PrintErrorTrace();
		return -1;
	    }
	    
	    //Starting the capture
	    error = cam.StartCapture();
	    if (error != FlyCapture2::PGRERROR_OK)
	    {
		error.PrintErrorTrace();
		return -1;
	    }
	    
	    
	    cam.RetrieveBuffer(&rawImage);
	    
	     image = cvCreateImage(cvSize(rawImage.GetCols(), rawImage.GetRows()), 8, 3);
	     im_colored = cvCreateImage(cvGetSize(image),8,3);	
        #endif
	
	
	#ifdef POINTGRAY
	
	#else
	CvCapture* capture = cvCreateCameraCapture( 0 );
	assert( capture );
	#endif
	
	cvNamedWindow("Calibration" );
	cvNamedWindow("Live Camera Feed");
	cvMoveWindow("Calibration", 500, 500);

	// Allocate Storage
	CvMat* image_points		= cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
	CvMat* object_points		= cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
	CvMat* point_counts		= cvCreateMat( n_boards, 1, CV_32SC1 );
	CvMat* intrinsic_matrix		= cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* distortion_coeffs	= cvCreateMat( 5, 1, CV_32FC1 );

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	int corner_count;
	int successes = 0;
	int step = 0;

	//cout<<"ok till here";
	#ifdef	POINTGRAY	
	image = PointGrayQueryFrame();
	#else
	image = cvQueryFrame( capture );
	#endif 
	
	IplImage *gray_image = cvCreateImage( cvGetSize( image ), 8, 1 );

	// Capture Corner views loop until we've got n_boards
	// succesful captures (all corners on the board are found)
      
              
	while( successes < n_boards ){
		// Skp every f frames to allow user to move chessboard
		
	       	int c=cvWaitKey(30);
	       	
	       	if(c==1048603||c==27)
	       	break;
	       	
	        if(c==1048608||c==32){
			// Find chessboard corners:
			int found = cvFindChessboardCorners( image, board_sz, corners,
				&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

			// Get subpixel accuracy on those corners
			cvCvtColor( image, gray_image, CV_BGR2GRAY );
			cvFindCornerSubPix( gray_image, corners, corner_count, cvSize( 11, 11 ), 
				cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			// Draw it
			cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );
			cvShowImage( "Calibration", image );

			// If we got a good board, add it to our data
			if( corner_count == board_n ){
				step = successes*board_n;
				for( int i=step, j=0; j < board_n; ++i, ++j ){
					CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w;
					CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w;
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
				}
				CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
				successes++;
				cout<<"\n The number of Boards Captured are"<<successes<<"\n";
			}
		} 

	#ifdef	POINTGRAY	
	image = PointGrayQueryFrame();
	#else
	image = cvQueryFrame( capture );
	#endif 
	
	cvShowImage( "Live Camera Feed", image );
				
	} // End collection while loop

	// Allocate matrices according to how many chessboards found
	CvMat* object_points2 = cvCreateMat( successes*board_n, 3, CV_32FC1 );
	CvMat* image_points2 = cvCreateMat( successes*board_n, 2, CV_32FC1 );
	CvMat* point_counts2 = cvCreateMat( successes, 1, CV_32SC1 );
	
	// Transfer the points into the correct size matrices
	for( int i = 0; i < successes*board_n; ++i ){
		CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0 );
		CV_MAT_ELEM( *image_points2, float, i, 1) = CV_MAT_ELEM( *image_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0 );
		CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2 );
	}

	for( int i=0; i < successes; ++i ){
		CV_MAT_ELEM( *point_counts2, int, i, 0 ) = CV_MAT_ELEM( *point_counts, int, i, 0 );
	}
	cvReleaseMat( &object_points );
	cvReleaseMat( &image_points );
	cvReleaseMat( &point_counts );

	// At this point we have all the chessboard corners we need
	// Initiliazie the intrinsic matrix such that the two focal lengths
	// have a ratio of 1.0

	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;

	// Calibrate the camera
	cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ), 
		intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO ); 

	// Save the intrinsics and distortions
	cvSave( "Intrinsics.xml", intrinsic_matrix );
	cvSave( "Distortion.xml", distortion_coeffs );

	// Example of loading these matrices back in
	CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
	CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );

	// Build the undistort map that we will use for all subsequent frames
	IplImage* mapx = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	IplImage* mapy = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic, distortion, mapx, mapy );

	// Run the camera to the screen, now showing the raw and undistorted image
	cvNamedWindow( "Undistort" );

	while( image ){
		IplImage *t = cvCloneImage( image );
		cvShowImage( "Calibration", image ); // Show raw image
		cvRemap( t, image, mapx, mapy ); // undistort image
		cvReleaseImage( &t );
		cvShowImage( "Undistort", image ); // Show corrected image

		// Handle pause/unpause and esc
		int c = cvWaitKey( 15 );
		if( c == 'p' ){
			c = 0;
			while( c != 'p' && c != 27 ){
				c = cvWaitKey( 250 );
			}
		}
		if( c == 1048603 ||c==27)
		exit(0);//break;
		
		#ifdef	POINTGRAY	
		image = PointGrayQueryFrame();
		#else
		image = cvQueryFrame( capture );
		#endif 
	
	
	}

	return 0;
}


