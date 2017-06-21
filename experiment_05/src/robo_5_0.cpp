//
// Institut für Informatik
// Robotik Praktikum
//
// robo_5_0.cpp
//
// Original Name: color control
//
// Autor: N.Goerke, A.Hochrath, T.Fiolka
//

// Necessary includes for ROS, opencv and all the messages between the nodes
//
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <math.h>

#include <geometry_msgs/Twist.h>


// Class definition
class color_control {
public:
	void mainLoop();
	double R_X, R_Y; 
	int C_C;
	double setSpeed( double, double );
	void do_some_magic( IplImage* , int , int , int , int );

	color_control()
	{
	  // node in root namespace 
	  ros::NodeHandle m_nodeHandle("/");
	  
	  m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
	}
	
	~color_control()
	{
	  cvDestroyWindow("result");
	}



protected:

	ros::NodeHandle m_nodeHandle;

	// Publisher and Membervariables for driving 
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// normalize center of color to propulsion command
double color_control::setSpeed( double center, double width ){
  
  double normedCenter = (12.0*center/width) - 6.0;
  // ROS_INFO("normed center: %f", normedCenter);
  double retval = 0.0;
  
  if ( fabs( normedCenter ) > 1.0 ){
      retval = (trunc( normedCenter )/10.0 );
  } 
  return retval;
  
}

// comments R 4 mollycoddles, guess what i'm doing!
void color_control::do_some_magic( IplImage* img, int red, int green, int blue, int tolerance) {

    int radius = 20 ;

    int width = img->width;
    int height = img->height;
    int channels = img->nChannels;
    int step = img->widthStep;

    unsigned char redP = 0, greenP = 0, blueP = 0;
    double S_x = 0.0 ;
    double S_y = 0.0 ;
    int red_difference, green_difference, blue_difference = 255;

    C_C = 0 ;

    for(int y=0;y<height;y++) {
        for(int x=0;x<width;x++) {
          blueP = img->imageData[y*step+x*channels+0] ;
          greenP = img->imageData[y*step+x*channels+1] ;
          redP = img->imageData[y*step+x*channels+2] ;

	  red_difference = fabs(red - (int)redP);
	  green_difference = fabs(green - (int)greenP);
	  blue_difference = fabs(blue - (int)blueP);

	  if ( (red_difference + green_difference + blue_difference) < tolerance ) {
	     C_C++ ;
	     S_x += x;
	     S_y += y;

	     //ROS_INFO( "Difference to target color: %i", (red_difference + green_difference + blue_difference) );
	    }
	
        }
     }

    S_x = S_x / (C_C) ;
    S_y = S_y / (C_C) ;

    R_X = setSpeed( S_x, img->width );
    R_Y = setSpeed( S_y, img->height );
    
    // draw a circle to into the control image 
    CvPoint center;
    center.x = S_x;
    center.y = S_y;
    cvCircle( img, center, radius, CV_RGB( 255 - red, 255 - green, 255 - blue ) , 3, 8, 0 );
    cvShowImage( "result", img );
}

// Main loop (what else)
void color_control::mainLoop() {
	// 
	ros::Rate loop_rate(20);

	CvCapture* capture = 0;
	IplImage *frame, *frame_copy = 0;

	// open the connection to the camera
	capture = cvCaptureFromCAM( 0 );
	//capture = cvCaptureFromCAM( 1 );
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
	
	cvNamedWindow( "result", 0 );

	// Stop if a kill command is sent
	while (m_nodeHandle.ok())
	{
		    if( !cvGrabFrame( capture )){
		        break;
		    }
		    
		    //frame = cvRetrieveFrame( capture );
		    frame = cvQueryFrame( capture );
		    if( !frame ) {
		        break;
		    }

		    if( !frame_copy )
		        frame_copy = cvCreateImage( cvSize(frame->width,frame->height),
		                                    IPL_DEPTH_8U, frame->nChannels );
		    if( frame->origin == IPL_ORIGIN_TL ){
		        cvCopy( frame, frame_copy, 0 );
		    }
		    else {
		        cvFlip( frame, frame_copy, 0 );
		    }
			
		    cvShowImage( "result", frame_copy );
		    
		    do_some_magic( frame , 225, 50, 75, 100);

		    if( cvWaitKey( 10 ) >= 0 )
		        break;
		    
		 ROS_INFO( "C_C: %d", C_C );

		 // set the direction to drive
		 if (C_C > 50)
		  {
			m_roombaCommand.linear.x = R_Y;
			m_roombaCommand.angular.z = -R_X;
		  } else {
			m_roombaCommand.linear.x = 0.0;
			m_roombaCommand.angular.z = 0.0;
		  }

		ROS_INFO(" forward: %f - turn: %f", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

		// send the driving command to the robot
		m_commandPublisher.publish(m_roombaCommand);

	// SpinOnce , just do the loop once
	ros::spinOnce();
	// sleep stops for ~50ms, to get aligned with desired looprate
	loop_rate.sleep();
	
	}
	
	cvReleaseImage( &frame_copy );
	cvReleaseCapture( &capture );
	
}

int main(int argc, char** argv) {

	// Initialize
	ros::init(argc, argv, "color_control");
	ros::NodeHandle n;
	color_control f_g;
	f_g.mainLoop();
	ros::spin();
	return 0;
	
}

// end of robo_5_0.cpp

