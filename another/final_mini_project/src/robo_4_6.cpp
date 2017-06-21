#include <math.h>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <tesseract/baseapi.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <math.h>
#include <algorithm>
#define pi 3.14159265359 


using namespace std;
using namespace cv;
// Class definition
class seek_digit {
public:
	seek_digit();
	~seek_digit()
	{
	  cvDestroyWindow("result");
	}
	void mainLoop();
	//using tesseract to recognize digit
	void recgonize_digit(cv::Mat,int);
	//convert RGB to CMYK
	void rgb_to_cmyk(cv::Mat& , std::vector<cv::Mat>&);
	//save image to Folder
	void save_image(cv::Mat ,int);

	//calculate angle between two lines
	double angle( Point pt1, Point pt2, Point pt0 );
	//draw the lines
	cv::Mat debugSquares( std::vector<std::vector<cv::Point> > , cv::Mat,int);
	//find paper using findContour
	void find_squares(cv::Mat&, vector<vector<Point> >& );
	//move to a direction
	void calculateCommand1();
	void calculateCommand(double target_x=24.15,double target_y=-41.26);
	void camera_task(IplImage *frame, IplImage *frame_copy,CvCapture* capture ,int index=0);

	std::string imagePlace;

	double pose_x;
	double pose_y;
	double pose_yaw;
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
 	void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void emergencyStop();


protected:
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

    	ros::Subscriber m_robotposeSubscriber;
	//ros::Subscriber m_robotorientationSubscriber;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;
};

seek_digit::seek_digit()
{
	// node in root namespace 
	ros::NodeHandle m_nodeHandle("/");
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
	m_nodeHandle.getParam("imagePlace", imagePlace);

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &seek_digit::laserCallback, this);
   	m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &seek_digit::poseCallback, this);
}

// callback for getting laser values 
void seek_digit::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback


void seek_digit::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{	
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
    pose_yaw = tf::getYaw(msg->pose.pose.orientation);

    //ROS_INFO("POS X= %f, Y= %f, yaw =  %f",pose_x, pose_y,pose_yaw);
}

// here we go
// this is the place where we will generate the commands for the robot
void seek_digit::calculateCommand(double target_x,double target_y) {


	//double target_x = 33.46;
	//double target_y = -45.85;
	double difference_target_current_x = target_x-pose_x;
	double difference_target_current_y = target_y-pose_y;
	double radian;
	double angular=0.0;
	double linear=0.3;
	double distance;	

	if( (&m_laserscan)->ranges.size() > 0)
          {
     ROS_INFO("POS X= %f, Y= %f, yaw =  %f",pose_x, pose_y,pose_yaw);
     ROS_INFO("difference_target_current_x= %f, difference_target_current_y= %f",difference_target_current_x, difference_target_current_y);

	//the range of tangent is between -pi/2 and pi/2
	radian=atan(difference_target_current_y/difference_target_current_x);
    ROS_INFO("radian= %f",radian);

	if(difference_target_current_x<0 && difference_target_current_y>0){
		radian=radian+pi;
	}
	else if(difference_target_current_x<0 && difference_target_current_y<0){
		radian=radian-pi;
	}
        double turn_to_target = radian - pose_yaw;
        double angle_target = turn_to_target*180 /pi;
        double angle_radian = radian*180.0 /pi;
        double angle_pose_yaw = pose_yaw*180.0 /pi;
        ROS_INFO(" angle: (angle_target=%f, angle_radian==%f  angle_pose_yaw=%f)", angle_target, angle_radian,angle_pose_yaw );

        // turn to the target direction
        if ((turn_to_target > pi) || (turn_to_target < 0 &&  turn_to_target >-pi )){
            angular = 0.3;
        }
        if ((turn_to_target >0 &&  turn_to_target< pi) || (turn_to_target < -pi)){
            angular = -0.3;
        
        }
	//******if there is an obstacle
	  int c_r = 0 ;
      int c_l = 0 ;
	 // first part
                for( int i=45;i<=270;i++)
                   if (m_laserscan.ranges[i] < 0.5)
                      c_r++; 

                // second part 
                for( int i=270;i<=490;i++)
                   if (m_laserscan.ranges[i] < 0.5)
                      c_l++ ;


                // third part 
                if (c_l > 5)
                   {
                        linear = 0.3, angular  = -0.3 ;
                   } // end of if

                // fourth part 
                if (c_r > 5)
                   {
                        linear = 0.3;
                        angular = +0.3 ;
                   } // end of if

	distance = sqrt(pow(target_y - pose_y,2) + pow(target_x - pose_x,2));
        
        if(distance < 1.0){
            angular = 0;
            linear = 0;
            ROS_INFO("reach the goal!");
        } 

     m_roombaCommand.linear.x  = linear ;
	 m_roombaCommand.angular.z = angular ;

	}


} // end of calculateCommands



/*
save image into a folder called pic
*/
void seek_digit::save_image(cv::Mat im0,int index){
		
	char buffer [23];
	//put string"%d" into buffer
	sprintf(buffer, "%d",index);	
	std::string filename;
	filename = std::string(buffer);
	std::string filetype (".png");
	std::string fileimage;
	fileimage = imagePlace +"/pic/"+ filename + filetype;
	cv::imwrite(fileimage, im0); 
}

// Covert RGB to CMYK using the formula from
// http://rapidtables.com/convert/color/rgb-to-cmyk.htm
void seek_digit::rgb_to_cmyk(cv::Mat& src, std::vector<cv::Mat>& cmyk)
{
    //Checks a condition
    //CV_8UC3 means we use unsigned char types that are 8 bit long and each pixel has three of these to form the three channels. 
    CV_Assert(src.type() == CV_8UC3);

    cmyk.clear();
    for (int i = 0; i < 4; ++i)
	// 4-byte floating point (float).
	// CMYK colorspace 
        cmyk.push_back(cv::Mat(src.size(), CV_32F));

    for (int i = 0; i < src.rows; ++i)
    {
        for (int j = 0; j < src.cols; ++j)
        {
            cv::Vec3b p = src.at<cv::Vec3b>(i,j);

            float r = p[2] / 255.;
            float g = p[1] / 255.;
            float b = p[0] / 255.;
            float k = (1 - std::max(std::max(r,g),b));

            cmyk[0].at<float>(i,j) = (1 - r - k) / (1 - k); 
            cmyk[1].at<float>(i,j) = (1 - g - k) / (1 - k);
            cmyk[2].at<float>(i,j) = (1 - b - k) / (1 - k);
            cmyk[3].at<float>(i,j) = k;
        }
    }
}
/*
 use tesseract OCR to recognize digit
*/
void seek_digit::recgonize_digit(cv::Mat im0,int index){
    
    std::vector<cv::Mat> cmyk;
    rgb_to_cmyk(im0, cmyk);


    cv::Mat im1;
    //blur(im0, im1, Size(3,3));
   // save_image(im1,7000+index);
    //Matrix multiplication: invert the M channel and multiply it with the K channel,Keep only the largest contour
    im1 = cmyk[3].mul(1 - cmyk[1]) > 0.5;
	//save the image
	//save_image(im1,1000+index);

    //Keep only the largest contour,
    cv::Mat im2;
    im1.convertTo(im2, CV_8U);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(im2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //find the maximum area
    double max_area = 0;
    int max_idx = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        max_idx  = area > max_area ? i : max_idx;
        max_area = area > max_area ? area : max_area;
    }
    
    im2.setTo(cv::Scalar(0));
    cv::drawContours(im2, contours, max_idx, cv::Scalar(255), -1);

	//save the image
	//save_image(im2,2000+index);

    //Use the image above to extract the digits from the original image,
    cv::Mat im3;
    cv::cvtColor(im0, im3, CV_BGR2GRAY);
    im3 = ((255 - im3) & im2) > 100;

   //Remove the remaining noise,
    cv::Mat dst = im3.clone();
    cv::findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	//save the image
	save_image(dst,3000+index);
    //Remove the remaining noise,
    for (int i = 0; i < contours.size(); i++)
    {
        if (cv::contourArea(contours[i]) < 100)
            cv::drawContours(dst, contours, i, cv::Scalar(0), -1);
    }

    tesseract::TessBaseAPI tess;
    tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tess.SetVariable("tessedit_char_whitelist", "0123456789");
    tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
    tess.SetImage((uchar*)dst.data, dst.cols, dst.rows, 1, dst.cols);

   char* out = tess.GetUTF8Text();
   int val = atoi(out);
   ROS_INFO( "the digit number is: %d", val );

}
/*
draw the rectangular on the image
*/
cv::Mat seek_digit::debugSquares( std::vector<std::vector<cv::Point> > squares, cv::Mat image,int index )
{
    for ( int i = 0; i< squares.size(); i++ ) {
        // draw contour
       // cv::drawContours(image, squares, i, cv::Scalar(255,0,0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
	
        // draw bounding rect
         cv::Rect rect = boundingRect(cv::Mat(squares[i]));
         cv::rectangle(image, rect.tl(), rect.br(), cv::Scalar(255,0,0), 2, 8, 0);
      
	 cv::Mat result; // segmentation result (4 possible values)
         cv::Mat bgModel,fgModel; // the models (internally used)
	 // GrabCut segmentation
  	 cv::grabCut(image,result,rect,bgModel,fgModel,1,cv::GC_INIT_WITH_RECT); 
	 // Get the pixels marked as likely foreground
    	 cv::compare(result,cv::GC_PR_FGD,result,cv::CMP_EQ);
	 // Generate output image
         cv::Mat foreground(image.size(),CV_8UC3,cv::Scalar(255,255,255));
	 // bg pixels not copied
         image.copyTo(foreground,result); 
	//save image
	 save_image(foreground,666+index);
	//recognize the digit on the image
	 recgonize_digit(foreground,index);
    }

    return image;
}
// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double seek_digit::angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/*
find the paper
*/
void seek_digit::find_squares(cv::Mat& image, vector<vector<Point> >& squares)
{
    // blur will enhance edge detection
    Mat blurred(image);
    //Blurs an image using the median filter.
    medianBlur(image, blurred, 9);

    Mat gray0(blurred.size(), CV_8U), gray;
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++)
    {
	//Copies specified channels from input arrays to the specified channels of output arrays.
        int ch[] = {c, 0};
        mixChannels(&blurred, 1, &gray0, 1, ch, 1);
        
        // try several threshold levels
        const int threshold_level = 2;
        for (int l = 0; l < threshold_level; l++)
        {
            // Use Canny instead of zero threshold level!
            // Canny helps to catch squares with gradient shading
            if (l == 0)
            {
                Canny(gray0, gray, 10, 20, 3); 

                // Dilate helps to remove potential holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                    gray = gray0 >= (l+1) * 255 / threshold_level;
            }

            // Find contours and store them in a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            // Test contours
            vector<Point> approx;
            for (size_t i = 0; i < contours.size(); i++)
            {
                    // approximate contour with accuracy proportional to the contour perimeter
		    //Calculates a contour perimeter or a curve length.
                    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		    //ROS_INFO("arcLength= %lf",arcLength(Mat(contours[i]), true)*0.02);
                    // Note: absolute value of an area is used because
                    // area may be positive or negative - in accordance with the
                    // contour orientation
                    if (approx.size() == 4 &&
                            fabs(contourArea(Mat(approx))) > 2000 &&
                            fabs(contourArea(Mat(approx))) < 65000&&
                            isContourConvex(Mat(approx)))
                    {
			int index=0;
			
			   ROS_INFO("contourArea= %lf",fabs(contourArea(Mat(approx))));
                            double maxCosine = 0;

                            for (int j = 2; j < 5; j++)
                            {
                                    double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                                    maxCosine = MAX(maxCosine, cosine);
                            }

                            if (maxCosine < 0.3)
                                    squares.push_back(approx);
                    }
            }
        }
    }
}
void seek_digit::calculateCommand1() {

	// set the roomba velocities
	// the linear velocity (front direction is x axis) is measured in m/sec
	// the angular velocity (around z axis, yaw) is measured in rad/sec
	m_roombaCommand.linear.x  = 0.2;
	m_roombaCommand.angular.z = 0.0;

} // end of calculateCommands
void seek_digit::camera_task(IplImage *frame, IplImage *frame_copy ,CvCapture* capture ,int index) {

 		    if( !cvGrabFrame( capture )){
		        return;
		    }
		    
		    //frame = cvRetrieveFrame( capture );
		    frame = cvQueryFrame( capture );
		    if( !frame ) {
		        return;
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
	
		    cv::Mat dispaly_image;
		    vector<vector<Point> > squares;
		    cv::Mat im0 = cv::cvarrToMat(frame_copy); 
		    //find the paper
		    find_squares(im0, squares);
		    if(squares.size()>0){

			dispaly_image=debugSquares(squares, im0,index);	
			cv::imshow( "result", dispaly_image );
		   }

			index++;
//A common mistake for opencv newcomers is to call cv::imshow() in a loop through video frames, without following up each draw with cv::waitKey(30). In this case, nothing appears on screen, because highgui is never given time to process the draw requests from cv::imshow().
		if( cvWaitKey( 10 ) >= 0 )
		       return;
}

// Main loop (what else)
void seek_digit::mainLoop() {
	
	ros::Rate loop_rate(20);

	CvCapture* capture = 0;
	IplImage *frame, *frame_copy = 0;

	// open the connection to the camera
	capture = cvCaptureFromCAM( 0 );
	//capture = cvCaptureFromCAM( 1 );
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
	
	cvNamedWindow( "result", 0 );

	int index=1;
	// Stop if a kill command is sent
	while (m_nodeHandle.ok())
	{
		calculateCommand1();
		// send the command to the roomrider for execution
		m_commandPublisher.publish(m_roombaCommand);

		camera_task(frame, frame_copy,capture,index);
		   

	// SpinOnce , just do the loop once. processes your callbacks 
	ros::spinOnce();
	// sleep stops for ~50ms, to get aligned with desired looprate
	loop_rate.sleep();
b

	}
	
	cvReleaseImage( &frame_copy );
	cvReleaseCapture( &capture );
	
}

int main(int argc, char** argv) {

	// Initialize
	ros::init(argc, argv, "seek_digit");
	ros::NodeHandle n;
	seek_digit f_g;
	f_g.mainLoop();
	ros::spin();
	return 0;
	
}

// end of robo_5_0.cpp

