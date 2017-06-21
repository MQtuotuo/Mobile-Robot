#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <math.h>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <tesseract/baseapi.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <Eigen/Dense>
#define pi 3.14159265359
using namespace std;
using namespace cv;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Class definition
class seek_digit {
public:
	seek_digit();
	~seek_digit()
	{
	  cvDestroyWindow("result");
	}
	std::string save_image_folder;
	std::string input_cam;


	void mainLoop();
    void emergencyStop();
	//using tesseract to recognize digit
	void recgonize_digit(cv::Mat,int);
	//convert RGB to CMYK
	void rgb_to_cmyk(cv::Mat& , std::vector<cv::Mat>&);
	//save image to Folder
	void save_image(cv::Mat ,int);

	//calculate angle between two lines
	double angle( Point pt1, Point pt2, Point pt0 );
	//draw the lines
	cv::Mat desplaySquares( std::vector<std::vector<cv::Point> > , cv::Mat,int);
	//find paper using findContour
	void find_squares(cv::Mat&, vector<vector<Point> >& );
	//move to a direction
	int calculateCommand();
	std::vector<std::vector<double> > save_informations;

	//using move_base to reach a goal
	int move_to_Goal(double , double , double );

	//if detect paper ,then save number and position
	void save_number_position(double, double , double,double,std::vector<std::vector<double> >&);
	//take picture
	void camera_task(IplImage *frame, IplImage *frame_copy,CvCapture* capture ,int index=0);

	double distance_image_currentPosi(double , double , double ,double);
	std::string imagePlace;

	//get detected picure's position and Quaternion
	//Eigen::Matrix4d getRelativeTransform(double length,double width,std::vector<std::vector<cv::Point> > ,double fx, double fy, double px, double py);
	
    //check the digital number
	double temp;
	double pose_x;
	double pose_y;
	double pose_yaw;
      //read calibration parameters
        double fx,fy,cx,cy;

	//void calculate_position_orientation_image(std::vector<std::vector<cv::Point> > four_points,vector<double> &position,Eigen::Quaternion<double> &rot_quaternion);

	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
 	void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
	

protected:
	ros::NodeHandle m_nodeHandle;

	// for laserscanner
	sensor_msgs::LaserScan m_laserscan;
	ros::Subscriber m_laserSubscriber;

	//initialize subscriber for ACML 
    ros::Subscriber m_robotposeSubscriber;

	//For USB camera
	image_transport::CameraSubscriber m_cameraSubscriber;
	image_transport::Publisher m_cameraPublisher;

	// Publisher for CMD 
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;
};

seek_digit::seek_digit()
{
	// node in root namespace 
	ros::NodeHandle m_nodeHandle("/");

	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
   	m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &seek_digit::poseCallback, this);
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &seek_digit::laserCallback, this);
	
	
	m_nodeHandle.getParam("save_image_folder", save_image_folder);
    temp =0;
	// image_transport::ImageTransport m_imageTransport(m_nodeHandle);
	// m_nodeHandle.param<std::string>("input_camera",input_cam,"/usb_cam/image_raw");
	// m_cameraSubscriber = m_imageTransport.subscribeCamera(input_cam, 1, &seek_digit::imageCallback, this);

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

    ROS_INFO("POS X= %f, Y= %f, yaw =  %f",pose_x, pose_y,pose_yaw);
}
/*
void seek_digit::imageCallback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info)
{

      //camera calibration
      // [ fx 0  cx  ]
      // [  0 fy cy  ]
      // [  0  0  1  ]
      if(cam_info->K[0]){
	 fx = cam_info->K[0];
	 fy = cam_info->K[4];
	 cx = cam_info->K[2];
	 cy = cam_info->K[5];
	 
      }
      else{ 
	 fx = 384.6756;  fy = 384.6756;  cx = 640;  cy = 512;
      }
 	ROS_INFO("camera parameters fx= %lf , fy= %lf , cx = %lf , cy = %lf ",fx, fy,cx,cy);
}
*/
void seek_digit::emergencyStop() {

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0)
	{
		for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
		{
			if( m_laserscan.ranges[i] <= 0.40) {
				m_roombaCommand.linear.x = 0.0;
				m_roombaCommand.angular.z = 0.0;
			}// end of if too close
		}// end of for all laser beams
	} // end of if we have laser data

}// end of emergencyStop

//save detected number and sort them into a vector
void  seek_digit::save_number_position(double x, double y, double yaw ,double number,std::vector<std::vector<double> >& save_informations){

    if(save_informations.size()==0){
	    std::vector<std::vector<double> > save_informations_temp(9,std::vector<double>(4,0));
        save_informations=save_informations_temp;
    }

	if(number>0&& number<10 && save_informations[number-1][0]==0){
		save_informations[number-1][0]=number;
		save_informations[number-1][1]=x;		
		save_informations[number-1][2]=y;
		save_informations[number-1][3]=yaw;
    	ROS_INFO("save parameters number= %lf , pose_y= %lf , pose_y = %lf , pose_yaw = %lf ",save_informations[number-1][0], save_informations[number-1][1],save_informations[number-1][2],save_informations[number-1][3]);
	}

}

//calculate the position and orientation of the detected image
/*
void seek_digit::calculate_position_orientation_image(std::vector<std::vector<cv::Point> > four_points,vector<double> &position,Eigen::Quaternion<double> &rot_quaternion){

      Eigen::Matrix4d transform = getRelativeTransform(0.297,0.210, four_points, fx, fy, cx, cy);
      //get orientation
      Eigen::Matrix3d rot = transform.block(0,0,3,3);
      rot_quaternion = Eigen::Quaternion<double>(rot);
      //get image position
      //position.x		
      position.push_back(transform(0,3));
      //position.y
      position.push_back(transform(1,3));  
}

*/



//calculate distance between current position and position of A4 paper
double seek_digit::distance_image_currentPosi(double x, double y, double x_current,double y_current){

	double distance= sqrt(((x-x_current)*(x-x_current))+((y-y_current)*(y-y_current)));
	return distance;
}

//use move_base move to a position
int seek_digit::move_to_Goal(double x, double y, double yaw){

	MoveBaseClient ac("move_base", true);

  	//wait for the action server to come up
  	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
  	}//end of while

	ROS_INFO("Navigate to goal [%lf,%lf]", x, y);

	//creates a goal
	move_base_msgs::MoveBaseGoal goal; 
	goal.target_pose.header.frame_id = "map";

	//convert the yaw/z to quaternion
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
	goal.target_pose.header.stamp = ros::Time::now();
	
	//set goals x,y,z/Yaw 	
	goal.target_pose.pose.orientation = quat;
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	
	//send goal to move_base
	ac.sendGoal(goal);

	//wait for the Robot to reach the goal
	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Reach Goal.");
		return 1;		
	} else { 
		ROS_INFO("Not reach the Goal - something went wrong!");
		return -1;			
	}//end of if

}
// this is the place where we will generate the commands for the robot
int seek_digit::calculateCommand() {

// see if we have laser data	
	if( (&m_laserscan)->ranges.size() > 0)
	  {
		// set the roomba velocities
		// the linear velocity (front direction is x axis) is measured in m/sec
		// the angular velocity (around z axis, yaw) is measured in rad/sec
		//if the velocity.z bigger than 0.4,the roomba may shakes

		double target_x = 24.00;
		double target_y = -41.00;

		//double target_x = 31.15;
		//double target_y = -36.26;
		double vel_linear=0.14;
		double vel_angular=0.00;
		int shortest_laser=0;
		double shortest_laser_value=2.0;
		double keep_distance=0.75;
		// find the shortest distance between roomba and wall
		for(int i=0;i<=495;i++){	

		   if(m_laserscan.ranges[i]<shortest_laser_value){
		       shortest_laser_value=m_laserscan.ranges[i];
		       shortest_laser=i;
		   }	
		}
		//move along left hand side 
		if(450<=shortest_laser && shortest_laser<495){
 		     if (shortest_laser_value <= keep_distance){ vel_angular = -0.3;  }
		     else{ vel_angular = 0.2; }

			//ROS_INFO(" shortest distance <450~495>: forward=%lf", shortest_laser_value);
		}
		//maintain certain distance between roomba and wall
      	        if (shortest_laser==90) {
		     if (shortest_laser_value >= keep_distance){ vel_angular = -0.2;  }
		     else {
				vel_angular = 0.3;
			}
			//ROS_INFO(" shortest distance <90>: distance=%lf", shortest_laser_value);
		}
		//if the wall on the -y direction
		if(90 < shortest_laser && shortest_laser < 270){
			if (shortest_laser_value <= keep_distance)
			{	vel_angular = 0.2; 
				if((shortest_laser )*pi / 360 < 0.2){
				     vel_angular = ((shortest_laser )*pi / 360);
				}
			}
			//ROS_INFO(" shortest distance <90--270>: distance=%lf", shortest_laser_value);
		}
		//maintain certain distance between roomba and wall
		if(shortest_laser==270){
		     if (shortest_laser_value >= keep_distance){ vel_angular = -0.3;  }
			else{
				vel_angular = 0.3;
			}
			//ROS_INFO(" shortest distance <270>: distance=%lf", shortest_laser_value);
		
		}
		//if the wall on the +y direction
		if(270<shortest_laser && shortest_laser<450){
		    if (shortest_laser_value <= keep_distance)
		    {
		 	  vel_angular= -0.4;
			
		    }
			//ROS_INFO(" shortest distance <270-450>: distance=%lf", shortest_laser_value);
		}
	     //reach a goal;
	     double distance= sqrt(pow(target_y - pose_y,2) + pow(target_x - pose_x,2));
   	     if(distance < 2.0){
    	        vel_angular = 0;
    	        vel_linear = 0;
     	       ROS_INFO("reach the goal!");
		return 1;
     	      } 
	       m_roombaCommand.linear.x  = vel_linear;
	       m_roombaCommand.angular.z = vel_angular;
            emergencyStop();
	  } // end of if we have laser values

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
	//save_image(dst,3000+index);
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
   if(val<10 && val>0 && temp == val){
	save_number_position(pose_x,pose_y,pose_yaw,val,save_informations);
	
   }
    temp = val;
}
/*
draw the rectangular on the image
*/
cv::Mat seek_digit::desplaySquares( std::vector<std::vector<cv::Point> > squares, cv::Mat image,int index )
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
	 //save_image(foreground,666+index);
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
                            fabs(contourArea(Mat(approx))) > 5000 &&
                            fabs(contourArea(Mat(approx))) < 21000&&
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
/*
Eigen::Matrix4d seek_digit::getRelativeTransform(double tag_size_length,double tag_size_width,std::vector<std::vector<cv::Point> >  p, double fx, double fy, double px, double py) {
  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double length = tag_size_length/2;
  double width= tag_size_width/2;
  objPts.push_back(cv::Point3f(-width,-length, 0));
  objPts.push_back(cv::Point3f( width,-length, 0));
  objPts.push_back(cv::Point3f( width, length, 0));
  objPts.push_back(cv::Point3f(-width, length, 0));

  Point p1 = cv::Point(p[0][0]);
  Point p2 = cv::Point(p[0][1]);
  Point p3 = cv::Point(p[0][2]);
  Point p4 = cv::Point(p[0][3]);
  imgPts.push_back(cv::Point2f(p1.x,p1.y));
  imgPts.push_back(cv::Point2f(p2.x,p2.y));
  imgPts.push_back(cv::Point2f(p3.x,p3.y));
  imgPts.push_back(cv::Point2f(p4.x,p4.y));

  cv::Mat rvec, tvec;
    //fx and fy are camera focal lengths,cx and cy are are the optical centers expressed in pixels coordinates.
  cv::Matx33f cameraMatrix(
                           fx, 0, px,
                           0, fy, py,
                           0,  0,  1);
  cv::Vec4f distParam(0,0,0,0); // all 0?
  //Finds an object pose from 3D-2D point correspondences.
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33d r;
  //Converts a rotation matrix to a rotation vector or vice versa.
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

  Eigen::Matrix4d T; 
  T.topLeftCorner(3,3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0,0,0,1;  

  return T;
}
*/

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

			dispaly_image=desplaySquares(squares, im0,index);	
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
	int reach_goal=0;
	// Stop if a kill command is sent
	while (m_nodeHandle.ok())
	{  
        if(reach_goal!=1){
			//wall_follower
            emergencyStop();
			reach_goal=calculateCommand();
            emergencyStop();
			// send the command to the roomrider for execution
		    m_commandPublisher.publish(m_roombaCommand);	

			camera_task(frame, frame_copy,capture,index);
		}else if(reach_goal==1 && save_informations.size()!=0){
	
    
			for(int i=0;i<9;i++){
            ROS_INFO("start moving to goal ----number= %lf , pose_x= %lf , pose_y = %lf , pose_yaw = %lf ",save_informations[i][0], save_informations[i]   [1],save_informations[i][2],save_informations[i][3]);
			move_to_Goal(save_informations[i][1] , save_informations[i][2] , save_informations[i][3]);
			
			}	
			
		}

	// SpinOnce , just do the loop once. processes your callbacks 
	ros::spinOnce();
	// sleep stops for ~50ms, to get aligned with desired looprate
	loop_rate.sleep();

	}
	
	cvReleaseImage( &frame_copy );
	cvReleaseCapture( &capture );
	
}

int main(int argc, char** argv) {

	// Initialize
	ros::init(argc, argv, "seek_digit");
	//ros::NodeHandle n;
	seek_digit f_g;
	f_g.mainLoop();
	//ros::spin();
	return 0;
	
}

// end of robo_5_0.cpp

