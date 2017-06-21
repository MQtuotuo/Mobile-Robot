//
//
// experiment_01
// robo_04
//
// A simple robot control named: Guess_what
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


#define pi 3.14159265359

// Class definition
class Guess_what {
public:
	Guess_what();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);//<sensor_msgs/LaserScan.h>
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);//<geometry_msgs/PoseWithCovarianceStamped.h>
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);//<sensor_msgs/Joy.h>
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);//<sensor_msgs/Image.h>

	void emergencyStop();
	void calculateCommand();
	void mainLoop();

    double estimate_x;
	double estimate_y;
	double estimate_yaw;

protected:

	// Nodehandle for Guess_what robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;

    //ros::Subscriber m_imageSubscriber;

	image_transport::ImageTransport it_(m_nodeHandle);
    image_transport::Subscriber image_sub_ ;
    image_transport::Publisher image_pub_ ;

    // subscriber for amcl robo pose
    ros::Subscriber m_robotposeSubscriber;

    ros::Subscriber m_joy_sub_;

	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
    
	geometry_msgs::Twist m_roombaCommand;
  
    int linear_, angular_;
    double l_scale_, a_scale_;
};

// constructor
Guess_what::Guess_what(){
	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Guess_what::laserCallback, this);
    
    //m_imageSubscriber = m_nodeHandle.subscribe("/camera/image_raw", 1, &Guess_what::imageCallback, this);
    

    image_sub_ = it_.subscribe("/camera/image_raw", 1, &Guess_what::imageCallback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE );
	
	
    m_nodeHandle.param("axis_linear", linear_, linear_);
    m_nodeHandle.param("axis_angular", angular_, angular_);
    m_nodeHandle.param("scale_angular", a_scale_, a_scale_);
    m_nodeHandle.param("scale_linear", l_scale_, l_scale_);


    m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &Guess_what::poseCallback, this);
    
    m_joy_sub_ = m_nodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, &Guess_what::joyCallback, this);	
   
    m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Guess_what constructor


// callback for getting laser values 
void Guess_what::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback


void Guess_what::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{	
    estimate_x = msg->pose.pose.position.x;
    estimate_y = msg->pose.pose.position.y;
    //The range of yaw: -pi ~ pi.
    estimate_yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("POS X= %f, Y= %f, yaw =  %f",estimate_x, estimate_y,estimate_yaw);
}

void Guess_what::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
   m_roombaCommand.angular.z = a_scale_*joy->axes[angular_];
   m_roombaCommand.linear.x= l_scale_*joy->axes[linear_];

}


void Guess_what::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img_rgb;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_rgb = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::Mat gray;
    //cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
   
    
    cv::imshow("Display Image", img_rgb);

    cv::waitKey(3);
	
	//image_pub_.publish(cv_ptr->toImageMsg());
}


// here we go
// this is the place where we will generate the commands for the robot
void Guess_what::calculateCommand() {

	// please find out what this part of the robot controller is doing
	// watch the robot 
	// look into the source file 
	// and try to deduce the underlying idea

        // see if we have laser data available
	if( (&m_laserscan)->ranges.size() > 0)
          {
		double qs = 0.2 ;
                double qt = 0.0 ; 
                int c_r = 0 ;
                int c_l = 0 ;

                // first part
                for( int i=45;i<=270;i++)
                   if (m_laserscan.ranges[i] < 0.5)
                      c_r++; 

                // second part 
                for( int i=270;i<=490;i++)
                   if (m_laserscan.ranges[i] < 0.7)
                      c_l++ ;


                // third part 
                if (c_l > 5)
                   {
                        qs = 0.1, qt  = -0.25 ;
                   } // end of if

                // fourth part 
                if (c_r > 5)
                   {
                        qs = 0.1 ;
                        qt = +0.3 ;
                   } // end of if

                // set the roomba velocities
                // the linear velocity (front direction is x axis) is measured in m/sec
                // the angular velocity (around z axis, yaw) is measured in rad/sec
                m_roombaCommand.linear.x  = qs ;
                m_roombaCommand.angular.z = qt ;

          } // end of if we have laser data
	  
} // end of calculateCommands

// robot shall stop, in case anything is closer than ... 
void Guess_what::emergencyStop() {

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	{
		for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
		{
			if( m_laserscan.ranges[i] <= 0.30) {
				m_roombaCommand.linear.x = 0.0;
				m_roombaCommand.angular.z = 0.0;
			}// end of if too close
		}// end of for all laser beams
	} // end of if we have laser data
}// end of emergencyStop



//
void Guess_what::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{
		//calculateCommand();
		emergencyStop();

		ROS_INFO(" robot_04 dude runs with: .x=%+6.2f[m/s], .z=%+6.2f[rad/s]", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

		// send the command to the roomrider for execution
		m_commandPublisher.publish(m_roombaCommand);

		// spinOnce, just make the loop happen once
		ros::spinOnce();
		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();
	}
}// end of mainLoop


int main(int argc, char** argv) {

	// initialize
	ros::init(argc, argv, "Guess_what");

	// get an object of type Guess_what and call it dude 
	Guess_what dude  ;

	// main loop
	// make dude execute it's task 
	dude.mainLoop();

	return 0;

}// end of main

// end of file: robo_04.cpp

