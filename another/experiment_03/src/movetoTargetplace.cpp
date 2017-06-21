//
// Lab Mobile Robots WS2014/2015
//
// University of Bonn
// Department of Computer Science 
//
// task63.cpp
//
//
// Author: University of Bonn, Autonomous Intelligent Systems 
//

// Includes for ROS and for the necessary messages between the nodes 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define pi 3.14159265359


// Class definition 
class Braitenberg {
public:
	Braitenberg();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();
	double estimate_x;
	double estimate_y;
	double estimate_yaw;
	double dist;
    double rest_times;


protected:

	// Nodehandle for Braitenberg node
	ros::NodeHandle m_nodeHandle;
	// Subscriber and Membervariable for controlling the laser
	ros::Subscriber m_laserSubscriber;
    
    ros::Subscriber m_robotposeSubscriber;
	//ros::Subscriber m_robotorientationSubscriber;
	
    sensor_msgs::LaserScan m_laserscan;

	// Publisher and Membervariable for controlling the robot
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// Constructor
Braitenberg::Braitenberg() {
    // Node is establised in root-Namespace 
    ros::NodeHandle m_nodeHandle("/");

    // Initialising the message handler 
    m_laserSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Braitenberg::laserCallback, this);

    m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &Braitenberg::poseCallback, this);

    m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
            
    rest_times = 0;
        
}

// Callback for the Laser
void Braitenberg::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    if( m_laserscan.ranges.size() < scanData->ranges.size())
	    m_laserscan.ranges.resize((size_t)scanData->ranges.size());
    for(unsigned int i = 0; i<scanData->ranges.size(); i++)
    {
	    m_laserscan.ranges[i] = scanData->ranges[i];
    }
}


void Braitenberg::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{	
    estimate_x = msg->pose.pose.position.x;
    estimate_y = msg->pose.pose.position.y;
    //The range of yaw: -pi ~ pi.
    estimate_yaw = tf::getYaw(msg->pose.pose.orientation);
    /*  AmclNode::getYaw(tf::Pose& t)
        {
           double yaw, pitch, roll;
          btMatrix3x3 mat = t.getBasis();
           mat.getEulerYPR(yaw,pitch,roll);
          return yaw;
        }*/
    //ROS_INFO("POS X= %f, Y= %f, yaw =  %f",estimate_x, estimate_y,estimate_yaw);
}

// Guess what i am doing 
void Braitenberg::emergencyStop() {
	for(unsigned int i=0; i<m_laserscan.ranges.size(); i++)
	{
		if( m_laserscan.ranges[i] <= 0.40) {
			//m_roombaCommand.linear.x = 0.0;
			//m_roombaCommand.angular.z = 0.0;
			if(dist > 1){
                m_roombaCommand.linear.x = -0.4;
                if(i<= 270){m_roombaCommand.angular.z = 0.3;}
                else {m_roombaCommand.angular.z = -0.3;}
			}
		}
		
	}
	
}

// 
// Calculate the command to steer the robot
//
void Braitenberg::calculateCommand() {
	if (m_laserscan.ranges.size() > 0)
	{
	    double turn, speed;
	    turn = 0;
	    speed = 0.3;
	    int shortest_bean_num = 90;
	    float shortest_bean = 10;
	    //float dist=0.6;
	
	           
        int count_left  = 0 ;
	    int count_right = 0 ;

	
	    for( int i=45;i<=270;i++) 
	      {
		    if (m_laserscan.ranges[i] < 0.5) 
		      count_right = count_right +1 ;
	      }// end of for 
	      
	    // is there something to yourleft ?
	    for( int i=270;i<=490;i++) 
	      {
		    if (m_laserscan.ranges[i] < 0.5)
		      count_left = count_left +1 ;
	      } // end of for

	
	    // This is the aim position
        double aim_x = 25.0;
	    double aim_y = -39.0;
        if(rest_times > 10){
             aim_x = 34.0;
	         aim_y = -49.0;
        }
	
       //theta is the angle from the amcl_estimate position to the aim position
        double theta = atan((aim_y - estimate_y)/(aim_x - estimate_x));
        
       //changing theta with the same range of yaw
        if((aim_x - estimate_x)<0 && (aim_y - estimate_y) >0){
            theta = theta + pi;
        }
        else if ((aim_x - estimate_x)<0 && (aim_y - estimate_y) <0){
            theta = theta - pi;
        }
        
        double delta_angle = theta - estimate_yaw;
        
        // turn to the target direction
        if (delta_angle > pi){
            turn = -0.4;
        }
        else if (delta_angle >0 &&  delta_angle< pi ){
            turn = 0.4;
        
        }
        
        else if (delta_angle < -pi ){
            turn = 0.4;
        
        }
        else if (delta_angle < 0 &&  delta_angle >-pi ){
            turn = -0.4;
        }
        
        
	    // turn if there is somthing at your left side
	    if (count_left > 5 && count_left > count_right) 
	       {
            speed = 0.2; 
		    turn  = -0.4 ; 
	       } 
        // turn if there is somthing at your right side
	    else if (count_right > 5) 
	       {
            speed = 0.2; 
		    turn  = +0.4 ; 
	       } // end of if    

	    

        dist = sqrt(pow(aim_y - estimate_y,2) + pow(aim_x - estimate_x,2));
        
        if(dist < 1){
            speed = 0;
            turn = 0;
            ROS_INFO("POS aim_X= %f, aim_Y= %f, dist = %f",aim_x, aim_y,dist);
            rest_times +=1;
        } 
        
       
        //ROS_INFO("estimate_x %f,estimate_y %f,theta: %f, yaw: %f, turn: %f, ",estimate_x,estimate_y , theta, estimate_yaw, turn);
	    m_roombaCommand.linear.x = speed;
	    m_roombaCommand.angular.z = turn;
	}

}// end of void Baraitenberg

// Main loop 
void Braitenberg::mainLoop() {
	// determines the number of loops per second 
	ros::Rate loop_rate(20);

	// loop stops if the node stops, e.g. by getting akill signal 
	while (m_nodeHandle.ok())
	{
		calculateCommand();
		emergencyStop();

		//ROS_INFO(" New Forward: %f - Turn: %f [rad/s]", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

		// send the motor command to the roomba 
		m_commandPublisher.publish(m_roombaCommand);

		// SpinOnce, just make the loop operate once 
		ros::spinOnce();
		// sleep, to keep 50ms delay
		loop_rate.sleep();
	}
}

int main(int argc, char** argv) {

	// Initialize
	ros::init(argc, argv, "Braitenberg");

    ros::Time::init();

	ros::Duration(5).sleep();

	Braitenberg b;

	// main loop
	b.mainLoop();

	return 0;
}

// End of file: task63.cpp

