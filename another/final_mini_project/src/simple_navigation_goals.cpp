#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Class definition
class simple_navigation_goals {
public:
	simple_navigation_goals();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void emergencyStop();
	void calculateCommand();
    int moveToGoal(double ,double, double);
	void mainLoop();

    double estimate_x;
	double estimate_y;
	double estimate_yaw;

protected:

	// Nodehandle for Guess_what robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
    // subscriber for amcl robo pose
    ros::Subscriber m_robotposeSubscriber;

	sensor_msgs::LaserScan m_laserscan;


	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
simple_navigation_goals::simple_navigation_goals() {
	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &simple_navigation_goals::laserCallback, this);

    m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &simple_navigation_goals::poseCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Guess_what constructor



// callback for getting laser values 
void simple_navigation_goals::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{http://answers.ros.org/question/164911/move_base-and-extrapolation-errors-into-the-future/
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback


void simple_navigation_goals::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{	
    estimate_x = msg->pose.pose.position.x;
    estimate_y = msg->pose.pose.position.y;
    //The range of yaw: -pi ~ pi.
    estimate_yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("POS X= %f, Y= %f, yaw =  %f",estimate_x, estimate_y,estimate_yaw);
}


// here we go
// this is the place where we will generate the commands for the robot
void simple_navigation_goals::calculateCommand() {

	// please find out what this part of the robot controller is doing
	// watch the robot 
	// look into the source file 
	// and try to deduce the underlying idea

        // see if we have laser data available
	if( (&m_laserscan)->ranges.size() > 0)
          {
                // set the roomba velocities
                // the linear velocity (front direction is x axis) is measured in m/sec
                // the angular velocity (around z axis, yaw) is measured in rad/sec
                m_roombaCommand.linear.x  = 0.0 ;
                m_roombaCommand.angular.z = 0.0 ;

          } // end of if we have laser data
	  
} // end of calculateCommands

// robot shall stop, in case anything is closer than ... 
void simple_navigation_goals::emergencyStop() {

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

int simple_navigation_goals::moveToGoal(double x ,double y, double z){

		//calculateCommand();
		emergencyStop();

        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);
        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
             ROS_INFO("Waiting for the move_base action server to come up");	
        }
         move_base_msgs::MoveBaseGoal goal;	
     	//goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.frame_id = "map";

        //we'll send a goal to the robot to move 1 meter forward
        
        goal.target_pose.header.stamp = ros::Time::now();
        //goal.target_pose.pose.position.x = 1.0;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y; 
        goal.target_pose.pose.orientation.w = z;
        

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("reach the goal");
       return 1;
      }
    else{
      ROS_INFO("The base failed for some reason"); 
       return -1;
 }
}
//
void simple_navigation_goals::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);
        int first =0;
        int second =0;
	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{

        if(first==0||first==-1){
           first= moveToGoal(22.43 ,-39.10, 1.0);
        }
        if(second==0||second==-1){
        second= moveToGoal(33.30 ,-41.22, 1.0);
        }
        if(first==1&&second==1)
            break;
// send the command to the roomrider for execution
		//m_commandPublisher.publish(m_roombaCommand);
		// spinOnce, just make the loop happen once
		ros::spinOnce();
		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();
    }
}// end of mainLoop


int main(int argc, char** argv) {

	// initialize
	ros::init(argc, argv, "simple_navigation_goals");

    // ros::Time::init();
	//ros::Duration(5).sleep();
	// get an object of type Guess_what and call it sng
	simple_navigation_goals sng  ;


	// main loop
	// make sng execute it's task 
	sng.mainLoop();

	return 0;

}// end of main

// end of file: robo_04.cpp

