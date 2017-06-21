//
//
// experiment_01
// robo_02
//
// A very simple robot control named: Just_move
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>

#define pi 3.1415926
// Class definition
class Just_move {
public:
	Just_move();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();


protected:

	// Nodehandle for Just_move robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Just_move::Just_move() {
	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Just_move::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
}// end of Just_move constructor



// callback for getting laser values
void Just_move::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {

	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size());
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// robot shall stop, in case anything is closer than ...
void Just_move::emergencyStop() {

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


// this is the place where we will generate the commands for the robot
void Just_move::calculateCommand() {
	// see if we have laser data
	if( (&m_laserscan)->ranges.size() > 0)
	  {
		// set the roomba velocities
		// the linear velocity (front direction is x axis) is measured in m/sec
		// the angular velocity (around z axis, yaw) is measured in rad/sec
		//if the velocity.z bigger than 0.4,the roomba may shakes
		double vel_linear=0.15;
		double vel_angular=0.00;
		int shortest_laser=0;
		double shortest_laser_value=2.0;
		double keep_distance=0.75;

		// find the shortest distance between roomba and wall
		for(int i=0;i<=450;i++){
		   if(m_laserscan.ranges[i]<shortest_laser_value){
		       shortest_laser_value=m_laserscan.ranges[i];
		       shortest_laser=i;
		   }	
		}
		//move along right hand side 
		if(0<=shortest_laser && shortest_laser<90){
 		     if (shortest_laser_value >= keep_distance){ vel_angular = -0.2;  }
		     else {
				vel_angular = 0.3;
			}
			ROS_INFO(" shortest distance <0~90>: forward=%lf", shortest_laser_value);
		}
		//maintain certain distance between roomba and wall
      	        if (shortest_laser==90) {
		     if (shortest_laser_value >= keep_distance){ vel_angular = -0.2;  }
		     else {
				vel_angular = 0.3;
			}
			ROS_INFO(" shortest distance <90>: distance=%lf", shortest_laser_value);
		}
		//if the wall on the -y direction
		if(90 < shortest_laser && shortest_laser < 270){
			if (shortest_laser_value <= keep_distance)
			{	vel_angular = 0.3; 
				if((shortest_laser )*pi / 360 < 0.3){
				     vel_angular = ((shortest_laser )*pi / 360);
				}
			}
			ROS_INFO(" shortest distance <90--270>: distance=%lf", shortest_laser_value);
		}
		//maintain certain distance between roomba and wall
		if(shortest_laser==270){
		     if (shortest_laser_value >= keep_distance){ vel_angular = 0.3;  }
			else{
				vel_angular = -0.3;
			}
			ROS_INFO(" shortest distance <270>: distance=%lf", shortest_laser_value);
		
		}
		//if the wall on the +y direction
		if(270<shortest_laser && shortest_laser<450){
		    if (shortest_laser_value <= keep_distance)
		    {
		 	  vel_angular= -0.3;
			if ((shortest_laser-360 )*pi / 360 < 0.3 ){
				vel_angular = -((shortest_laser-360 )*pi / 360);
			}
		    }
			ROS_INFO(" shortest distance <270-450>: distance=%lf", shortest_laser_value);
		}
	
	       m_roombaCommand.linear.x  = vel_linear;
	       m_roombaCommand.angular.z = vel_angular;
	  } // end of if we have laser values

	
} // end of calculateCommands

//
void Just_move::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{
		calculateCommand();
		emergencyStop();

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
	ros::init(argc, argv, "Just_move");

	// get an object of type Just_move and call it robbi
	Just_move robbi;

	// main loop
	// make robbi do whatever robbi wants to do
	robbi.mainLoop();

	return 0;

}// end of main

// end of file: robo_02.cpp
