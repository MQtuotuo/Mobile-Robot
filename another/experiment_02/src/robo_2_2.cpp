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
#include <sstream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <math.h>

using namespace std;

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
		double vel_linear=0.00;
		double vel_angular=0.00;


		//for derivative data
		vector<double>deriv_data;
		//for potential leg range number
		vector<int>scannerData_leg;
		double derivative_data;

		int middle_leg;

	 //caculate derivative data
	 deriv_data.push_back(m_laserscan.ranges[0]);
	 for (int i = 1; i < 540; i++) {
		derivative_data = (double)((m_laserscan.ranges[i] - m_laserscan.ranges[i - 1])*0.5000);
		deriv_data.push_back(derivative_data);
		//outfile << derivative_data <<endl;
	}

	//find the potential leg candidate datas
	int front_leg;
	double edge_1;
	double edge_2;
	double length_leg;
	for (int i = 0; i < 539; i++) {

		if (deriv_data[i]<-1 && deriv_data[i + 1]>-0.5) {

			//value smaller than -1
			front_leg = i+1;
			edge_1 = m_laserscan.ranges[front_leg];
			while (deriv_data[i + 1]<1 && i + 1<539) {
				i++;
			}
			//value bigger than 1
			edge_2 = m_laserscan.ranges[i];
			//caqlculate the length of leg
			length_leg = sqrt(edge_1*edge_1+ edge_2*edge_2-2*edge_1*edge_2*cos((edge_2-edge_1)*3.1415926/360));
			//check which data is leg
			if (length_leg>0.06 && length_leg<0.2) {
					//scannerData_leg.push_back(front_leg);
					scannerData_leg.push_back((front_leg+i)/2);
			}
		}
	}
	
	if(scannerData_leg.size()>0){
		double x, y;
		int x_i = -1, y_i = -1;
		double distance_between_leg;
		//check if those legs are people
		for (int i = 0; i < scannerData_leg.size() - 1 && scannerData_leg.size()>0; i++) {
			x_i = scannerData_leg[i];
			y_i = scannerData_leg[i + 1];
			x = m_laserscan.ranges[x_i];
			y = m_laserscan.ranges[y_i];
			distance_between_leg = sqrt(x*x + y*y-2*x*y*cos((y-x)*3.1415926/360));
			//cout << distance_between_leg << " and " << x_i << " and " << y_i << endl;
			if (distance_between_leg < 0.4 && distance_between_leg >0.005 && (y_i- x_i<20)) {
				ROS_INFO("find a person between =%d and =%d", x_i,y_i);
				middle_leg=(y_i+x_i)/2;
				ROS_INFO("middle_leg =%d ", middle_leg);
			}
		}
//ROS_INFO(" position_middle=%lf", position_middle);
		//if the wall on the -y direction
		if(middle_leg <265){
	    	 vel_angular =  -0.20; 
	    	 vel_linear=0.20;
			ROS_INFO(" moving to right <45-265>: distance=%d,vel_angular=%lf", middle_leg,vel_angular);
		}


		if(265 <=middle_leg && middle_leg <= 275){
		     vel_angular = 0.00; 
		     vel_linear=0.20;
		ROS_INFO(" moving to right <265-275>: distance=%d,vel_angular=%lf", middle_leg,vel_angular);	
		}
 	  		
		//if the wall on the +y direction
		if(275<middle_leg){
		     vel_angular=0.20;
	   	     vel_linear= 0.20;
		ROS_INFO(" moving to left <275-495>: distance=%d,vel_angular=%lf", middle_leg,vel_angular);
		}

	  
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


