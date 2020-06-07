#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "leap_motion/Finger.h"
#include "leap_motion/Bone.h"
#include "leap_motion/Gesture.h"
#include "leap_motion/leapros.h"

#include <iostream>

#include "../inc/lmc_listener.h"
#include "Leap.h"


// INIt Variables
float speed = 0;
float rot = 0;
float in_min = 0.1;
float out_min = -0.1;
float in_max = 0.3;
float out_max = 0.1;

//This tutorial demonstrates simple receipt of messages over the ROS system.
void LeapPoseCallback(const leap_motion::Human::ConstPtr& LeapPosemsg)
{
	//speed = LeapPosemsg->direction.y;
	//rot = LeapPosemsg->direction.x;
	//ROS_INFO("Number of hands [%i], Grab Strength [%f]", LeapPosemsg->nr_of_hands, LeapPosemsg->right_hand.grab_strength);
	//ROS_INFO("X: [%f], Y: [%f], Z: [%f],", LeapPosemsg->right_hand.direction.x, LeapPosemsg->right_hand.direction.y, 	LeapPosemsg->right_hand.direction.z);
	

	//ROS_INFO("X: [%f], Y: [%f], Z: [%f]", LeapPosemsg->right_hand.palm_center.x,LeapPosemsg->right_hand.palm_center.y,LeapPosemsg->right_hand.palm_center.z);


	
	speed = LeapPosemsg->right_hand.palm_center.y;
	rot = -(LeapPosemsg->right_hand.direction.x);


	// mapping speed from 0.1-0.3 to -0.1-0.1
	speed = (speed-in_min)*(out_max-out_min)/(in_max-in_min) + out_min;
	if (speed < -0.1){
		speed = -0.1;	
	}
	else if (speed > 0.1) {
		speed = 0.1;
	}
	else {
		speed = speed;
	}

// Check if hand is interpreted in stop or drive state

	if(LeapPosemsg->nr_of_hands >=1 && LeapPosemsg->right_hand.grab_strength<=0.15){
	speed = speed;
	rot = rot;
	}
	else{
		speed = 0;
		rot = 0;
	}

	//ROS_INFO("speed: [%f]", speed);


}

int main(int argc, char **argv){
	// Name of node
  ros::init(argc, argv, "leap_controller");

  ros::NodeHandle n;
	//subscriber
  ros::Subscriber sub = n.subscribe("/leap_motion/leap_filtered", 1000, LeapPoseCallback);
	//publisher
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	while(ros::ok()){
	// Create Twist message
	  geometry_msgs::Twist twist;
	  

	  twist.linear.x = (speed)*15/2;
	  twist.linear.y =0;
	  twist.linear.z = 0;

	  twist.angular.x = 0;
	  twist.angular.y = 0;
	  twist.angular.z = rot;
	  //ros::spin();
	  pub.publish(twist);
	  ros::spinOnce();
	}


  return 0;
}