//Ubuntu 18.04, Ros Melodic

//Declare necessary libraries for this program
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

//declare variables and Posestamped object to be used
using namespace std;
float x_val = 0;
float y_val = 0;
float z_val = 0;
geometry_msgs::PoseStamped pose;
mavros_msgs::State current_state;

//Fill the current_state object with values gotten from the subscribed node mavros/state channel
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//Fill pose object with offset values gotten from the qr_scanning vision program
void qr_code_data(const geometry_msgs::PoseStamped msg){
	pose.pose.position.y = msg.pose.position.x;
	pose.pose.position.z = msg.pose.position.y;
	pose.pose.position.x = 1.5 - msg.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
	
//Subscribed channels
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
//ch_mov is the channel where the vision program publishes it's gotten x, y and z values			
    ros::Subscriber qr_data = nh.subscribe<geometry_msgs::PoseStamped>
            ("ch_mov", 10, qr_code_data);
			
//Publishing channels to the drone			
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10);
	ros::Publisher filler_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);	
			
    //20Hz refresh rate so that the drone gets updated frequently enough
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

//Initial filling of the pose object
//pose orientation can be added here
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
	

	mavros_msgs::PositionTarget filler_loc;
	filler_loc.header.stamp = ros::Time::now();

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
		
	filler_pos_pub.publish(filler_loc);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
		pose.header.stamp = ros::Time::now();
		filler_loc.header.stamp = ros::Time::now();

		local_pos_pub.publish(pose);
		filler_pos_pub.publish(filler_loc);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
