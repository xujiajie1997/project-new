
/*
 * mr_inv_kinematics_node.cpp
 *
 *  Created on: Aug 14, 2014
 *      Author: robot
 */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <string>

#define AccT 4
#define DccT 4
#define freq 100
#define time_lenth 4
#define vel 0.4
#define vel1 0.1
#define vel_roll 0.15

ros::Publisher pub_originalvel;
int start_flag=1;
std::string ss_temp;
int command_mode=0;
int flag_com=0;
int count=0;
void velocity_planner()
{
geometry_msgs::Twist out_vel;

if (start_flag==1)
{
count++;
ROS_INFO("COUNT:%d",count);
switch (command_mode) {
	case 0:
	{
		
		out_vel.linear.x=vel;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		break;
	}
	case 1:
	{
		out_vel.linear.x=0;
		out_vel.linear.y=vel;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		break;
	}

	case 2:
	{
		out_vel.linear.x=-vel;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		
		break;
	}
	case 3:
	{
		out_vel.linear.x=0;
		out_vel.linear.y=-vel;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;;
		
		break;
	}
	case 4:
	{
		out_vel.linear.x=0.4;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		//out_vel.angular.z=0;
		out_vel.angular.z=vel_roll;
		break;
	}
/*	case 5:
	{
		out_vel.linear.x=-vel;
		out_vel.linear.y=-vel;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		break;
	}
*/
	case 5:
	{
		out_vel.linear.x=0;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		
		break;
	}

	default: break;

	}
	
}

else
{

switch (flag_com) {
	case 1:
	{
		
		out_vel.linear.x=vel1;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		break;
	}
	case 2:
	{
		out_vel.linear.x=0;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=vel1;
		
		break;
	}

	case 3:
	{
		out_vel.linear.x=-vel1;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		break;
	}
	case 4:
	{
		out_vel.linear.x=0;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=-vel1;
		
		break;
	}
case 5:
	{
		out_vel.linear.x=0;
		out_vel.linear.y=-vel1;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		break;
	}
case 6:
	{
		out_vel.linear.x=0;
		out_vel.linear.y=vel1;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		break;
	}



case 7:
	{
		out_vel.linear.x=0;
		out_vel.linear.y=0;
		out_vel.linear.z=0;
		out_vel.angular.x=AccT;
		out_vel.angular.y=DccT;
		out_vel.angular.z=0;
		
		break;
	}

	default: break;

	}

}

pub_originalvel.publish(out_vel);
}
 
void callback_socket_com(const std_msgs::String::ConstPtr& msg)
{

	ss_temp=msg->data.c_str();
	if ((strcmp(ss_temp.data(),"automove"))==0)
	 {
		start_flag=1;

	}	
	if ((strcmp(ss_temp.data(),"stop"))==0)
	{
		start_flag=0;
		flag_com=7;
	}

	if ((strcmp(ss_temp.data(),"left"))==0)
	 {
		flag_com=1;
start_flag=0;

	}	
	if ((strcmp(ss_temp.data(),"turnleft"))==0)
	 {
		flag_com=2;
start_flag=0;
	}
	if ((strcmp(ss_temp.data(),"right"))==0)
	 {
		flag_com=3;
start_flag=0;

	}
	if ((strcmp(ss_temp.data(),"turnright"))==0)
	 {
		flag_com=4;
start_flag=0;

	}		
	if ((strcmp(ss_temp.data(),"forward"))==0)
	 {
		flag_com=5;
start_flag=0;
	}

	if ((strcmp(ss_temp.data(),"back"))==0)
	 {
		flag_com=6;
start_flag=0;
	}			
	
	ROS_INFO("data_callbacked");

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "autoplanner");
	ros::NodeHandle nh;
	ros::Rate loop_rate(freq);
	std::string pub_originalvel_="/original_vel";
    pub_originalvel=nh.advertise<geometry_msgs::Twist> (pub_originalvel_,10);
	std::string topic_socket_="/socket_com";
    ros::Subscriber topic_socket=nh.subscribe(topic_socket_,1,callback_socket_com);
	
	
	
    while(ros::ok())
    {
if (count<=time_lenth*freq)
{
command_mode=0;

}

if ((count>time_lenth*freq)&&(count<=2*time_lenth*freq))
{
command_mode=1;

}

if ((count>2*time_lenth*freq)&&(count<=3*time_lenth*freq))
{
command_mode=2;

}
if ((count>3*time_lenth*freq)&&(count<=4*time_lenth*freq))
{
command_mode=3;

}
if ((count>4*time_lenth*freq)&&(count<=2400))//4080
{
command_mode=4;
}
if ((count>2400)&&(count<=2800))//4800
{
command_mode=5;
}

if (count>2800)//4800
{
count=0;
}
/*
if ((count>5*time_lenth*freq)&&(count<=6*time_lenth*freq))
{
command_mode=5;

}

if (count>6*time_lenth*freq)
{
command_mode=6;
count=0;

}
*/
 velocity_planner();
	
//count++;
//ROS_INFO("The current time,%d",count);
     
	ros::spinOnce();
      loop_rate.sleep();
	}
	return 0;
}
