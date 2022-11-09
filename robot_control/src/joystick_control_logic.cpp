/*
 *  This source file is a joystic_congtrol solution for omni-directional robot.
 *  joystick_control_logic.cpp
 *  This node is valid for the joy_stick
 *  Updated on: April 21, 2020
 *  Author: Tianjiangzheng
 *  Email: ztj_1@163.com
 *  This file subscribe velocities in task space "joy","extvel", publish velocities for the motors of the caster "original velocities".
 *  Subscribe /joy node and /extvel , and published  /original_vel as task space velocity command,  /scurve_params as the acceleration and deceleration parameters for the s-curve velocity planner);
 */
#include "std_msgs/Int32MultiArray.h"
#include"geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include"ros/ros.h"
#include <math.h>

#define USE_ULTRASONIC 0
#define NO_EXTVEL_MAX 5
#define MAXIMUM_VEL 1.0
#define DELAY_TIME 200
#define FREQ 100

class JoystickControl {
public:
	JoystickControl(std::string topic_joystick,std::string topic_joyctrlvel,std::string topic_extvel,std::string topic_velparams,std::string topic_readserial,std::string topic_status,ros::NodeHandle nh);
	~JoystickControl(){
	};
	void JoystickOutput(void);

private:
	int joy_mode;
	int systerm_count;
	int delay_time;

	double oldtime;
	double time_count;
	bool watchdog_flag;
	bool delayFlag;

	ros::NodeHandle nh_;

	std::string topic_joystick_;
	//std::string topic_joyctrlvel_;
	std::string topic_extvel_;
	std::string topic_readserial_;
	std::string topic_status_;
	std::string topic_velparams_;
	std_msgs::String echo_status;
	ros::Publisher pub_velparams;
	ros::Publisher pub_joycontrol;
	ros::Publisher pub_status;

	ros::Subscriber sub_joystick;
	ros::Subscriber sub_extvel;
	ros::Subscriber sub_serial;
	sensor_msgs::Joy joy_bts;
	geometry_msgs::Twist vel_pub,ext_vel,vel_params;
	std_msgs::String ultro;

	void callback_Joystick(const sensor_msgs::Joy::Ptr &topic_joystick);
	void callback_serialport(const std_msgs::String::Ptr &topic_serial);
	void callback_extvel(const geometry_msgs::Twist::Ptr &topic_extvel);
};

JoystickControl::JoystickControl(std::string topic_joystick,std::string topic_joyctrlvel,std::string topic_extvel,std::string topic_velparams,std::string topic_readserial,std::string topic_status,ros::NodeHandle nh)
{
	nh_ = nh;
	joy_mode=0;
	delayFlag=0;
	time_count=0;
	systerm_count=0;
	watchdog_flag=0;
	delay_time=DELAY_TIME;

	joy_bts.buttons.resize(8);
	joy_bts.axes.resize(4);
	joy_bts.axes[3] = 0.3;
	echo_status.data="Joy stick started";


	//topic_joyctrlvel_=topic_joyctrlvel;
	topic_readserial_=topic_readserial;
	topic_joystick_=topic_joystick;
	topic_extvel_=topic_extvel;
	topic_status_=topic_status;
	topic_velparams_=topic_velparams;
	oldtime=ros::Time::now().toSec();
	sub_joystick = nh_.subscribe(topic_joystick_, 10, &JoystickControl::callback_Joystick, this);
	sub_extvel = nh_.subscribe(topic_extvel_, 10, &JoystickControl::callback_extvel, this);
	sub_serial = nh_.subscribe(topic_readserial_, 10, &JoystickControl::callback_serialport, this);
	pub_velparams = nh_.advertise<geometry_msgs::Twist>(topic_velparams, 10);
	pub_joycontrol = nh_.advertise<geometry_msgs::Twist>(topic_joyctrlvel, 10);
	pub_status=nh_.advertise<std_msgs::String>(topic_status, 10);
}
void JoystickControl::callback_extvel(const geometry_msgs::Twist::Ptr &topic_extvel)
{	
	oldtime=ros::Time::now().toSec();
	ext_vel.linear.x=topic_extvel->linear.x;
	ext_vel.linear.y=topic_extvel->linear.y;
	ext_vel.linear.z=topic_extvel->linear.z;
	ext_vel.angular.x=topic_extvel->angular.x;
	ext_vel.angular.y=topic_extvel->angular.y;
	ext_vel.angular.z=-topic_extvel->angular.z;
}

void JoystickControl::callback_serialport(const std_msgs::String::Ptr &topic_serial)
{
	std::string temp;
	ultro.data=topic_serial->data;
	temp=ultro.data.c_str();

	if (USE_ULTRASONIC==true)
	{
		if (temp=="obstancled")
		{
			delayFlag=1;
			systerm_count=0;
			ROS_INFO("Received Ultrasonic Obstacle!");
		}
		else
		{
			delayFlag=0;
			systerm_count=0;
			//ROS_INFO("Obstacle cleard!");
		}
	}

}


void JoystickControl::JoystickOutput(void)
{
	std::string serialport = ultro.data.c_str();
	switch (joy_mode) 
	{
	case 0:                   //Stop command   //Set velocity to zero
	{
		vel_pub.linear.x=0;
		vel_pub.linear.y=0;
		vel_pub.linear.z=0;
		vel_pub.angular.x=0;
		vel_pub.angular.y=0;
		vel_pub.angular.z=0;
		break;
	}
	case 1:    //Homing command   //Set velocity to zero first then send homing flag
	{
		vel_pub.linear.x=0;
		vel_pub.linear.y=0;
		vel_pub.linear.z=0;
		vel_pub.angular.x=0;
		vel_pub.angular.y=0;
		vel_pub.angular.z=0;
		ROS_INFO("Robot is set to homing mode");
		break;
	}
	case 2:
	{
		if(joy_bts.buttons[6]==true)   //Send oblique velocity automatically
		{
			vel_pub.linear.x=joy_bts.axes[3]*0.707;    //0.707 is used for limit the maximum velocity
			vel_pub.linear.y=joy_bts.axes[3]*0.707;
			vel_pub.linear.z=0;
			vel_pub.angular.x=0;
			vel_pub.angular.y=0;
			vel_pub.angular.z=0;
		}
		else if (joy_bts.buttons[7]==true)   //Send oblique velocity automatically
		{
			vel_pub.linear.x=-joy_bts.axes[3]*0.707;
			vel_pub.linear.y=-joy_bts.axes[3]*0.707;
			vel_pub.linear.z=0;
			vel_pub.angular.x=0;
			vel_pub.angular.y=0;
			vel_pub.angular.z=0;
		}
		else
		{
			vel_pub.linear.x=joy_bts.axes[1]*joy_bts.axes[3];
			vel_pub.linear.y=joy_bts.axes[0]*joy_bts.axes[3];
			vel_pub.linear.z=0;
			vel_pub.angular.x=0;
			vel_pub.angular.y=0;
			vel_pub.angular.z=joy_bts.axes[2]*joy_bts.axes[3];
		}

		vel_params.linear.x=4;                // Acceleration parameter of x-direction
		vel_params.linear.y=4;				  // Acceleration parameter of y-direction
		vel_params.linear.z=3;				  // Acceleration parameter of w-direction
		vel_params.angular.x=1.5;				  // Deceleration parameter of x-direction
		vel_params.angular.y=1.5;				  // Deceleration parameter of y-direction
		vel_params.angular.z=0.8;				  // Deceleration parameter of w-direction
		pub_velparams.publish(vel_params);
		break;
	}
	case 3:
	{
		double temp_time;
		temp_time=ros::Time::now().toSec();
		time_count=temp_time-oldtime;

		if ((time_count>=NO_EXTVEL_MAX)&&(delayFlag==0))
		{
			watchdog_flag=0;
			ROS_WARN("External velocity mode, but the external device didn't send commands!");
			if (time_count>100)
			{
				//ROS_INFO("time_count,%f",time_count);
				time_count=100;
			}
		}
		if (time_count<NO_EXTVEL_MAX)
		{
			watchdog_flag=1;
			time_count=0;
		}
		if(watchdog_flag==1)
		{
			//ROS_INFO("delayFlag,%f",double(delayFlag));
			if(delayFlag==1)
			{
				systerm_count++;
				if (systerm_count<delay_time)
				{
					ROS_INFO("systerm_count,%d",systerm_count);
					ROS_WARN("System stopped, waiting for time counter decreased to zero,%d",systerm_count);
					vel_pub.linear.x=0;
					vel_pub.linear.y=0;
					vel_pub.linear.z=0;
					vel_pub.angular.x=0;
					vel_pub.angular.y=0;
					vel_pub.angular.z=0;
				}
				else
				{
					delayFlag=0;
					systerm_count=0;
				}
			}
			else
			{
				vel_pub.linear.x=ext_vel.linear.x;
				vel_pub.linear.y=ext_vel.linear.y;
				vel_pub.linear.z=0;
				vel_pub.angular.x=0;
				vel_pub.angular.y=0;
				vel_pub.angular.z=ext_vel.angular.z;
			}
		}
		else
		{
			vel_pub.linear.x=0;
			vel_pub.linear.y=0;
			vel_pub.linear.z=0;
			vel_pub.angular.x=0;
			vel_pub.angular.y=0;
			vel_pub.angular.z=0;
		}
		break;
	}

	default: break;
	}

	if (vel_pub.linear.x>=MAXIMUM_VEL)
		vel_pub.linear.x=MAXIMUM_VEL;
	if (vel_pub.linear.x<=-MAXIMUM_VEL)
		vel_pub.linear.x=-MAXIMUM_VEL;
	if (vel_pub.linear.y>=MAXIMUM_VEL)
		vel_pub.linear.y=MAXIMUM_VEL;
	if (vel_pub.linear.y<=-MAXIMUM_VEL)
		vel_pub.linear.y=-MAXIMUM_VEL;
	if (vel_pub.angular.z>=MAXIMUM_VEL)
		vel_pub.angular.z=MAXIMUM_VEL;
	if (vel_pub.angular.z<=-MAXIMUM_VEL)
		vel_pub.angular.z=-MAXIMUM_VEL;

	pub_joycontrol.publish(vel_pub);

}

void JoystickControl::callback_Joystick(const sensor_msgs::Joy::Ptr &topic_joystick)
{
	joy_bts.buttons[0]=topic_joystick->buttons[8];//Stop button (Back on Joy stick)
	joy_bts.buttons[1]=topic_joystick->buttons[9];//Homing button (Start on Joy stick)
	joy_bts.buttons[2]=topic_joystick->buttons[7];//Set joy stick mode to manually control; //(Button right down)
	joy_bts.buttons[3]=topic_joystick->buttons[5];// Set joy stick mode to accept external velocity command; //(Button right up)
	joy_bts.buttons[4]=topic_joystick->buttons[10];// Keep for future use
	joy_bts.buttons[5]=topic_joystick->buttons[11];// Keep for future use

	if(joy_bts.buttons[0]==true)
	{
		joy_mode=0;                //Send stop command
		echo_status.data="Stop";
		ROS_INFO("Robot is set to stop mode");
	}
	if(joy_bts.buttons[1]==true)
	{
		joy_mode=1;    			   //Send homing command
		echo_status.data="Homing";
		ROS_INFO("Robot is set to stop homing mode");
	}
	if(joy_bts.buttons[2]==true)
	{
		joy_mode=2;                 //Changed to manually mode
		echo_status.data="Manually";
		ROS_INFO("Robot is set to manually control mode");
	}
	if(joy_bts.buttons[3]==true)
	{
		joy_mode=3;   				//Changed to accept external velocity command mode
		if(delayFlag == 0)
		{
			echo_status.data="External";
			ROS_INFO("Robot is set to external velocity control mode");
		}
		else
		{
			echo_status.data="External_obstancled";
			ROS_INFO("Obstacle avoidance is starting");
		}

	}
	if(joy_bts.buttons[4]==true)
	{		joy_mode=4;     }     //Keep for future use
	if(joy_bts.buttons[5]==true)
	{		joy_mode=5;     }     // Keep for future use


	/**************buttons[4] and buttons[6] used to control the robot increase the speed and decrease the speed*************************/
	if (joy_mode==2)
	{

		/**************buttons[1] and buttons[3] used to control the robot run in oblique mode*************************/
		if ((topic_joystick->buttons[1] == true)&&(topic_joystick->buttons[3] == false))
		{
			joy_bts.buttons[7]=true;
			echo_status.data="Manually_Noblique";
			ROS_INFO("Negative oblique command sent");
		}
		else if ((topic_joystick->buttons[1] == false)&&(topic_joystick->buttons[3] == true))
		{
			joy_bts.buttons[6]=true;
			echo_status.data="Manually_Poblique";
			ROS_INFO("Positive oblique command sent");
		}
		else
		{
			joy_bts.buttons[6]=false;
			joy_bts.buttons[7]=false;
		}


		if ((topic_joystick->buttons[4] == true) && (topic_joystick->buttons[6] == false)&& (joy_bts.axes[3] <1))
		{
			joy_bts.axes[3] += 0.1 ; //Task space speed increase
		}
		if ((topic_joystick->buttons[4] == false) && (topic_joystick->buttons[6] == true)&& (joy_bts.axes[3] > 0.1))
		{
			joy_bts.axes[3] -= 0.1; //Task space speed decrease
		}

		if (joy_bts.axes[3]>=1)
		{
			joy_bts.axes[3]=1;
		}
		if (joy_bts.axes[3]<=0.1)
		{
			joy_bts.axes[3]=0.1;
		}
		ROS_INFO("Current speed is now,%f",joy_bts.axes[3]);

		joy_bts.axes[0] = 1*(topic_joystick->axes[4]);   //根改
		joy_bts.axes[1] = 1*(topic_joystick->axes[5]);
		/**************buttons[0] and buttons[2] used to control the robot rotate direction*************************/
		if ((topic_joystick->buttons[0] == true) && (topic_joystick->buttons[2] == false))
		{
			joy_bts.axes[2] = -topic_joystick->buttons[0]; //Robot rotate about clockwise
		}
		else if ((topic_joystick->buttons[0] == false) && (topic_joystick->buttons[2] == true))
		{
			joy_bts.axes[2] = topic_joystick->buttons[2]; //Robot rotate about anti-clockwise
		}
		else
		{
			joy_bts.axes[2] = 0;
		}



	}
	//ROS_INFO("echo_status,%s",echo_status);
	pub_status.publish(echo_status);
}

int main(int argc, char** argv) {
	double current_time, actFrequency;
	ros::init(argc, argv, "joystick_control");
	ros::NodeHandle nh;
	JoystickControl Robot("/joy", "/original_vel","/cmd_extvel","/vel_params","/read_serial","/joy_status",nh);
	ros::Rate loop_rate(FREQ);
	current_time=ros::Time::now().toSec();
	ROS_INFO("Joy stick start, no mode set");
	while (ros::ok()) {
		Robot.JoystickOutput();
		actFrequency=1/(ros::Time::now().toSec() - current_time);
		if (actFrequency<FREQ*0.7)
		{
			ROS_WARN("Warning: The actual execute frequency is lower than 70 percent of set frequency which is %f, check the cycle frequency.",actFrequency);
		}
		current_time=ros::Time::now().toSec();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
