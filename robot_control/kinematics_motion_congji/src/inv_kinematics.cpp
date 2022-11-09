
/*
 * This source file is a inverse kinematic solution for two wheeled active decouppled casters.
 * inv_kinematics.cpp
 *
 *  Updated on: Mar 4, 2016
 *  Author: Tianjiangzheng 
 *  Email: ztj_1@163.com
 *  This file subscrible velocities in taskspace "task_vel", publish velocities for the motors of the caster "joint_vel".
 */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "parameters_mobilemanipulator.h"
#include <std_msgs/Float64MultiArray.h>
#define E_LIM 0.00001
ros::Publisher pub_joint_vel;
double beta,fai1,fai2,fai3,fai4;
double Vx, Vy, w;
sensor_msgs::JointState Joint_vel;
double current_time;


double find_steermotor_position(double a,double b,double c,double usesolution)
{
	double temp_k,temp1,solution_1,solution_2,return_value;
	temp1=sqrt(pow(b,2)-4*a*c);
	solution_1=2*atan2(-b+temp1,2*a);
	solution_2=2*atan2(-b-temp1,2*a);

	if (usesolution==1)
	{
		return_value=solution_1;
	}
	if (usesolution==2)
	{
		return_value=solution_2;
	}
	temp_k=int(return_value/(2*M_PI));
	return_value=return_value-2*temp_k*M_PI;
	if (return_value>=M_PI)
	{
		return_value=return_value-2*M_PI;
	}
	if (return_value<-M_PI)
	{
		return_value=return_value+2*M_PI;
	}
	return return_value;

}
double set_angular_value(double data)
{
	if (data>=M_PI)
	{
		data=data-2*M_PI;
	}
	if (data<-M_PI)
	{
		data=data+2*M_PI;
	}
	return data;

}
void callback_task_velocity (const std_msgs::Float64MultiArray::Ptr &msg){
	int i;
	double temp,temp_k;
	double wheelTargetAngular_1,wheelTargetAngular_2;
	double wheelTargetAngular_3,wheelTargetAngular_4;
	double set_wheel1,set_wheel2,set_wheel3,set_wheel4;
	double t_fai[4];
	double set_limit=0.0;
	//double set_limit=0.0095;
	current_time=ros::Time::now().toSec();
	Joint_vel.name.resize(8);
	Joint_vel.velocity.resize(8);
	//ROS_INFO("Task space velocity retrieved");
	Vx=msg->data[0];                  //velocity of x direction
	Vy=msg->data[1];                  //velocity of y direction
	w=msg->data[2];                   //velocity of rotation
	beta=msg->data[3];
	t_fai[0]=msg->data[4];               //current position of steer wheel 1
	t_fai[1]=msg->data[5];               //current position of steer wheel 2
	t_fai[2]=msg->data[6];               //current position of steer wheel 3
	t_fai[3]=msg->data[7];               //current position of steer wheel 4

	fai1=t_fai[0];
	fai2=t_fai[1];
	fai3=t_fai[2];
	fai4=t_fai[3];

	
	for (i=0;i<4;i++)
	{
		temp_k=int(t_fai[i]/(2*M_PI));
		t_fai[i]=t_fai[i]-2*temp_k*M_PI;
		if (t_fai[i]>=M_PI)
		{
			t_fai[i]=t_fai[i]-2*M_PI;
		}
		if (t_fai[i]<-M_PI)
		{
			t_fai[i]=t_fai[i]+2*M_PI;
		}
	}
	


	if ((Vx>-E_LIM)&&(Vx<=E_LIM))
	{
		Vx=0;
	}
	if ((Vy>-E_LIM)&&(Vy<=E_LIM))
	{
		Vy=0;
	}
	if ((w>-E_LIM)&&(w<=E_LIM))
	{
		w=0;
	}

	if ((Vx==0)&&(Vy==0)&&(w==0))
	{
		Joint_vel.name[0]="fai1";
		Joint_vel.velocity[0]=0;
		Joint_vel.name[1]="row1";
		Joint_vel.velocity[1]=0;
		Joint_vel.name[2]="fai2";
		Joint_vel.velocity[2]=0;
		Joint_vel.name[3]="row2";
		Joint_vel.velocity[3]=0;
		Joint_vel.name[4]="fai3";
		Joint_vel.velocity[4]=0;
		Joint_vel.name[5]="row3";
		Joint_vel.velocity[5]=0;
		Joint_vel.name[6]="fai4";
		Joint_vel.velocity[6]=0;
		Joint_vel.name[7]="row4";
		Joint_vel.velocity[7]=0;
	}
	else
	{

		double a, b,c;
		double temp_steerwheel1,temp_steerwheel2,temp_steerwheel3,temp_steerwheel4;
		double temp_rollwheel1,temp_rollwheel2,temp_rollwheel3,temp_rollwheel4;
		a=Vy+w*H*cos(beta)+w*B;
		b=2*(w*H*sin(beta)+Vx);
		c=-(Vy+w*H*cos(beta)-w*B);
		wheelTargetAngular_1=find_steermotor_position(a,b,c,1);
		a=Vy+w*H*cos(beta)+w*B;
		b=-2*(w*H*sin(beta)-Vx);
		c=-(Vy+w*H*cos(beta)-w*B);
		wheelTargetAngular_4=find_steermotor_position(a,b,c,1);

		a=-(Vy-w*H*cos(beta)-w*B);
		b=-2*(w*H*sin(beta)+Vx);
		c=(Vy-w*H*cos(beta)+w*B);
		wheelTargetAngular_2=find_steermotor_position(a,b,c,2);
		a=-(Vy-w*H*cos(beta)-w*B);
		b=2*(w*H*sin(beta)-Vx);
		c=(Vy-w*H*cos(beta)+w*B);
		wheelTargetAngular_3=find_steermotor_position(a,b,c,2);
		/*a=Vy+w*H*cos(beta)+w*B;
		b=2*(w*H*sin(beta)+Vx);
		c=-(Vy+w*H*cos(beta)-w*B);
		wheelTargetAngular_4=find_steermotor_position(a,b,c,1);
		a=Vy+w*H*cos(beta)+w*B;
		b=-2*(w*H*sin(beta)-Vx);
		c=-(Vy+w*H*cos(beta)-w*B);
		wheelTargetAngular_3=find_steermotor_position(a,b,c,1);


		a=-(Vy-w*H*cos(beta)-w*B);
		b=-2*(w*H*sin(beta)+Vx);
		c=(Vy-w*H*cos(beta)+w*B);
		wheelTargetAngular_1=find_steermotor_position(a,b,c,2);

		a=-(Vy-w*H*cos(beta)-w*B);
		b=2*(w*H*sin(beta)-Vx);
		c=(Vy-w*H*cos(beta)+w*B);
		wheelTargetAngular_2=find_steermotor_position(a,b,c,2);*/

		/*a=(Vy+w*H*cos(beta));
		b=(Vx+w*H*sin(beta));
		wheelTargetAngular_1=atan2(a,b);
		//if (wheelTargetAngular_1-fai1
		a=(Vy+w*H*cos(beta));
		b=(Vx-w*H*sin(beta));
		wheelTargetAngular_2=atan2(a,b);

		a=(Vy-w*H*cos(beta));
		b=(Vx+w*H*sin(beta));
		wheelTargetAngular_3=atan2(a,b);

		a=(Vy-w*H*cos(beta));
		b=(Vx-w*H*sin(beta));
		wheelTargetAngular_4=atan2(a,b);*/
		/*******wheel1********************/
		set_wheel1=set_angular_value(wheelTargetAngular_1-fai1);
		set_wheel2=set_angular_value(wheelTargetAngular_2-fai2);
		set_wheel3=set_angular_value(wheelTargetAngular_3-fai3);
		set_wheel4=set_angular_value(wheelTargetAngular_4-fai4);
		
		temp_steerwheel1=(-sin(fai1)*Vx+cos(fai1)*Vy+(H*cos(fai1+beta)-B)*w)/B;
		temp_steerwheel2=(-sin(fai2)*Vx+cos(fai2)*Vy-(H*cos(fai2-beta)-B)*w)/B;
		temp_steerwheel3=(-sin(fai3)*Vx+cos(fai3)*Vy-(H*cos(fai3+beta)-B)*w)/B;
		temp_steerwheel4=(-sin(fai4)*Vx+cos(fai4)*Vy+(H*cos(fai4-beta)-B)*w)/B;
		
		temp_rollwheel1=(cos(fai1)*Vx+sin(fai1)*Vy+H*sin(fai1+beta)*w)/R;
		temp_rollwheel2=(cos(fai2)*Vx+sin(fai2)*Vy-H*sin(fai2-beta)*w)/R;
		temp_rollwheel3=(cos(fai3)*Vx+sin(fai3)*Vy-H*sin(fai3+beta)*w)/R;
		temp_rollwheel4=(cos(fai4)*Vx+sin(fai4)*Vy+H*sin(fai4-beta)*w)/R;
			
		
		Joint_vel.name[0]="fai1";
		Joint_vel.name[2]="fai2";
		Joint_vel.name[4]="fai3";
		Joint_vel.name[6]="fai4";


		if (fabs(set_wheel1)>M_PI/2)
		{
			if ((temp_steerwheel1>=0)&&(temp_steerwheel1)<set_limit*2*M_PI)
			{
				Joint_vel.velocity[0]=set_limit*2*M_PI;
			}
			else if ((temp_steerwheel1>=-set_limit*2*M_PI)&&(temp_steerwheel1<0))
				{
				Joint_vel.velocity[0]=set_limit*2*M_PI;
				}
			else
			{
				Joint_vel.velocity[0]=temp_steerwheel1;
			}
		}
		else
		{
			Joint_vel.velocity[0]=temp_steerwheel1;
		}
		/*******wheel2********************/
		
		if (fabs(set_wheel2)>M_PI/2)
		{
			if ((temp_steerwheel2>=0)&&(temp_steerwheel2)<set_limit*2*M_PI)
			{
				Joint_vel.velocity[2]=set_limit*2*M_PI;
			}
			else if ((temp_steerwheel2>=-set_limit*2*M_PI)&&(temp_steerwheel2<0))
			{
				Joint_vel.velocity[2]=set_limit*2*M_PI;
			}
			else
			{
				Joint_vel.velocity[2]=temp_steerwheel2;
			}
		}
		else
		{
			Joint_vel.velocity[2]=temp_steerwheel2;
		}
		/*******wheel3********************/
		
		if (fabs(set_wheel3)>M_PI/2)
		{
			if ((temp_steerwheel3>=0)&&(temp_steerwheel3)<set_limit*2*M_PI)
			{
				Joint_vel.velocity[4]=set_limit*2*M_PI;
			}
			else if ((temp_steerwheel3>=-set_limit*2*M_PI)&&(temp_steerwheel3<0))
			{
				Joint_vel.velocity[4]=set_limit*2*M_PI;
			}
			else
			{
				Joint_vel.velocity[4]=temp_steerwheel3;
			}
		}
		else
		{
			Joint_vel.velocity[4]=temp_steerwheel3;
		}
		/*******wheel4********************/
		
		if (fabs(set_wheel4)>M_PI/2)
		{
			if ((temp_steerwheel4>=0)&&(temp_steerwheel4)<set_limit*2*M_PI)
			{
				Joint_vel.velocity[6]=set_limit*2*M_PI;
			}
			else if ((temp_steerwheel4>=-set_limit*2*M_PI)&&(temp_steerwheel4<0))
			{
				Joint_vel.velocity[6]=set_limit*2*M_PI;
			}
			else
			{
				Joint_vel.velocity[6]=temp_steerwheel4;
			}
		}
		else
		{
			Joint_vel.velocity[6]=temp_steerwheel4;
		}
		/*******rolling wheels******************/

		//Joint_vel.velocity[0]=temp_steerwheel1;
		//Joint_vel.velocity[2]=temp_steerwheel2;
		//Joint_vel.velocity[4]=temp_steerwheel3;
		//Joint_vel.velocity[6]=temp_steerwheel4;

		Joint_vel.name[1]="row1";
		Joint_vel.velocity[1]=temp_rollwheel1;
		Joint_vel.name[3]="row2";
		Joint_vel.velocity[3]=temp_rollwheel2;
		Joint_vel.name[5]="row3";
		Joint_vel.velocity[5]=temp_rollwheel3;
		Joint_vel.name[7]="row4";
		Joint_vel.velocity[7]=temp_rollwheel4;
	}
	/*ROS_INFO("fai1: %f", fai1*180/M_PI);
	ROS_INFO("fai2: %f", fai2*180/M_PI);
	ROS_INFO("fai3: %f", fai3*180/M_PI);
	ROS_INFO("fai4: %f", fai4*180/M_PI);
	ROS_INFO("Vy: %f", Vy);
	ROS_INFO("Vx: %f", Vx);
	ROS_INFO("w: %f", w);
	ROS_INFO("msg->data[3]: %f", msg->data[3]*180/M_PI);
	ROS_INFO("wheelTargetAngular_1: %f", wheelTargetAngular_1*180/M_PI);
	ROS_INFO("wheelTargetAngular_2: %f", wheelTargetAngular_2*180/M_PI);
	ROS_INFO("wheelTargetAngular_3: %f", wheelTargetAngular_3*180/M_PI);
	ROS_INFO("wheelTargetAngular_4: %f", wheelTargetAngular_4*180/M_PI);
	ROS_INFO("wheel_1_1: %f", set_wheel1*180/M_PI);
	ROS_INFO("wheel_2_2: %f", set_wheel2*180/M_PI);
	ROS_INFO("wheel_2_3: %f", set_wheel3*180/M_PI);
	ROS_INFO("wheel_2_4: %f", set_wheel4*180/M_PI);*/
	for (i=0;i<8;i++)
	{
		temp=Joint_vel.velocity[i];

		Joint_vel.velocity[i]=temp/(2*M_PI);
	}
	pub_joint_vel.publish(Joint_vel);

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "inv_kinematics");
	ros::NodeHandle nh;
	std::string topic_joint_vel="/joint_vel";
	pub_joint_vel=nh.advertise<sensor_msgs::JointState> (topic_joint_vel,1);
	std::string topic_task_vel="/task_vel";
	ros::Subscriber sub_joy=nh.subscribe(topic_task_vel,1,callback_task_velocity);
	ros::spin();
	return 0;
}
