
/*
 * This source file is a velocity plan solution for two wheeled active decouppled casters.
 * inv_kinematics.cpp
 *
 *  Updated on: Mar 11, 2016
 *  Author: Tianjiangzheng
 *  Email: ztj_1@163.com
 *  This file subscrible velocities in taskspace "original_vel", publish velocities for the motors of the caster "planned velocities".
 */
#include"ros/ros.h"
#include"geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include"velocity_planner.h"
#include <math.h>
//#include <iostream>
//#include <fstream>
//#include <iomanip>
class VelocityPlanner {
public:
	VelocityPlanner(std::string topic_originalvel,std::string topic_plannedvel,ros::NodeHandle nh);
	~VelocityPlanner(){
		//	outfile.close();
	};
	double planned_vel[3];
	void ScurveProduce(double []);
private:
	ros::NodeHandle nh_;
	double global_count;
	double Acc_count[3],Dcc_count[3];
	double AccTime,DccTime;
	bool Acc_flag[3],Dcc_flag[3],AccInit_flag[3],DccInit_flag[3];
	double instantaneous_vel[3];
	double vel_org[3];
	double current_coff[3];
	geometry_msgs::Twist objective_vel,objective_vel_old,Scurve_vel;
	std::string topic_plannedvel_;
	std::string topic_originalvel_;
	ros::Publisher pub_plannedvel;
	ros::Subscriber sub_originalvel;
	//std::ofstream outfile;

	void callback_OriginalVel(const geometry_msgs::Twist::Ptr &topic_originalvel);
};

VelocityPlanner::VelocityPlanner(std::string topic_originalvel,std::string topic_plannedvel,ros::NodeHandle nh)
{
	nh_ = nh;

	global_count=0;
	topic_originalvel_=topic_originalvel;
	topic_plannedvel_=topic_plannedvel;
	AccTime=0;
	DccTime=0;
	objective_vel_old.linear.x=0;
	objective_vel_old.linear.y=0;
	objective_vel_old.angular.z=0;
	memset(instantaneous_vel, 0, sizeof(instantaneous_vel));
	memset(vel_org, 0, sizeof(vel_org));
	memset(DccInit_flag, 1, sizeof(DccInit_flag));
	memset(AccInit_flag, 1, sizeof(AccInit_flag));
	memset(Dcc_flag, 0, sizeof(Dcc_flag));
	memset(Acc_flag, 0, sizeof(Acc_flag));
	memset(Acc_count, 0, sizeof(Acc_count));
	memset(Dcc_count, 0, sizeof(Dcc_count));
	memset(current_coff, 1, sizeof(current_coff));
	//	outfile.open("/home/cnimi/data.txt");

	/*if(!outfile)
	{
		std::cout << "Unable to open data.txt"<<std::endl;
		exit(1);
	}
	 */
	sub_originalvel = nh_.subscribe(topic_originalvel_, 10, &VelocityPlanner::callback_OriginalVel, this);
	pub_plannedvel = nh_.advertise<geometry_msgs::Twist>(topic_plannedvel_, 10);
}
void VelocityPlanner::ScurveProduce(double planned_vel[])
{
	int i;
	double Acc_jerk[3],Dcc_jerk[3];
	double t,t1;
	double ATm[3],DTm[3];
	double Scurve_AccTime,Scurve_DccTime;


	Scurve_AccTime=AccTime*pub_velfreq;                       // Here 1 means acc time is 1s
	Scurve_DccTime=DccTime*pub_velfreq;                      // Here 1 means dcc time is 1s


	//ROS_INFO("planned velocity of x:%f",planned_vel[0]);
	//ROS_INFO("planned velocity of y:%f",planned_vel[1]);
	//ROS_INFO("planned  velocity of z:%f",planned_vel[2]);

	for (i=0;i<3;i++)
	{

		if(instantaneous_vel[i]<(planned_vel[i]-absolute))
		{
			if(i==2)
			{
			ATm[i]=XYW_conf*0.5*Scurve_AccTime/2;
			}	
			else
			{
			ATm[i]=0.5*Scurve_AccTime/2;

			}
			Acc_flag[i]=1;
			Dcc_count[i]=0;
			Dcc_flag[i]=0;
			if (AccInit_flag[i]==1)
			{
				vel_org[i]=instantaneous_vel[i];
				AccInit_flag[i]=0;
			}
			Acc_jerk[i]=((planned_vel[i]-vel_org[i])/pow(ATm[i],2));
			t=Acc_count[i];
			//ROS_INFO("deltaT velocity:%f",Acc_jerk);
			if(t<ATm[i])
			{
				t1=(Acc_jerk[i]*pow(t,2))/2;
				instantaneous_vel[i]=vel_org[i]+t1;
			}
			if((t>=ATm[i])&&(t<2*ATm[i]))
			{
				t1=2*Acc_jerk[i]*ATm[i]*t-(Acc_jerk[i]*pow(t,2))/2-Acc_jerk[i]*pow(ATm[i],2);
				instantaneous_vel[i]=vel_org[i]+t1;
			}
			if(t>=2*ATm[i])
			{
				vel_org[i]=planned_vel[i];
				instantaneous_vel[i]=vel_org[i];
			}
			Acc_count[i]=Acc_count[i]+1;
		}
		if(Acc_flag[i]==1)
		{
			if((instantaneous_vel[i]>=(planned_vel[i]-absolute))&&(instantaneous_vel[i]<=(planned_vel[i]+absolute)))
			{
				Acc_count[i]=0;
				Dcc_count[i]=0;
				instantaneous_vel[i]=planned_vel[i];
			}
		}
		/*if((instantaneous_vel[i]>=(planned_vel[i]-absolute))&&(instantaneous_vel[i]<=planned_vel[i]+absolute))
		{
			vel_org[i]=planned_vel[i];
			instantaneous_vel[i]=vel_org[i];
		}*/

		if(instantaneous_vel[i]>planned_vel[i]+absolute)
		{
			//	ROS_INFO("Dcc Process!!");

			if(i==2)
			{
			DTm[i]=XYW_conf*0.5*Scurve_DccTime/2;
			}	
			else
			{
			DTm[i]=0.5*Scurve_DccTime/2;

			}


			Acc_flag[i]=0;
			Dcc_flag[i]=1;
			Acc_count[i]=0;
			if (DccInit_flag[i]==1)
			{
				vel_org[i]=instantaneous_vel[i];
				DccInit_flag[i]=0;
			}
			Dcc_jerk[i]=((vel_org[i]-planned_vel[i])/pow(DTm[i],2));
			t=Dcc_count[i];
			//ROS_INFO("deltaT velocity:%f",deltaT);
			if(t<DTm[i])
			{
				t1=(Dcc_jerk[i]*pow(t,2))/2;
				instantaneous_vel[i]=vel_org[i]-t1;
			}
			if((t>=DTm[i])&&(t<2*DTm[i]))
			{
				t1=(Dcc_jerk[i]*pow(t,2))/2+Dcc_jerk[i]*pow(DTm[i],2)-2*Dcc_jerk[i]*DTm[i]*t;
				instantaneous_vel[i]=vel_org[i]+t1;
			}
			if(t>=2*DTm[i])
			{
				vel_org[i]=planned_vel[i];
				instantaneous_vel[i]=vel_org[i];
			}
			Dcc_count[i]=Dcc_count[i]+1;
		}

		if(Dcc_flag[i]==1)
		{
			if((instantaneous_vel[i]<=(planned_vel[i]+absolute))&&(instantaneous_vel[i]>=(planned_vel[i]-absolute)))
			{
				Acc_count[i]=0;
				Dcc_count[i]=0;
				instantaneous_vel[i]=planned_vel[i];
			}
		}
	}
	global_count=global_count+1;
	//ROS_INFO("instananeous value,%f",Scurve_AccTime);
	//outfile<<global_count<<" "<<instantaneous_vel[0]<<" "<<instantaneous_vel[1]<<" "<< instantaneous_vel[2]<<std::endl;

	/*	ROS_INFO("current_count ACC x,%f",Acc_count[0]);
	ROS_INFO("current_count ACC y,%f",Acc_count[1]);
	ROS_INFO("current_count ACC z,%f",Acc_count[2]);
	ROS_INFO("current_count DCC x,%f",Dcc_count[0]);
	ROS_INFO("current_count DCC y,%f",Dcc_count[1]);
	ROS_INFO("current_count DCC z,%f",Dcc_count[2]);
	ROS_INFO("Published velocity of x:%f",instantaneous_vel[0]);
	ROS_INFO("Published velocity of y:%f",instantaneous_vel[1]);
	ROS_INFO("Published velocity of z:%f",instantaneous_vel[2]);*/
	Scurve_vel.linear.x=instantaneous_vel[0];
	Scurve_vel.linear.y=instantaneous_vel[1];
	//	Scurve_vel.linear.z=0;
	Scurve_vel.angular.x=0;
	Scurve_vel.angular.y=0;
	Scurve_vel.angular.z=instantaneous_vel[2];

	Scurve_vel.linear.z=objective_vel.linear.z;                //transfer velocity

	pub_plannedvel.publish(Scurve_vel);

	//ROS_INFO("Published velocity:%f",instantaneous_vel.linear.x);
	//ROS_INFO("Published velocity: \r  linear: x:%f, y:%f, z:%f,angular: x:%f, y:%f, z:%f",
	//	Scurve_vel.linear.x,Scurve_vel.linear.y,Scurve_vel.linear.z,Scurve_vel.angular.x,Scurve_vel.angular.y,Scurve_vel.angular.z);
}


void VelocityPlanner::callback_OriginalVel(const geometry_msgs::Twist::Ptr &topic_originalvel)
{
	objective_vel.linear.x=topic_originalvel->linear.x;
	objective_vel.linear.y=topic_originalvel->linear.y;
	objective_vel.linear.z=topic_originalvel->linear.z;
	objective_vel.angular.x=topic_originalvel->angular.x;
	objective_vel.angular.y=topic_originalvel->angular.y;
	objective_vel.angular.z=topic_originalvel->angular.z;
	planned_vel[0]=objective_vel.linear.x;
	planned_vel[1]=objective_vel.linear.y;
	planned_vel[2]=objective_vel.angular.z;

	if((objective_vel.linear.x!=objective_vel_old.linear.x))
	{
		DccInit_flag[0]=1;
		AccInit_flag[0]=1;
		//ROS_INFO("Velocity X Upated!");
		objective_vel_old.linear.x=objective_vel.linear.x;
	}
	if((objective_vel.linear.y!=objective_vel_old.linear.y))
	{
		DccInit_flag[1]=1;
		AccInit_flag[1]=1;
		//ROS_INFO("Velocity Y Upated!");
		objective_vel_old.linear.y=objective_vel.linear.y;
	}
	if((objective_vel.angular.z!=objective_vel_old.angular.z))
	{
		DccInit_flag[2]=1;
		AccInit_flag[2]=1;
		//ROS_INFO("Velocity Z Upated!");
		objective_vel_old.angular.z=objective_vel.angular.z;
	}

	if(objective_vel.angular.x<=0)
	{
		AccTime=0.1;
		ROS_INFO("cannot set acceleration time to minus value!!!");
	}
	if(objective_vel.angular.x>0)
	{
		if	(objective_vel.angular.x!=AccT)
		{
			AccTime=objective_vel.angular.x;
		}
		else
		{
			AccTime=AccT;
		}
	}
	if(objective_vel.angular.y<=0)
	{
		DccTime=0.1;
		ROS_INFO("cannot set deceleration time to minus value!!!");
	}
	if(objective_vel.angular.y>0)
	{
		if	(objective_vel.angular.y!=DccT)
		{
			DccTime=objective_vel.angular.y;
		}
		else
		{
			DccTime=DccT;
		}
	}
	//ROS_INFO("old velocity x is %f,new velocity x is %f",objective_vel_old.linear.x,objective_vel.linear.x);
	//ROS_INFO("old velocity y is %f,new velocity y is %f",objective_vel_old.linear.y,objective_vel.linear.y);
	//ROS_INFO("old velocity z is %f,new velocity z is %f",objective_vel_old.angular.z,objective_vel.angular.z);
	//ROS_INFO("Original velocity: \r  linear: x:%f, y:%f, z:%f,angular: x:%f, y:%f, z:%f",
	//topic_originalvel->linear.x,topic_originalvel->linear.y,topic_originalvel->linear.z,topic_originalvel->angular.x,topic_originalvel->angular.y,topic_originalvel->angular.z);
}

int main(int argc, char** argv) {
	double current_time;
	ros::init(argc, argv, "velocity_planner");
	ros::NodeHandle nh;
	VelocityPlanner Robot("/original_vel", "/cmd_tsvel",nh);
	ros::Rate loop_rate(pub_velfreq);
	int count = 0;
	current_time=ros::Time::now().toSec();
	while (ros::ok()) {
		Robot.ScurveProduce(Robot.planned_vel);
		//	ROS_INFO("delta time: %f", ros::Time::now().toSec() - current_time);
		current_time=ros::Time::now().toSec();
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
