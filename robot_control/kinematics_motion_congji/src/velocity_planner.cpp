
/*
 * Velocity Planner V2.0
 * This source file is a velocity plan solution for omni-directional robot.
 *  Updated on: April 9, 2020
 *  Author: Tianjiang zheng
 *  Email: ztj_1@163.com
 *  This file subscribe velocities in task space "original_vel", publish planned velocities based on s-curve.
 *  Subscribe /original_vel, and published  /cmd_tsvel as planned S-curve, published /scurve_params as the acceleration and deceleration parameters );
 */
#include"ros/ros.h"
#include"geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <math.h>
#define FREQ 100
#define X_ACC 0.2   // the maximum acc is better to limit in (100*(double)Circum), that is 23.5619m/s^2
#define X_DCC 0.2
#define Y_ACC 0.2
#define Y_DCC 0.2
#define W_ACC 0.1
#define W_DCC 0.1
//#define ACC_DCC_CONFF 0.6
//#define BREAK_CONFF 0.55
#define ACC_DCC_CONFF 0.05  
#define BREAK_CONFF 0.2
#define E_LIM 0.00001
class VelocityPlanner {
public:
	VelocityPlanner(std::string topic_originalvel,std::string topic_plannedvel,std::string topic_parameters,ros::NodeHandle nh);
	~VelocityPlanner(){};
	void GenerateScurveForXYW();
private:
	ros::NodeHandle nh_;
	geometry_msgs::Twist receivedVelocity,receivedVelocity_old,pubVelocity;
	std::string topic_plannedvel_;
	std::string topic_originalvel_;
	std::string topic_parameters_;
	ros::Publisher pub_plannedvel;
	ros::Subscriber sub_originalvel;
	ros::Subscriber sub_parameters;

	double tempAccXYW[3],tempDccXYW[3],targetTemp[3];
	double recvAccXYW[3],recvDccXYW[3];
	double instantaneousVelocity[3];
	double accTime[3],dccTime[3];
	double originalVelocity[3];
	double accXYW[3],dccXYW[3];
	double targetVelocity[3];
	int breakingFlag[3];
	int accMode[3];
	int accFlag[3];
	int dccFlag[3];
	bool recvdParamsFlag;
	bool reverseFlag[3];
	int buttonsFlag[3];
	int accCount[3];
	int dccCount[3];

	double GetScurveProfile(int m_TimeCount, bool m_Flag, double tar_Velocity,double ori_Velocity,double m_Tm);
	void CallbackOriginalVel(const geometry_msgs::Twist::Ptr &topic_originalvel);
	void CallbackParameters(const geometry_msgs::Twist::Ptr &topic_parameters);

};

VelocityPlanner::VelocityPlanner(std::string topic_originalvel,std::string topic_plannedvel,std::string topic_parameters,ros::NodeHandle nh)
{
	nh_ = nh;
	recvdParamsFlag=false;
	topic_originalvel_=topic_originalvel;
	topic_plannedvel_=topic_plannedvel;
	topic_parameters_=topic_parameters;

	accXYW[0]=X_ACC*ACC_DCC_CONFF+(X_ACC-X_ACC*ACC_DCC_CONFF)*fabs(targetVelocity[0]-originalVelocity[0]);
	accXYW[1]=Y_ACC*ACC_DCC_CONFF+(Y_ACC-Y_ACC*ACC_DCC_CONFF)*fabs(targetVelocity[1]-originalVelocity[1]);
	accXYW[2]=W_ACC*ACC_DCC_CONFF+(W_ACC-W_ACC*ACC_DCC_CONFF)*fabs(targetVelocity[2]-originalVelocity[2]);
	dccXYW[0]=X_DCC*ACC_DCC_CONFF+(X_DCC-X_DCC*ACC_DCC_CONFF)*fabs(targetVelocity[0]-originalVelocity[0]);
	dccXYW[1]=Y_DCC*ACC_DCC_CONFF+(Y_DCC-Y_DCC*ACC_DCC_CONFF)*fabs(targetVelocity[1]-originalVelocity[1]);
	dccXYW[2]=W_DCC*ACC_DCC_CONFF+(W_DCC-W_DCC*ACC_DCC_CONFF)*fabs(targetVelocity[2]-originalVelocity[2]);

	receivedVelocity.linear.x=0;
	receivedVelocity.linear.y=0;
	receivedVelocity.linear.z=0;
	receivedVelocity.angular.x=0;
	receivedVelocity.angular.y=0;
	receivedVelocity.angular.z=0;
	receivedVelocity_old=receivedVelocity;
	pubVelocity=receivedVelocity;

	memset(instantaneousVelocity, 0, sizeof(instantaneousVelocity));
	memset(originalVelocity, 0, sizeof(originalVelocity));
	memset(breakingFlag, 0, sizeof(breakingFlag));
	memset(buttonsFlag, 0, sizeof(buttonsFlag));
	memset(reverseFlag, 0, sizeof(reverseFlag));
	memset(recvAccXYW, 0, sizeof(recvAccXYW));
	memset(recvDccXYW, 0, sizeof(recvDccXYW));
	memset(recvAccXYW, 0, sizeof(recvAccXYW));
	memset(tempDccXYW, 0, sizeof(tempDccXYW));
	memset(targetTemp, 0, sizeof(targetTemp));
	memset(accCount, 0, sizeof(accCount));
	memset(dccCount, 0, sizeof(dccCount));
	memset(accTime, 0, sizeof(accTime));
	memset(dccTime, 0, sizeof(dccTime));
	memset(accFlag, 0, sizeof(accFlag));
	memset(dccFlag, 0, sizeof(dccFlag));
	memset(accMode, 0, sizeof(accMode));

	sub_parameters = nh_.subscribe(topic_parameters_, 10, &VelocityPlanner::CallbackParameters, this);
	sub_originalvel = nh_.subscribe(topic_originalvel_, 10, &VelocityPlanner::CallbackOriginalVel, this);
	pub_plannedvel = nh_.advertise<geometry_msgs::Twist>(topic_plannedvel_, 10);
}

void VelocityPlanner::GenerateScurveForXYW()
{
	int i,j;
	for(j=0;j<3;j++)
	{
		accTime[j]=(accXYW[j]*FREQ)/2;
		dccTime[j]=(dccXYW[j]*FREQ)/2;
	}

	//ROS_INFO("instantaneousVelocity[0]: %f", instantaneousVelocity[0]);
	//ROS_INFO("dccTime[0]: %f", dccTime[0]);

	for (i=0;i<3;i++)
	{
		double temp_time;
		if(instantaneousVelocity[i]<(targetVelocity[i]-E_LIM))
		{
			accFlag[i]=1;
			dccCount[i]=0;
			dccFlag[i]=0;
			if (targetVelocity[i]>0)
				temp_time=accTime[i];
			else
				temp_time=dccTime[i];
			instantaneousVelocity[i]=GetScurveProfile(accCount[i],accFlag[i],targetVelocity[i],originalVelocity[i],temp_time);
			accCount[i]=accCount[i]+1;
		}
		if((instantaneousVelocity[i]>=(targetVelocity[i]-E_LIM))&&(instantaneousVelocity[i]<=(targetVelocity[i]+E_LIM)))
		{
			if ((accFlag[i]==true)||(dccFlag[i]==true))
			{
				accCount[i]=0;
				dccCount[i]=0;
				dccFlag[i]=0;
				accFlag[i]=0;
				instantaneousVelocity[i]=targetVelocity[i];
				originalVelocity[i]=instantaneousVelocity[i];
			}
		}
		if(instantaneousVelocity[i]>targetVelocity[i]+E_LIM)
		{
			accFlag[i]=0;
			accCount[i]=0;
			dccFlag[i]=1;
			if (targetVelocity[i]>=0)
				temp_time=dccTime[i];
			else
				temp_time=accTime[i];

			instantaneousVelocity[i]=GetScurveProfile(dccCount[i],dccFlag[i],targetVelocity[i],originalVelocity[i],temp_time);
			dccCount[i]=dccCount[i]+1;

		}

		if (breakingFlag[i]==1)
		{
			if (fabs(instantaneousVelocity[i])<E_LIM)
			{
				ROS_INFO("Breaking Done,%f",fabs(instantaneousVelocity[i]));
				breakingFlag[i]=0;
				buttonsFlag[i]=0;
				accCount[i]=0;
				dccCount[i]=0;
				targetVelocity[i]=targetTemp[i];
				originalVelocity[i]=instantaneousVelocity[i];
				accXYW[i]=tempAccXYW[i]*ACC_DCC_CONFF+(tempAccXYW[i]-tempAccXYW[i]*ACC_DCC_CONFF)*fabs(targetVelocity[i]-originalVelocity[i]);
				dccXYW[i]=tempDccXYW[i]*ACC_DCC_CONFF+(tempDccXYW[i]-tempDccXYW[i]*ACC_DCC_CONFF)*fabs(targetVelocity[i]-originalVelocity[i]);
			}
		}
	}
	pubVelocity.linear.x=instantaneousVelocity[0];
	pubVelocity.linear.y=instantaneousVelocity[1];
	pubVelocity.linear.z=0;
	pubVelocity.angular.x=0;
	pubVelocity.angular.x=0;
	pubVelocity.angular.z=instantaneousVelocity[2];
	pub_plannedvel.publish(pubVelocity);
}

double VelocityPlanner::GetScurveProfile(int m_TimeCount, bool m_Flag, double tar_Velocity,double ori_Velocity,double m_Tm)
{
	double tempVelocity;
	double m_Jerk;
	if (m_Flag==true)
	{
		m_Jerk=((tar_Velocity-ori_Velocity)/pow(m_Tm,2));
		if(m_TimeCount<m_Tm)
		{
			tempVelocity=ori_Velocity+(m_Jerk*pow(m_TimeCount,2))/2;
		}
		if((m_TimeCount>=m_Tm)&&(m_TimeCount<2*m_Tm))
		{
			tempVelocity=ori_Velocity+2*m_Jerk*m_Tm*m_TimeCount-(m_Jerk*pow(m_TimeCount,2))/2-m_Jerk*pow(m_Tm,2);
		}
		if (m_TimeCount>=2*m_Tm)
		{
			ori_Velocity=tar_Velocity;
			tempVelocity=ori_Velocity;
		}
	}
	if (m_Flag==false)
	{
		m_Jerk=((ori_Velocity-tar_Velocity)/pow(m_Tm,2));
		if(m_TimeCount<m_Tm)
		{
			tempVelocity=ori_Velocity-(m_Jerk*pow(m_TimeCount,2))/2;
		}
		if((m_TimeCount>=m_Tm)&&(m_TimeCount<2*m_Tm))
		{
			tempVelocity=ori_Velocity-2*m_Jerk*m_Tm*m_TimeCount+(m_Jerk*pow(m_TimeCount,2))/2+m_Jerk*pow(m_Tm,2);
		}
		if (m_TimeCount>=2*m_Tm)
		{
			ori_Velocity=tar_Velocity;
			tempVelocity=ori_Velocity;
		}
	}
	//	if ((accInitFlag[0]==true)||(dccInitFlag[0]==true))
	//	{
	//		ROS_INFO("m_Tm: %f", m_Tm);
	//		ROS_INFO("m_Jerk: %f", m_Jerk);
	//		ROS_INFO("ori_Velocity: %f", ori_Velocity);
	//		ROS_INFO("tempVelocity: %f", tempVelocity);
	//		ROS_INFO("instantaneousVelocity[0]: %f", instantaneousVelocity[0]);
	//		ROS_INFO("tar_Velocity: %f", tar_Velocity);
	//		ROS_INFO("m_TimeCount: %d", m_TimeCount);
	//		accInitFlag[0]=0;
	//		dccInitFlag[0]=0;
	//	}
	//	ROS_INFO("tempVelocity: %f", tempVelocity);
	//	ROS_INFO("m_Tm: %f", m_Tm);
	return tempVelocity;
}


void VelocityPlanner::CallbackParameters(const geometry_msgs::Twist::Ptr &topic_parameters)
{

	recvAccXYW[0]=topic_parameters->linear.x;
	recvAccXYW[1]=topic_parameters->linear.y;
	recvAccXYW[2]=topic_parameters->linear.z;
	recvDccXYW[0]=topic_parameters->angular.x;
	recvDccXYW[1]=topic_parameters->angular.y;
	recvDccXYW[2]=topic_parameters->angular.z;
	if (topic_parameters->linear.x<=0.1)
	{	recvAccXYW[0]=0.1;
	ROS_WARN("Cannot set X-direction Acceleration time to minus value, we changed it into 0.1");
	}
	if (topic_parameters->linear.y<=0.1)
	{	recvAccXYW[1]=0.1;
	ROS_WARN("Cannot set Y-direction Acceleration time to minus value, we changed it into 0.1");
	}
	if (topic_parameters->linear.z<=0.1)
	{	recvAccXYW[2]=0.1;
	ROS_WARN("Cannot set W-direction Acceleration time to minus value, we changed it into 0.1");
	}
	if (topic_parameters->angular.x<=0.1)
	{	recvDccXYW[0]=0.1;
	ROS_WARN("Cannot set X-direction Deceleration time to minus value, we changed it into 0.1");
	}
	if (topic_parameters->angular.y<=0.1)
	{	recvDccXYW[1]=0.1;
	ROS_WARN("Cannot set Y-direction Deceleration time to minus value, we changed it into 0.1");
	}
	if (topic_parameters->angular.z<=0.1)
	{	recvDccXYW[2]=0.1;
	ROS_WARN("Cannot set W-direction Deceleration time to minus value, we changed it into 0.1");
	}
	recvdParamsFlag=true;

}

void VelocityPlanner::CallbackOriginalVel(const geometry_msgs::Twist::Ptr &topic_originalvel)
{

	int i;
	receivedVelocity.linear.x=topic_originalvel->linear.x;
	receivedVelocity.linear.y=topic_originalvel->linear.y;
	receivedVelocity.linear.z=topic_originalvel->linear.z;
	receivedVelocity.angular.x=topic_originalvel->angular.x;
	receivedVelocity.angular.y=topic_originalvel->angular.y;
	receivedVelocity.angular.z=topic_originalvel->angular.z;

	targetTemp[0]=topic_originalvel->linear.x;
	targetTemp[1]=topic_originalvel->linear.y;
	targetTemp[2]=topic_originalvel->angular.z;

	if (recvdParamsFlag==true)
	{
		tempAccXYW[0]=recvAccXYW[0];
		tempAccXYW[1]=recvAccXYW[1];
		tempAccXYW[2]=recvAccXYW[2];
		tempDccXYW[0]=recvDccXYW[0];
		tempDccXYW[1]=recvDccXYW[1];
		tempDccXYW[2]=recvDccXYW[2];
	}
	else
	{
		tempAccXYW[0]=X_ACC;
		tempAccXYW[1]=Y_ACC;
		tempAccXYW[2]=W_ACC;
		tempDccXYW[0]=X_DCC;
		tempDccXYW[1]=Y_DCC;
		tempDccXYW[2]=W_DCC;
	}
	for (i=0;i<3;i++)
	{
		if (targetTemp[i]>0)
		{
			buttonsFlag[i]=1;
			if ((targetTemp[i]>=instantaneousVelocity[i])&&(instantaneousVelocity[i]>=0))
				accMode[i]=1;      //accMode=1代表正向加速
			if ((targetTemp[i]>=instantaneousVelocity[i])&&(instantaneousVelocity[i]<0))
			{
				if (breakingFlag[i]==0)
				{
					accMode[i]=2;		//accMode=2代表负向减速
				}
				else
				{
					accMode[i]=0;
				}
			}

			if (targetTemp[i]<instantaneousVelocity[i])
				accMode[i]=3;		//accMode=3代表正向减速
		}
		else if (targetTemp[i]<0)
		{
			buttonsFlag[i]=-1;
			if ((targetTemp[i]<=instantaneousVelocity[i])&&(instantaneousVelocity[i]<=0))
				accMode[i]=4; 		//accMode=4代表负向加速
			if ((targetTemp[i]<=instantaneousVelocity[i])&&(instantaneousVelocity[i]>0))
			{
				if (breakingFlag[i]==0)
				{
					accMode[i]=5;		//accMode=5代表正向减速
				}
				else
				{
					accMode[i]=0;
				}
			}
			if (targetTemp[i]>instantaneousVelocity[i])
				accMode[i]=6;		//accMode=6代表负向减速
		}
	}

	//ROS_INFO("buttonsFlag[0] %d.",buttonsFlag[0]);
	//ROS_INFO("accMode[0] %d.",accMode[0]);
	//ROS_INFO("accFlag[0] %d.",int(accFlag[0]));
	//ROS_INFO("breakingFlag[0] %f.",breakingFlag[0]);
	if((receivedVelocity.linear.x!=receivedVelocity_old.linear.x))
	{
		if (buttonsFlag[0]==0)
		{
			if((accMode[0]==1)||(accMode[0]==4)||(accMode[0]==3)||(accMode[0]==6))
			{
				breakingFlag[0]=0;
				accCount[0]=0;
				dccCount[0]=0;
				targetVelocity[0]=topic_originalvel->linear.x;
				originalVelocity[0]=instantaneousVelocity[0];
				accXYW[0]=tempAccXYW[0]*ACC_DCC_CONFF+(tempAccXYW[0]-tempAccXYW[0]*ACC_DCC_CONFF)*fabs(targetVelocity[0]-originalVelocity[0]);
				dccXYW[0]=tempDccXYW[0]*ACC_DCC_CONFF+(tempDccXYW[0]-tempDccXYW[0]*ACC_DCC_CONFF)*fabs(targetVelocity[0]-originalVelocity[0]);
			}
		}
		else
		{
			if((accMode[0]==1)||(accMode[0]==4)||(accMode[0]==3)||(accMode[0]==6))
			{
				breakingFlag[0]=0;
				accCount[0]=0;
				dccCount[0]=0;
				targetVelocity[0]=topic_originalvel->linear.x;
				originalVelocity[0]=instantaneousVelocity[0];
				accXYW[0]=tempAccXYW[0]*ACC_DCC_CONFF+(tempAccXYW[0]-tempAccXYW[0]*ACC_DCC_CONFF)*fabs(targetVelocity[0]-originalVelocity[0]);
				dccXYW[0]=tempDccXYW[0]*ACC_DCC_CONFF+(tempDccXYW[0]-tempDccXYW[0]*ACC_DCC_CONFF)*fabs(targetVelocity[0]-originalVelocity[0]);

			}
			if ((accMode[0]==5)||(accMode[0]==2))
			{
				ROS_INFO("Breaking!");
				accCount[0]=0;
				dccCount[0]=0;
				targetVelocity[0]=0;
				originalVelocity[0]=instantaneousVelocity[0];
				accXYW[0]=BREAK_CONFF*(tempAccXYW[0]*ACC_DCC_CONFF+(tempAccXYW[0]-tempAccXYW[0]*ACC_DCC_CONFF)*fabs(targetVelocity[0]-originalVelocity[0]));
				dccXYW[0]=BREAK_CONFF*(tempDccXYW[0]*ACC_DCC_CONFF+(tempDccXYW[0]-tempDccXYW[0]*ACC_DCC_CONFF)*fabs(targetVelocity[0]-originalVelocity[0]));
				breakingFlag[0]=1;
			}
		}
		receivedVelocity_old.linear.x=receivedVelocity.linear.x;
	}


	if(receivedVelocity.linear.y!=receivedVelocity_old.linear.y)
	{
		if (buttonsFlag[1]==0)
		{
			if((accMode[1]==1)||(accMode[1]==4)||(accMode[1]==3)||(accMode[1]==6))
			{
				breakingFlag[1]=0;
				accCount[1]=0;
				dccCount[1]=0;
				targetVelocity[1]=topic_originalvel->linear.y;
				originalVelocity[1]=instantaneousVelocity[1];
				accXYW[1]=tempAccXYW[1]*ACC_DCC_CONFF+(tempAccXYW[1]-tempAccXYW[1]*ACC_DCC_CONFF)*fabs(targetVelocity[1]-originalVelocity[1]);
				dccXYW[1]=tempDccXYW[1]*ACC_DCC_CONFF+(tempDccXYW[1]-tempDccXYW[1]*ACC_DCC_CONFF)*fabs(targetVelocity[1]-originalVelocity[1]);
			}
		}
		else
		{
			if((accMode[1]==1)||(accMode[1]==4)||(accMode[1]==3)||(accMode[1]==6))
			{
				breakingFlag[1]=0;
				accCount[1]=0;
				dccCount[1]=0;
				targetVelocity[1]=topic_originalvel->linear.y;
				originalVelocity[1]=instantaneousVelocity[1];
				accXYW[1]=tempAccXYW[1]*ACC_DCC_CONFF+(tempAccXYW[1]-tempAccXYW[1]*ACC_DCC_CONFF)*fabs(targetVelocity[1]-originalVelocity[1]);
				dccXYW[1]=tempDccXYW[1]*ACC_DCC_CONFF+(tempDccXYW[1]-tempDccXYW[1]*ACC_DCC_CONFF)*fabs(targetVelocity[1]-originalVelocity[1]);

			}
			if ((accMode[1]==5)||(accMode[1]==2))
			{
				ROS_INFO("Breaking!");
				accCount[1]=0;
				dccCount[1]=0;
				targetVelocity[1]=0;
				originalVelocity[1]=instantaneousVelocity[1];
				accXYW[1]=BREAK_CONFF*(tempAccXYW[1]*ACC_DCC_CONFF+(tempAccXYW[1]-tempAccXYW[1]*ACC_DCC_CONFF)*fabs(targetVelocity[1]-originalVelocity[1]));
				dccXYW[1]=BREAK_CONFF*(tempDccXYW[1]*ACC_DCC_CONFF+(tempDccXYW[1]-tempDccXYW[1]*ACC_DCC_CONFF)*fabs(targetVelocity[1]-originalVelocity[1]));
				breakingFlag[1]=1;
			}
		}

		receivedVelocity_old.linear.y=receivedVelocity.linear.y;
	}
	if(receivedVelocity.angular.z!=receivedVelocity_old.angular.z)
	{
		if (buttonsFlag[2]==0)
		{
			if((accMode[2]==1)||(accMode[2]==4)||(accMode[2]==3)||(accMode[2]==6))
			{
				breakingFlag[2]=0;
				accCount[2]=0;
				dccCount[2]=0;
				targetVelocity[2]=topic_originalvel->angular.z;
				originalVelocity[2]=instantaneousVelocity[2];
				accXYW[2]=tempAccXYW[2]*ACC_DCC_CONFF+(tempAccXYW[2]-tempAccXYW[2]*ACC_DCC_CONFF)*fabs(targetVelocity[2]-originalVelocity[2]);
				dccXYW[2]=tempDccXYW[2]*ACC_DCC_CONFF+(tempDccXYW[2]-tempDccXYW[2]*ACC_DCC_CONFF)*fabs(targetVelocity[2]-originalVelocity[2]);
			}
		}
		else
		{
			if((accMode[2]==1)||(accMode[2]==4)||(accMode[2]==3)||(accMode[2]==6))
			{
				breakingFlag[2]=0;
				accCount[2]=0;
				dccCount[2]=0;
				targetVelocity[2]=topic_originalvel->angular.z;
				originalVelocity[2]=instantaneousVelocity[2];
				accXYW[2]=tempAccXYW[2]*ACC_DCC_CONFF+(tempAccXYW[2]-tempAccXYW[2]*ACC_DCC_CONFF)*fabs(targetVelocity[2]-originalVelocity[2]);
				dccXYW[2]=tempDccXYW[2]*ACC_DCC_CONFF+(tempDccXYW[2]-tempDccXYW[2]*ACC_DCC_CONFF)*fabs(targetVelocity[2]-originalVelocity[2]);

			}
			if ((accMode[2]==5)||(accMode[2]==2))
			{
				ROS_INFO("Breaking!");
				accCount[2]=0;
				dccCount[2]=0;
				targetVelocity[2]=0;
				originalVelocity[2]=instantaneousVelocity[2];
				accXYW[2]=BREAK_CONFF*(tempAccXYW[2]*ACC_DCC_CONFF+(tempAccXYW[2]-tempAccXYW[2]*ACC_DCC_CONFF)*fabs(targetVelocity[2]-originalVelocity[2]));
				dccXYW[2]=BREAK_CONFF*(tempDccXYW[2]*ACC_DCC_CONFF+(tempDccXYW[2]-tempDccXYW[2]*ACC_DCC_CONFF)*fabs(targetVelocity[2]-originalVelocity[2]));
				breakingFlag[2]=1;
			}
		}
		receivedVelocity_old.angular.z=receivedVelocity.angular.z;
	}



	/*for (i=0;i<3;i++)
	{
		if (instantaneousVelocity[i]*targetVelocity[i]<0)
		{
			reverseFlag[i]=1;
			accXYW[i]=0.5*(tempAccXYW[i]*ACC_DCC_CONFF+(tempAccXYW[i]-tempAccXYW[i]*ACC_DCC_CONFF)*fabs(targetVelocity[i]-originalVelocity[i]));
			dccXYW[i]=0.5*(tempDccXYW[i]*ACC_DCC_CONFF+(tempDccXYW[i]-tempDccXYW[i]*ACC_DCC_CONFF)*fabs(targetVelocity[i]-originalVelocity[i]));
		}
		accXYW[i]=tempAccXYW[i]*ACC_DCC_CONFF+(tempAccXYW[i]-tempAccXYW[i]*ACC_DCC_CONFF)*fabs(targetVelocity[i]-originalVelocity[i]);
		dccXYW[i]=tempDccXYW[i]*ACC_DCC_CONFF+(tempDccXYW[i]-tempDccXYW[i]*ACC_DCC_CONFF)*fabs(targetVelocity[i]-originalVelocity[i]);

	}*/




}

int main(int argc, char** argv) {
	double current_time;
	ros::init(argc, argv, "VelocityPlanner");
	ros::NodeHandle nh;
	VelocityPlanner Robot("/original_vel", "/cmd_tsvel","/vel_params",nh);
	ros::Rate loop_rate(FREQ);
	double actFrequency;
	current_time=ros::Time::now().toSec();
	while (ros::ok()) {
		Robot.GenerateScurveForXYW();
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
