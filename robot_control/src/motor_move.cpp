/*
 * motor_move.cpp
 * This code used for two wheeled active decouppled caster robot.
 * The driver of galil motion control card need to be installed first.
 *  Updated on: Mar 4, 2016
 *  Author: Tianjiang Zheng
 *  Email: ztj_1@163.com
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "parameters_mobilemanipulator.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <iostream> //std::cout
#include <string> //to_string, string, etc.
#include <cstdio> //sprintf, etc.
#include <cstring> //strlen, etc.
#include <unistd.h>
#include "gclib.h"
#include "gclibo.h"
#include <sstream>
#include <iomanip>

using namespace std;
GCon g = 0;
string temp;
char* cmd;

inline void x_e(GReturn rc)
{
	if (rc != G_NO_ERROR) 
	{
		throw rc;
	}

}



class MotorMove {
public:
	MotorMove(std::string topic_jointvel,std::string topic_taskspacevel,std::string topic_extvelcommand,std::string topic_encoderdata,std::string topic_iodata,std::string topic_velocitydata,ros::NodeHandle nh);
	~MotorMove()
	{
		try{
			x_e(GCmd(g, "ST"));
			x_e(GCmd(g, "MO"));
			ROS_INFO("Stopped");
			GClose(g);
		}
		catch (int e) {
			std::cout << e;
		}
	}
	void Statemachine(void);


private:
	ros::NodeHandle nh_;
	double Encoderdata[8];
	double Veldata[8];
	double Joint_space_vel[8];
	double temp_taskvel[3];
	bool status_left, status_right,status_up,status_down, Homingdone,Homingflag,Emergencybutton,Emergencyflag,Btprevious;
	bool diginput_bit[16];
	geometry_msgs::Twist Taskvel_fromext;
	int ControlMode;
	bool Fangzhuang_flag;
	int set_count;

	char inputread_io[100];
	char inputread_encoder[100];
	char inputread_vel[100];
	char tell_error[100];

	std::string topic_bluetoothjoy_;
	std::string topic_jointvel_;
	std::string topic_taskspacevel_;
	std::string topic_extvelcommand_;
	std::string topic_encoderdata_;
	std::string topic_iodata_;
	std::string topic_veldata_;

	ros::Subscriber sub_bluetoothjoy;
	ros::Subscriber sub_externalvelcommand;
	ros::Subscriber sub_inversekinematics;

	ros::Publisher pub_tsvelocity;
	ros::Publisher pub_encoderdata;
	ros::Publisher pub_iodata;
	ros::Publisher pub_veldata;

	void DoHoming(void);
	void JointSpaceOut(void);
	void ParaseInput(void);
	void ParaseEncoder(void);
	void EmergencyStop(void);
	void ParaseVel(void);
	void DataProcessOut(void);
	void callback_extcommand(const geometry_msgs::Twist::Ptr &topic_extvelcommand);
	void calback_invkin(const sensor_msgs::JointState::Ptr &topic_jointvel);

};

MotorMove::MotorMove(std::string topic_jointvel,std::string topic_taskspacevel,std::string topic_extvelcommand,std::string topic_encoderdata,std::string topic_iodata, std::string topic_velocitydata,ros::NodeHandle nh)
{
	nh_ = nh;
	status_left = true;
	status_right = true;
	status_up = true;
	status_down = true;
	set_count=0;
	Homingdone = 0;
	Homingflag = 1;
	Emergencybutton=0;
	Emergencyflag=1;
	inputread_encoder[0]='0';
	inputread_io[0]='0';
	tell_error[0]='0';
	inputread_vel[0]='0';
	ControlMode=0;
	Btprevious=0;
	memset(Joint_space_vel, 0, sizeof(Joint_space_vel));
	memset(Encoderdata, 0, sizeof(Encoderdata));
	memset(Veldata, 0, sizeof(Veldata));
	memset(diginput_bit, 0, sizeof(diginput_bit));
	memset(temp_taskvel, 0, sizeof(temp_taskvel));
	topic_jointvel_=topic_jointvel;
	topic_taskspacevel_=topic_taskspacevel;
	topic_extvelcommand_=topic_extvelcommand;
	topic_encoderdata_=topic_encoderdata;
	topic_iodata_=topic_iodata;
	topic_veldata_=topic_velocitydata;
	sub_externalvelcommand = nh_.subscribe(topic_extvelcommand, 10, &MotorMove::callback_extcommand, this);
	sub_inversekinematics = nh_.subscribe(topic_jointvel_, 10, &MotorMove::calback_invkin, this);
	pub_tsvelocity = nh_.advertise<std_msgs::Float64MultiArray>(topic_taskspacevel_, 10);
	pub_encoderdata = nh_.advertise<std_msgs::Float64MultiArray>(topic_encoderdata_, 10);
	pub_iodata = nh_.advertise<std_msgs::Int32MultiArray>(topic_iodata_, 10);
	pub_veldata = nh_.advertise<std_msgs::Float64MultiArray>(topic_veldata_, 10);
	Fangzhuang_flag=0;

}

void MotorMove::DoHoming() {
	try {
		if (Homingflag == 1) {
			x_e(GCmd(g, "ST"));
			x_e(GCmd(g, "MO"));
			x_e(GCmd(g, "DP 0,0,0,0,0,0,0,0"));
			//x_e(GCmd(g, "PR 0,400000,0,400000,0,400000,0,400000"));
			x_e(GCmd(g, "PR 170000,200000,170000,200000,170000,200000,170000,200000"));
			x_e(GCmd(g, "SP 0,10000,0,10000,0,10000,0,10000"));
			//x_e(GCmd(g, "SP 0,50000,0,50000,0,50000,0,50000"));			
			x_e(GCmd(g, "AC 512000,512000,512000,512000,512000,512000,512000,512000"));	
			x_e(GCmd(g, "DC 512000,512000,512000,512000,512000,512000,512000,512000"));
			x_e(GCmd(g, "KP 10, 10, 10, 10, 10, 10, 10 ,10"));
			//x_e(GCmd(g, "KP 10, 25, 10, 25, 10, 25, 10 ,25"));
			x_e(GCmd(g, "KI 0.5,0.25,0.5,0.25,0.5,0.25,0.5,0.25"));			
			//x_e(GCmd(g, "KD 120,100,120,100,120,100,120,100"));	
			x_e(GCmd(g, "KD 120,95,120,95,120,95,120,95"));
			Homingflag = 0;
			x_e(GCmd(g, "SH BDFH"));				
			x_e(GCmd(g, "BG BDFH"));

		}
		if ((status_left == true) && (diginput_bit[7] == 0)) {
			x_e(GCmd(g, "STB"));
			usleep(50);
			x_e(GCmd(g, "DPB=0"));
			x_e(GCmd(g, "PRB=0"));
			x_e(GCmd(g, "SPB=0"));
			//g.command("STB");			
			ROS_INFO("left wheel homing done");
			status_left = false;
			//printf("%d\n",
			//cout<<status_left<<endl;
		}

		if ((status_right == true) && (diginput_bit[1] == 0)) {
			x_e(GCmd(g, "STD"));
			usleep(50);
			x_e(GCmd(g, "DPD=0"));
			x_e(GCmd(g, "PRD=0"));
			x_e(GCmd(g, "SPD=0"));
			//g.command("STD");		
			ROS_INFO("right wheel homing done");
			status_right = false;
			//printf("%d\n",
			//cout<<status_right<<endl;

		}

		if ((status_up == true) && (diginput_bit[3] == 0)) {
			x_e(GCmd(g, "STF"));
			x_e(GCmd(g, "DPF=0"));
			x_e(GCmd(g, "PRF=0"));
			x_e(GCmd(g, "SPF=0"));
			usleep(50);
			//x_e(GCmd(g, "DPH=0"));
			ROS_INFO("up wheel homing done");
			status_up = false;

		}
		if ((status_down == true) && (diginput_bit[5] == 0)) {
			x_e(GCmd(g, "STH"));
			usleep(50);
			x_e(GCmd(g, "DPH=0"));
			x_e(GCmd(g, "DPH=0"));
			x_e(GCmd(g, "PRH=0"));
			x_e(GCmd(g, "SPH=0"));

			ROS_INFO("down wheel homing done");
			status_down = false;
		}
		if ((status_right == false) && (status_left == false) && (status_up == false)&&(status_down == false)) {
			x_e(GCmd(g, "ST"));			
			x_e(GCmd(g, "SH"));
			//x_e(GCmd(g, "AC 1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000"));
			//x_e(GCmd(g, "DC 1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000"));
			x_e(GCmd(g, "AC 512000,512000,512000,512000,512000,512000,512000,512000"));
			x_e(GCmd(g, "DC 512000,512000,512000,512000,512000,512000,512000,512000"));
			x_e(GCmd(g, "DP 0,0,0,0,0,0,0,0"));
			x_e(GCmd(g, "JG 0,0,0,0,0,0,0,0"));
			usleep(50);
			x_e(GCmd(g, "BG A,B,C,D,E,F,G,H"));
			Homingdone = true;
			ROS_INFO("All wheels homing done");
		}
	} catch (int e) {
		std::cout << e;
	}
}
void MotorMove::JointSpaceOut() {
	stringstream s_cmd;
	set_count+=1;
	int temp_const[8];
	int i,j;
	GSize bytes;
	temp_const[0]=const_motor_A;
	temp_const[1]=const_motor_B;
	temp_const[2]=const_motor_C;
	temp_const[3]=const_motor_D;
	temp_const[5]=const_motor_E;
	temp_const[4]=const_motor_F;
	temp_const[7]=const_motor_G;
	temp_const[6]=const_motor_H;
	for (i=0;i<8;i++)
	{
		if (Joint_space_vel[i] > Max_vel_turns*temp_const[i])
			Joint_space_vel[i] = Max_vel_turns*temp_const[i];
		if (Joint_space_vel[i] < -Max_vel_turns*temp_const[i])
			Joint_space_vel[i] = -Max_vel_turns*temp_const[i];
	}
	for (j = 0; j < 8; j++) {
		if (abs(Joint_space_vel[j]) < 1)
			Joint_space_vel[j] = 0;
	}


	s_cmd << "JG " << Joint_space_vel[0] << "," << Joint_space_vel[1]  << ","<<  Joint_space_vel[2] << "," <<Joint_space_vel[3] <<"," 
			<< Joint_space_vel[4]<< "," << Joint_space_vel[5]<< ","<< Joint_space_vel[6]<< "," << Joint_space_vel[7];
	
	temp=s_cmd.str();

	cmd=(char*)temp.data();
	try{
		x_e(GCmd(g, cmd));
	}
	catch (int e){
		std::cout<<e;
	}
	if (set_count>10)
	{
		x_e(GCommand(g,"TE",tell_error,sizeof(tell_error),&bytes));
		cout<<"The following errors are:"<<tell_error;
		set_count=0;
	}

}

void MotorMove::ParaseInput() {
	int inputdata, temp,temp_t,i;
	GSize bytes;
	try{
		x_e(GCommand(g,"TI0",inputread_io,sizeof(inputread_io),&bytes));		
	}
	catch (int e){
		std::cout<<e;
	}
	inputdata = std::atoi(inputread_io);
	temp = inputdata & 0x00FF;

	for (i=0;i<8;i++)
	{
		temp_t=0x01<<(i);
		if ((temp & temp_t) ==temp_t) {      diginput_bit[i] = 1;  }
		if ((temp & temp_t) ==0) {      diginput_bit[i] = 0;  }
	}
	try{
		x_e(GCommand(g,"TI1",inputread_io,sizeof(inputread_io),&bytes));		
	}
	catch (int e){
		std::cout<<e;
	}
	inputdata = std::atoi(inputread_io);
	temp = inputdata & 0x00FF;

	for (i=0;i<8;i++)
	{
		temp_t=0x01<<(i);
		if ((temp & temp_t) ==temp_t) {      diginput_bit[i+8] = 1;  }
		if ((temp & temp_t) ==0) {      diginput_bit[i+8] = 0;  }
	}
	if(diginput_bit[4]==0)   //0 is represent the button pressed
	{
		if(Emergencyflag==1)
		{
			//Emergencybutton=1;
			//ControlMode=2;
		}
	}
	if(diginput_bit[4]==1)
	{
		//Emergencyflag=1;
		//Emergencybutton=0;
	}
	if(diginput_bit[5]==0)
	{
		//Fangzhuang_flag=1;

	}
	if(diginput_bit[5]==1)
	{
		//Fangzhuang_flag=0;
	}
	std_msgs::Int32MultiArray temp_io;
	temp_io.data.resize(16);
	for(i=0;i<16;i++)
	{
		temp_io.data[i]=diginput_bit[i];
	}
	pub_iodata.publish(temp_io);
}



void MotorMove::ParaseEncoder() {
	double temp;
	char i;
	char *value,*next;
	const char * split = ",";
	char *p[8];
	GSize bytes;
	try{
		x_e(GCommand(g,"TP",inputread_encoder,sizeof(inputread_encoder),&bytes));
	}
	catch (int e){
		std::cout<<e;
	}
	value=inputread_encoder;
	for (i=0;i<8;i++)
	{
		p[i] = strsep(&value,split);
		Encoderdata[i]=std::atof(p[i]);
		next=value;
	}
	Encoderdata[0] = 2 * M_PI * Encoderdata[0] / const_motor_A;
	Encoderdata[1] = 2 * M_PI * Encoderdata[1] / const_motor_B;
	Encoderdata[2] = 2 * M_PI * Encoderdata[2] / const_motor_C;
	Encoderdata[3] = 2 * M_PI * Encoderdata[3] / const_motor_D;
	Encoderdata[4] = 2 * M_PI * Encoderdata[4] / const_motor_E;
	Encoderdata[5] = 2 * M_PI * Encoderdata[5] / const_motor_F;
	Encoderdata[6] = 2 * M_PI * Encoderdata[6] / const_motor_G;
	Encoderdata[7] = 2 * M_PI * Encoderdata[7] / const_motor_H;
	std_msgs::Float64MultiArray temp_en;
	temp_en.data.resize(8);
	int temp_k;
	for (i=0;i<8;i++)
	{
		temp_en.data[i]=Encoderdata[i];
	}
	pub_encoderdata.publish(temp_en);
}

void MotorMove::DataProcessOut(void)
{
	double beta=0.8300;

	ParaseEncoder();
	ParaseVel();
	//ROS_INFO("this thread start");
	std_msgs::Float64MultiArray taskvel_parameters;
	taskvel_parameters.data.resize(8);
	taskvel_parameters.data[0]=temp_taskvel[0];
	taskvel_parameters.data[1]=temp_taskvel[1];
	taskvel_parameters.data[2]=temp_taskvel[2];
	taskvel_parameters.data[3]=beta;
	taskvel_parameters.data[4]=Encoderdata[1];
	taskvel_parameters.data[5]=Encoderdata[3];
	taskvel_parameters.data[6]=Encoderdata[5];
	taskvel_parameters.data[7]=Encoderdata[7];
	pub_tsvelocity.publish(taskvel_parameters);

}


void MotorMove::ParaseVel() {
	double temp;
	char i;
	char *value,*next;
	const char * split = ",";
	char *p[8];
	GSize bytes;
	try{
		x_e(GCommand(g,"TV",inputread_vel,sizeof(inputread_vel),&bytes));
	}
	catch (int e){
		std::cout<<e;
	}
	value=inputread_vel;
	for (i=0;i<8;i++)
	{
		p[i] = strsep(&value,split);
		Veldata[i]=std::atof(p[i]);
		next=value;
	}
	Veldata[0] = 2 * M_PI * Veldata[0] / const_motor_A;
	Veldata[1] = 2 * M_PI * Veldata[1] / const_motor_B;
	Veldata[2] = 2 * M_PI * Veldata[2] / const_motor_C;
	Veldata[3] = 2 * M_PI * Veldata[3] / const_motor_D;
	Veldata[4] = 2 * M_PI * Veldata[4] / const_motor_E;
	Veldata[5] = 2 * M_PI * Veldata[5] / const_motor_F;
	Veldata[6] = 2 * M_PI * Veldata[6] / const_motor_G;
	Veldata[7] = 2 * M_PI * Veldata[7] / const_motor_H;
	std_msgs::Float64MultiArray temp_vel;
	temp_vel.data.resize(8);
	for (i=0;i<8;i++)
	{
		temp_vel.data[i]=Veldata[i];
	}
	pub_veldata.publish(temp_vel);
}


void MotorMove::EmergencyStop()
{
	if ((Emergencyflag==1)&&(Emergencybutton==1))
	{
		try{
			x_e(GCmd(g,"ST"));
			usleep(Delay_stop_time);     //10ms
			x_e(GCmd(g,"DP 0,0,0,0,0,0,0,0"));
			x_e(GCmd(g,"MO"));
		}
		catch (int e){
			std::cout<<e;
		}
		Emergencyflag=0;
	}
	if ((Emergencyflag==0)&&(Emergencybutton==1))
	{
		ROS_INFO("SYSTEM EMERGENCY STOPPED!!!");
		ROS_INFO("Waiting for reset system!!!");
	}
}

void MotorMove::callback_extcommand(const geometry_msgs::Twist::Ptr &topic_extvelcommand)
{
	double beta=0;
	if (Fangzhuang_flag==1)
	{
		ROS_INFO("Fangzhuang is startting");
		temp_taskvel[0]=0;
		temp_taskvel[1]=0;
		temp_taskvel[2]=0;
	}
	else{
		temp_taskvel[0]=max_velcoff*topic_extvelcommand->linear.x;
		temp_taskvel[1]=max_velcoff*topic_extvelcommand->linear.y;
		temp_taskvel[2]=max_velcoff*topic_extvelcommand->angular.z;
	}

	if(topic_extvelcommand->linear.z==1)   //0 is represent the button pressed
	{
		ROS_INFO("Homing Process!!!!");
		ControlMode=0;
		Homingdone=false;
		Homingflag = 1;
		status_left = true;
		status_right = true;
		status_up = true;
		status_down = true;
	}
}

void MotorMove::calback_invkin(const sensor_msgs::JointState::Ptr &topic_jointvel)
{

	Joint_space_vel[0] = topic_jointvel->velocity[0] * const_motor_A;
	Joint_space_vel[1] = topic_jointvel->velocity[1] * const_motor_B;
	Joint_space_vel[2] = topic_jointvel->velocity[2] * const_motor_C;
	Joint_space_vel[3] = topic_jointvel->velocity[3] * const_motor_D;
	Joint_space_vel[4] = topic_jointvel->velocity[4] * const_motor_E;
	Joint_space_vel[5] = topic_jointvel->velocity[5] * const_motor_F;
	Joint_space_vel[6] = topic_jointvel->velocity[6] * const_motor_G;
	Joint_space_vel[7] = topic_jointvel->velocity[7] * const_motor_H;

}

void MotorMove::Statemachine()
{

	switch (ControlMode) {
	case 0:
	{
		if (Homingdone==false)
		{
			ParaseInput();     //get IO input value
			ROS_INFO("Do Homing Start!!!!");
			DoHoming();
			ROS_INFO("ControlMode 0");
		}
		if (Homingdone==true)
		{
			ControlMode=1;
		}
		break;
	}
	case 1:
	{
		MotorMove::DataProcessOut();
		JointSpaceOut();
		break;
	}

	case 2:
	{
		EmergencyStop();
		break;
	}
	case 3:
	{
		break;
	}

	default: break;

	}

}




int main(int argc, char* argv[]) {
	int rc = G_NO_ERROR;
	double current_time;
	ros::init(argc, argv, "Rbt_control_system");
	ros::NodeHandle nh;
	x_e(GOpen("169.254.11.33 -d", &g));
	MotorMove Robot("/joint_vel", "/task_vel","/cmd_tsvel","/data_encoder","/data_io","/data_vel",nh);
	ros::Rate loop_rate(500);
	int count = 0;
	current_time=ros::Time::now().toSec();
	ROS_INFO("Waiting for homing.");
	while (ros::ok()) {
		Robot.Statemachine();
		//ROS_INFO("delta time: %f", 1/(ros::Time::now().toSec() - current_time));
		current_time=ros::Time::now().toSec();
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}

