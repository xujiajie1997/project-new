#!/usr/bin/env python
import rospy
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from numpy import*
from pylab import*
from matplotlib import*
import time
from std_msgs.msg import Int16MultiArray
import tf

H = 0.41379
B = 0.0625
R = 0.0625
beta=0.8110
fq=50

class fwd_kinematics:
	def __init__(self):
		self.command="start"
		self.dfai1=0
		self.dfai2=0
		self.dfai3=0
		self.dfai4=0

		self.drow1=0
		self.drow2=0
		self.drow3=0
		self.drow4=0

		self.fai1=0
		self.fai2=0
		self.fai3=0
		self.fai4=0
		
		self.velocity_flag=0
		self.degree_flag=0

		self.tpos_x=0
		self.tpos_y=0
		self.tpos_theta=0
		self.ptask_x=0
		self.ptask_y=0
		self.ptask_theta=0
		self.gblpos=Twist()
		self.odom_all_out=Odometry()
		self.odom_broadcaster=tf.TransformBroadcaster()
#		self.vel_out=Twist()
		rospy.init_node('forward_kinematics',anonymous=True)
		self.start_time=rospy.get_time()
		self.pub_odompos=rospy.Publisher('/odom_pos',Twist,queue_size=10)
		self.pub_odom_all=rospy.Publisher('/odom_all',Odometry,queue_size=10)

#		self.pub_vel=rospy.Publisher('/odom_vel',Twist,queue_size=10)
		rospy.Subscriber('/data_encoder',Float64MultiArray,self.callback_encoderdata)
		rospy.Subscriber('/data_vel',Float64MultiArray,self.callback_veldata)
		rospy.Subscriber('/pos_zero',String,self.callback_string)
		rate=rospy.Rate(fq)	
		while not rospy.is_shutdown():
			
			dfai1=self.dfai1
			dfai2=self.dfai2
			dfai3=self.dfai3
			dfai4=self.dfai4
			drow1=self.drow1
			drow2=self.drow2
			drow3=self.drow3
			drow4=self.drow4

			fai1=self.fai1
			fai2=self.fai2
			fai3=self.fai3
			fai4=self.fai4
			[dlt_x,dlt_y,dlt_theta]=self.velocitycalculate(dfai1,drow1,dfai2,drow2,dfai3,drow3,dfai4,drow4,fai1,fai2,fai3,fai4)
			if self.command=="setzero":
				print "position data set to zero"
				pos_x=0
				pos_y=0
				pos_theta=0
				self.tpos_x=0
				self.tpos_y=0
				self.ptask_theta=0
				self.command="start"
			[pos_x,pos_y,pos_theta]=self.posecalculate(dlt_x,dlt_y,dlt_theta)
			
			current_time=rospy.Time.now()
			odom_quat=tf.transformations.quaternion_from_euler(0,0,pos_theta)
			self.odom_broadcaster.sendTransform((pos_x,pos_y,0),odom_quat,current_time,"base_link","odom")
			
			self.odom_all_out.header.stamp=rospy.Time.now()
			self.odom_all_out.header.frame_id="odom"
			self.odom_all_out.child_frame_id="base_footprint"

			self.odom_all_out.twist.twist.linear.x=-dlt_x
			self.odom_all_out.twist.twist.linear.y=-dlt_y
			self.odom_all_out.twist.twist.linear.z=0
			self.odom_all_out.twist.twist.angular.x=0
			self.odom_all_out.twist.twist.angular.y=0
			self.odom_all_out.twist.twist.angular.z=dlt_theta
			
			self.odom_all_out.pose.pose.position.x=pos_x
			self.odom_all_out.pose.pose.position.y=pos_y
			self.odom_all_out.pose.pose.position.z=0	
			aa=tf.transformations.quaternion_from_euler(0,0,pos_theta)
			#print "aa",aa[0]
			self.odom_all_out.pose.pose.orientation.x=aa[0]
			self.odom_all_out.pose.pose.orientation.y=aa[1]
			self.odom_all_out.pose.pose.orientation.z=aa[2]
			self.odom_all_out.pose.pose.orientation.w=aa[3]

			self.odom_all_out.twist.covariance[0] = 0.5*0.5
			self.odom_all_out.twist.covariance[7] = 0.5*0.5
			self.odom_all_out.twist.covariance[14] = 0.0
			self.odom_all_out.twist.covariance[21] = 0.0
			self.odom_all_out.twist.covariance[28] = 0.0
			self.odom_all_out.twist.covariance[35] = 0.314*0.314
			self.pub_odom_all.publish(self.odom_all_out)

			#print "Jointspace velocity %f,%f,%f,%f",(fai1,row1,fai2,row2)
			#print "odometry is %f,%f,%f",(pos_x,pos_y,pos_theta)
			#print "Taskspace velocity %f,%f,%f",(dlt_x,dlt_y,dlt_theta)
			
			self.gblpos.linear.x=pos_x
			self.gblpos.linear.y=pos_y
			self.gblpos.angular.z=pos_theta		
			#print "Basespace Pos:",(pos_x,pos_y,pos_theta)	
			self.pub_odompos.publish(self.gblpos)
			#plt.ion()
			#plt.plot(pos_x,pos_y,'*')
			#plt.draw()
			rate.sleep()
			
	//偶数滚，奇数转
	def callback_veldata(self,data):
		#self.velocity_flag=data.data[8]	
		#if(self.velocity_flag==1):
			self.drow1=data.data[0]
			self.drow2=data.data[2]
			self.drow3=data.data[4]
			self.drow4=data.data[6]
			self.dfai1=data.data[1]
			self.dfai2=data.data[3]
			self.dfai3=data.data[5]
			self.dfai4=data.data[7]
	def callback_string(self,data):
		self.command=data.data


	def callback_encoderdata(self,data):
		#self.degree_flag=data.data[8]
		#if(self.degree_flag==1):
			self.fai1=data.data[1]
			self.fai2=data.data[3]
			self.fai3=data.data[5]
			self.fai4=data.data[7]
	
	def velocitycalculate(self,dfai1,drow1,dfai2,drow2,dfai3,drow3,dfai4,drow4,fai1,fai2,fai3,fai4):
        	M_A1=array([[-sin(fai1),cos(fai1),H*cos(fai1+beta)-B],[cos(fai1), sin(fai1),H*sin(fai1+beta)],[-sin(fai2),cos(fai2),-H*cos(fai2-beta)-B],[cos(fai2),sin(fai2),-H*sin(fai2-beta)]])
		M_A2=array([[-sin(fai3),cos(fai3),-H*cos(fai3+beta)-B],[cos(fai3),sin(fai3),-H*sin(fai3+beta)],[-sin(fai4),cos(fai4), H*cos(fai4-beta)-B],[cos(fai4),sin(fai4),H*sin(fai4-beta)]])
		M_A=row_stack((M_A1,M_A2))
		q_dot=array([B*dfai1,R*drow1,B*dfai2,R*drow2,B*dfai3,R*drow3,B*dfai4,R*drow4])
		M=dot(M_A.T,M_A)
		n=dot(M_A.T,q_dot)
		dlt_P=linalg.solve(M,n)
		#print "FWD_NODE"
		return dlt_P

	def posecalculate(self,dlt_x,dlt_y,dlt_theta):
	
		dlt_time=rospy.get_time()-self.start_time
		Pos_task=array([dlt_x,dlt_y,dlt_theta])
		
		P_task=dlt_time*Pos_task
		self.ptask_x=P_task[0]
		self.ptask_y=P_task[1]
		self.ptask_theta =P_task[2]
#		print "P_theta",P_task[2]		
#		print "P_task_x",self.ptask_x
#		print "P_task_y",self.ptask_y
#		print "P_theta_int",self.ptask_theta

		
		M_D=([[cos(self.tpos_theta),-sin(self.tpos_theta),0],[sin(self.tpos_theta),cos(self.tpos_theta),0],[0,0,1]])
		p_task1=array([self.ptask_x,self.ptask_y,self.ptask_theta])
		[lpos_x,lpos_y,lpos_theta]=dot(M_D,p_task1)

		self.tpos_x += lpos_x
		self.tpos_y += lpos_y
		self.tpos_theta += lpos_theta

		pos_p = [self.tpos_x,self.tpos_y,self.tpos_theta]
#		print "Time consuming: " ,dlt_time
		self.start_time=rospy.get_time()
		return pos_p

if __name__=='__main__':
	try:
		
		fwd_kinematics()
		
	except rospy.ROSInterruptException: pass
