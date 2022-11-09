#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import rospy
import thread
import binascii
import numpy as np
import math
from matplotlib import pylab
from pylab import *
from lib_emuc_b202 import *
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
M_DEBUG = 0

READ_FREQ = 200

gl_com_port=24

emuc = EMUCClass()

class MotorMoveCANBUS:
	def __init__(self):
		rospy.init_node('MotormoveCANBUS', anonymous=True) 
		#self.cfg_file_name = './emuc_config'
		self.baudrate_can1 = BAUDRATE.EMUC_BAUDRATE_1M
		self.baudrate_can2 = BAUDRATE.EMUC_BAUDRATE_1M
		self.runFlag = 1
		self.sendDataHigh = 0
		self.sendDataLow = 0
		self.setZeroPos = 0
		self.cnt_1 = 0.0
		self.cnt_2 = 0
		self.kai = 0
		self.stopping = [0,0,0,0,0,0,0,0]
		self.starting = 0
		self.velocity=0.0
		self.beta=0.8110
		self.shuju1=-1
		self.shuju2=1
		self.shuju3=-1
		self.shuju4=1
		self.motorJointState = JointState()
		self.sendThread_oldTime = rospy.get_time()
		self.receiveThread_oldTime = rospy.get_time()
		self.const_motor=[1020,1200,1020,1200,1020,1200,1020,1200]		#线速度转换为转速的转换系数
		self.const_motor1=[170000,200000,170000,200000,170000,200000,170000,200000]		#编码器处理系数（由总减速比得）
		self.sendingdata=[0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
		self.readingdata=[0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]			
		self.motorID1 = [1409,1410, 1411, 1412, 1413, 1414, 1415, 1416]				#接受读取光电开关的i/o数字输入地址信息
		self.motorID2 = [0x601, 0x602, 0x603, 0x604, 0x605, 0x606, 0x607, 0x608]		#八个节点的发送地址
		self.motorID_node_mum = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]		#开启402映射的命令需要的输入地址信息，等于节点数
		self.motorID_node_receive = [385,386,387,388,389,390,391,392]				#接受402映射的八个驱动器地址信息
		self.Encoderdata = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]					#处理后的编码器数据
		self.Encoderdata1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]					#原始编码器数据
		self.temp_taskvel = [0,0,0]
		self.taskvel_parameters = Float64MultiArray()						#声明taskvel_parameters数据类型
		self.temp_en = Float64MultiArray()
		self.temp_vel = Float64MultiArray()		
		self.temp_en.data=[0,0,0,0,0,0,0,0]
		self.temp_vel.data=[0,0,0,0,0,0,0,0]
		self.Joint_space_vel = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.taskvel_parameters.data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.pub_motorParameters = rospy.Publisher('/MotorState', JointState, queue_size=10)
		self.pub_motor = rospy.Publisher('/motorparam', Float64MultiArray, queue_size=10)
		rospy.Subscriber('/cmd_tsvel',Twist,self.callback_veldata)
		rospy.Subscriber('/joint_vel',JointState,self.callback_joint)
		self.pub_tsvelocity = rospy.Publisher('/task_vel',Float64MultiArray,queue_size=10)   
 		self.pub_encoderdata = rospy.Publisher('/data_encoder',Float64MultiArray,queue_size=10)
		self.pub_veldata = rospy.Publisher('/data_vel',Float64MultiArray,queue_size=10)
		emuc.EMUCSetCanOn(gl_com_port,self.baudrate_can1,self.baudrate_can2)
		# ----- start emuc tx/rx thread -----
		if (self.runFlag == 1):      #总线程
			try:
				thread.start_new_thread(self.ReadMotorData, ())     #归位读取光电开关，i/o数字输入读取
				thread.start_new_thread(self.controlvel, ())        #控制八个驱动器，传输速度信号
				thread.start_new_thread(self.dealreading, ())		#读取驱动器自动发送的编码器信息，0，1，2，3，4，6
				thread.start_new_thread(self.dealreading1, ())		#读取驱动器自动发送的编码器信息，5，7
				print "Creating receive thread." 
			except:
				print "Error:Creating received thread failed" 
		#rt = rospy.Rate(500)
		if self.starting == 0:
			self.weizhi()		#四个转向电机转两圈，位置环

		self.sendingdata=[0x2b,0x0b,0x20,0x02,0xff,0x00,0x00,0x00]
		self.send_group(0X605,self.sendingdata)

		while not rospy.is_shutdown():
			temp_time = rospy.get_time()
			self.cnt_1 = self.cnt_1 +1                                                    
			if self.cnt_1>512:
				self.cnt_1=0
			self.velocity=self.cnt_1
			self.set_s16(self.velocity)
			if self.starting == 0:
				self.bianbie()		#辨别光电开关，判断是否到达初始位置，并且控制驱动器停止转动，且归入速度环，速度为0
			if self.starting == 1:
				self.DataProcessOut()		#发送（手柄节点接受的速度信号，转向电机的编码器信号）到逆运动学节点
				if self.kai == 0:
					self.kaiqi()		#开启驱动器402映射（自动返回读取的编码器信息）
		#rt.sleep()
	def DataProcessOut(self):
		#while (self.runFlag):
		#if self.starting == 1:
		self.taskvel_parameters.data[0]=self.temp_taskvel[0]
		self.taskvel_parameters.data[1]=self.temp_taskvel[1]
		self.taskvel_parameters.data[2]=self.temp_taskvel[2]
		self.taskvel_parameters.data[3]=self.beta
		self.taskvel_parameters.data[4]=self.Encoderdata[1]
		self.taskvel_parameters.data[5]=self.Encoderdata[3]
		self.taskvel_parameters.data[6]=self.Encoderdata[5]
		self.taskvel_parameters.data[7]=self.Encoderdata[7]
		self.pub_tsvelocity.publish(self.taskvel_parameters)

		#接受手柄节点返回的移动机器人的x，y，z的速度信息
	def callback_veldata(self,data):
		'''self.shuju1 = data.linear.x
		self.shuju3 = data.linear.y
		if ((self.shuju1 * self.shuju2 < 0) or (self.shuju3 * self.shuju4 < 0)):
			self.shuju2 = self.shuju1
			self.shuju4 = self.shuju3
			for i in range(0,8,2):
				self.Joint_space_vel[i] = 102'''
		self.temp_taskvel[0] = data.linear.x
		self.temp_taskvel[1] = data.linear.y
		self.temp_taskvel[2] = data.angular.z
	def send_group(self,canid,data):     #发送数据到can2口
		frame_send_ptr = [CAN_FRAME_INFO(),]
        # ----- setup variable -----
		frame_send_ptr[0].CAN_port = CAN_PORT.EMUC_CAN_2
		frame_send_ptr[0].id_type = ID_TYPE.EMUC_SID
		frame_send_ptr[0].rtr = RTR.EMUC_DIS_RTR
		frame_send_ptr[0].dlc = 8
		frame_send_ptr[0].id = canid 
		for i in range(0,8):
 			frame_send_ptr[0].data[i] = data[i]
		emuc.EMUCSend(gl_com_port, frame_send_ptr[0])


	def send_group1(self,canid,data):	 #发送数据到can1口	
		frame_send_ptr = [CAN_FRAME_INFO(),]
        # ----- setup variable -----
		frame_send_ptr[0].CAN_port = CAN_PORT.EMUC_CAN_1
		frame_send_ptr[0].id_type = ID_TYPE.EMUC_SID
		frame_send_ptr[0].rtr = RTR.EMUC_DIS_RTR
		frame_send_ptr[0].dlc = 8
		frame_send_ptr[0].id = canid 
		for i in range(0,8):
 			frame_send_ptr[0].data[i] = data[i]
		emuc.EMUCSend(gl_com_port, frame_send_ptr[0])


	def print_data(frame_info):
		print "data received"
	def ReadMotorData(self):
		while (self.runFlag):
			#time.sleep(1 / 2000)
			frame_recv_ptr = [CAN_FRAME_INFO(),]
			self.rtn = emuc.EMUCReceive(gl_com_port, frame_recv_ptr[0])
			if self.starting == 0:
				for i in range(1,8,2):		#判断四个转向电机是否到达归位零点位置
					if (frame_recv_ptr[0].id == self.motorID1[i] and frame_recv_ptr[0].data[4] == 62 and self.starting == 0):
						self.stopping[i] = 1
					if (frame_recv_ptr[0].id == self.motorID1[1] and (frame_recv_ptr[0].data[4] == 6 or frame_recv_ptr[0].data[4] == 54) and self.starting == 0):
						self.stopping[1] = 1
			if(self.rtn == 1):
				if(frame_recv_ptr[0].msg_type == MSG_TYPE.EMUC_DATA_TYPE):
					self.i=1
				elif(frame_recv_ptr[0].msg_type == MSG_TYPE.EMUC_EEERR_TYPE): 
					print 'EEPROM Error !'  
				elif(frame_recv_ptr[0].msg_type == MSG_TYPE.EMUC_BUSERR_TYPE):
					print 'Bus Error !'   
					print 'Bus status (CAN 1): '  
					for j in range(1, DATA_LEN_ERR):
						print '%02X ' % frame_recv_ptr[0].data_err[CAN_PORT.EMUC_CAN_2][j]
					print 'Bus status (CAN 2): ' 
					for j in range(0, DATA_LEN_ERR): 
						print '\n' 
		print "Receive thread exit\n"


	def dealreading(self):
		while (self.runFlag):
			time.sleep(1 / 500)
			if self.starting == 1:
				frame_recv_ptr = [CAN_FRAME_INFO(),]
				emuc.EMUCReceive(gl_com_port, frame_recv_ptr[0])
				if ((frame_recv_ptr[0].id == 386)):
					self.Encoderdata1[1] = float(self.set_s32(frame_recv_ptr[0].data[3],frame_recv_ptr[0].data[2],frame_recv_ptr[0].data[1],frame_recv_ptr[0].data[0]))
				if ((frame_recv_ptr[0].id == 388)):
					self.Encoderdata1[3] = float(self.set_s32(frame_recv_ptr[0].data[3],frame_recv_ptr[0].data[2],frame_recv_ptr[0].data[1],frame_recv_ptr[0].data[0]))	
				self.dealEncoderdata()
				#print self.Encoderdata,"bianma1"

	def dealreading1(self):
		while (self.runFlag):
			if self.starting == 1:
				frame_recv_ptr = [CAN_FRAME_INFO(),]
				emuc.EMUCReceive(gl_com_port, frame_recv_ptr[0])
				if (frame_recv_ptr[0].id == 392):
					self.Encoderdata1[7] = float(self.set_s32(frame_recv_ptr[0].data[3],frame_recv_ptr[0].data[2],frame_recv_ptr[0].data[1],frame_recv_ptr[0].data[0]))
					#print "qihao"
				if (frame_recv_ptr[0].id == 390):
					self.Encoderdata1[5] = float(self.set_s32(frame_recv_ptr[0].data[3],frame_recv_ptr[0].data[2],frame_recv_ptr[0].data[1],frame_recv_ptr[0].data[0]))
				self.dealEncoderdata()
				#print self.Encoderdata,"bianma1"



	def set_s16(self, val):
		temp = val
		if temp >= 0:	#一个十六位转两个八位信息
			self.sendDataHigh= ((int(temp) & 0xff00) >> 8)
			self.sendDataLow = (int(temp) & 0x00ff)
		if temp < 0:
			temp_1 = (int(temp) + 0x10000)	
			self.sendDataHigh = ((int(temp_1) & 0xff00) >> 8)
			self.sendDataLow = (int(temp_1) & 0x00ff)

	def set_s32(self,data1,data2,data3,data4):     #四个八位信息转为一个三十二位信息  
		val= ((data1 & 0x000000ff) << 24) | ((data2 & 0x000000ff) << 16) | ((data3 & 0x000000ff) << 8) | (data4 & 0x000000ff)	
		if (val<0x80000000):
			return val
		else:
			return val-0x100000000



	def set_s4_8(self,data):		#一个32位转为四个八位信息
		temp = data
		if temp >= 0:	
			self.data1= ((int(temp) & 0xff000000) >> 24)
			self.data2= ((int(temp) & 0x00ff0000) >> 16)
			self.data3= ((int(temp) & 0x0000ff00) >> 8)
			self.data4 = (int(temp) & 0x000000ff)
		if temp < 0:
			temp_1 = (int(temp) + 0x100000000)	
			self.data1= ((int(temp_1) & 0xff000000) >> 24)
			self.data2= ((int(temp_1) & 0x00ff0000) >> 16)
			self.data3= ((int(temp_1) & 0x0000ff00) >> 8)
			self.data4 = (int(temp_1) & 0x000000ff)	


	def dealEncoderdata(self):		#处理接收到的编码器原始数据，并且发送到编码器节点
		for i in range(0,8):
			self.Encoderdata[i] = 2 * pi * self.Encoderdata1[i] / self.const_motor1[i]	
		for i in range(0,8):
			self.temp_en.data[i] = self.Encoderdata[i]
		self.pub_encoderdata.publish(self.temp_en)

	def callback_joint(self,data):			#运动学回调函数，接受八个驱动器的控制速度
		for i in range(0,8):
			if (i%2==0):
				self.Joint_space_vel[i] = data.velocity[i] * self.const_motor[i]
				#print self.Joint_space_vel[i]
			else:
				self.Joint_space_vel[i] = -1*data.velocity[i] * self.const_motor[i]
				#print self.Joint_space_vel[i]

		for i in range(0,8):			
			if self.Joint_space_vel[i] > 2950:
				self.Joint_space_vel[i] = 2950
			if self.Joint_space_vel[i] < -2950:
				self.Joint_space_vel[i] = -2950
		for i in range(0,8):
			if (( self.Joint_space_vel[i] < 10 ) and ( self.Joint_space_vel[i] > -10 )):
				self.Joint_space_vel[i] = 0
		#print self.Joint_space_vel,"zuizhong"
	def weizhi(self):
		if self.starting == 0:
			for i in range(0,8):
				if ((i== 5 )or(i== 7)):
					continue
				time.sleep(0.05)
				if i%2 == 0:
					self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata)
	 				self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata) 
					self.sendingdata=[0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata)	
				else:
					self.sendingdata=[0x2f,0x60,0x60,0x00,0x01,0x00,0x00,0x00]			#设置位置环
					self.send_group(self.motorID2[i],self.sendingdata)
	 				self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata)  
					self.sendingdata=[0x23,0x81,0x60,0x00,0x2a,0x00,0x00,0x00]			#设置位置环速度
					self.send_group(self.motorID2[i],self.sendingdata) 
	 				self.sendingdata=[0x23,0x7a,0x60,0x00,0x60,0xe3,0x16,0x00]			#设置位置环转动位置
					self.send_group(self.motorID2[i],self.sendingdata)
					self.sendingdata=[0x2b,0x40,0x60,0x00,0x5f,0x00,0x00,0x00]			#设置位置环为相对位置控制
					self.send_group(self.motorID2[i],self.sendingdata) 
			self.sendingdata=[0x2f,0x60,0x60,0x00,0x01,0x00,0x00,0x00]
			self.send_group1(0x606,self.sendingdata)
	 		self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
			self.send_group1(0x606,self.sendingdata)  
			self.sendingdata=[0x23,0x81,0x60,0x00,0x2a,0x00,0x00,0x00]
			self.send_group1(0x606,self.sendingdata) 
	 		self.sendingdata=[0x23,0x7a,0x60,0x00,0x60,0xe3,0x16,0x00]
			self.send_group1(0x606,self.sendingdata)
			self.sendingdata=[0x2b,0x40,0x60,0x00,0x5f,0x00,0x00,0x00]
			self.send_group1(0x606,self.sendingdata) 

			self.sendingdata=[0x2f,0x60,0x60,0x00,0x01,0x00,0x00,0x00]
			self.send_group1(0x608,self.sendingdata)
	 		self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
			self.send_group1(0x608,self.sendingdata)  
			self.sendingdata=[0x23,0x81,0x60,0x00,0x2a,0x00,0x00,0x00]
			self.send_group1(0x608,self.sendingdata) 
	 		self.sendingdata=[0x23,0x7a,0x60,0x00,0x60,0xe3,0x16,0x00]
			self.send_group1(0x608,self.sendingdata)
			self.sendingdata=[0x2b,0x40,0x60,0x00,0x5f,0x00,0x00,0x00]
			self.send_group1(0x608,self.sendingdata) 		




	def kaiqi(self):
		for i in range(0,2):
			self.sendingdata=[0x01,self.motorID_node_mum[1],0x00,0x00,0x00,0x00,0x00,0x00]
			self.send_group(0x00,self.sendingdata)
			self.sendingdata=[0x01,self.motorID_node_mum[3],0x00,0x00,0x00,0x00,0x00,0x00]
			self.send_group(0x00,self.sendingdata)
			self.sendingdata=[0x01,self.motorID_node_mum[5],0x00,0x00,0x00,0x00,0x00,0x00]
			self.send_group1(0x00,self.sendingdata) 
			self.sendingdata=[0x01,self.motorID_node_mum[7],0x00,0x00,0x00,0x00,0x00,0x00]
			self.send_group1(0x00,self.sendingdata) 
			self.sendingdata=[0x01,self.motorID_node_mum[7],0x00,0x00,0x00,0x00,0x00,0x00]
			self.send_group1(0x00,self.sendingdata) 
			self.sendingdata=[0x01,self.motorID_node_mum[7],0x00,0x00,0x00,0x00,0x00,0x00]
			self.send_group1(0x00,self.sendingdata) 
		self.kai = 1
				
	def bianbie(self):
		for i in range(0,6):
			self.sendingdata=[0x40,0x0b,0x20,0x01,0x00,0x00,0x00,0x00]		#读取四个转向电机的光电开关信息，多读几次
			self.send_group1(0x608,self.sendingdata)
			self.sendingdata=[0x40,0x0b,0x20,0x01,0x00,0x00,0x00,0x00]
			self.send_group(0x602,self.sendingdata)
			self.sendingdata=[0x40,0x0b,0x20,0x01,0x00,0x00,0x00,0x00]
			self.send_group(0x604,self.sendingdata)
			self.sendingdata=[0x40,0x0b,0x20,0x01,0x00,0x00,0x00,0x00]
			self.send_group1(0x606,self.sendingdata)
			time.sleep(0.05)
		for i in range(1,8,2):
			if ((i== 5 )or(i== 7)):
				continue
			if self.stopping[i] == 1:
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]		#转为速度环
				self.send_group(self.motorID2[i],self.sendingdata)
				self.sendingdata=[0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00]		#设置速度为零
				self.send_group(self.motorID2[i],self.sendingdata)
				for j in range(0,2):	
					self.sendingdata=[0x2B,0x04,0x20,0x20,0x03,0x00,0x00,0x00]			#编码器位置反馈归零
					self.send_group(self.motorID2[i],self.sendingdata)
					self.sendingdata=[0x2B,0x04,0x20,0x20,0x00,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata)
					#print "homing"
		if self.stopping[5] == 1:
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
				self.send_group1(self.motorID2[5],self.sendingdata)
				self.sendingdata=[0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00]
				self.send_group1(self.motorID2[5],self.sendingdata)
				for j in range(0,2):	
					self.sendingdata=[0x2B,0x04,0x20,0x20,0x03,0x00,0x00,0x00]
					self.send_group1(self.motorID2[5],self.sendingdata)
					self.sendingdata=[0x2B,0x04,0x20,0x20,0x00,0x00,0x00,0x00]
					self.send_group1(self.motorID2[5],self.sendingdata)

		if self.stopping[7] == 1:
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
				self.send_group1(self.motorID2[7],self.sendingdata)
				self.sendingdata=[0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00]
				self.send_group1(self.motorID2[7],self.sendingdata)
				for j in range(0,2):	
					self.sendingdata=[0x2B,0x04,0x20,0x20,0x03,0x00,0x00,0x00]
					self.send_group1(self.motorID2[7],self.sendingdata)
					self.sendingdata=[0x2B,0x04,0x20,0x20,0x00,0x00,0x00,0x00]
					self.send_group1(self.motorID2[7],self.sendingdata)

		for i in range(0,8,2):
			for j in range(0,2):	
				self.sendingdata=[0x2B,0x04,0x20,0x20,0x03,0x00,0x00,0x00]
				self.send_group(self.motorID2[i],self.sendingdata)
				self.sendingdata=[0x2B,0x04,0x20,0x20,0x00,0x00,0x00,0x00]
				self.send_group(self.motorID2[i],self.sendingdata)
		if self.stopping == [0,1,0,1,0,1,0,1]:
			print "starting successing"
			self.stopping = [0,0,0,0,0,0,0,0]	#复位停止系数归零
			self.starting = 1			#控制系数设为1，进入手柄控制模式，为0设置为复位模式

	def controlvel(self):
		while (self.runFlag):
			'''for i in range(0,3):
				print self.temp_taskvel[i]'''
			if self.starting == 1:	
				#for i in range(0,8):
					#print self.Joint_space_vel[i]
				#time.sleep(0.01)
				for i in range(1,8,2):			#设置滚动电机
					self.set_s4_8(self.Joint_space_vel[i])
					self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
					self.send_group(self.motorID2[i-1],self.sendingdata)
					self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]	#开使能
					self.send_group(self.motorID2[i-1],self.sendingdata) 
					self.sendingdata=[0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00]	#发送电机速度信号		
					self.sendingdata[4] =  self.data4
					self.sendingdata[5] =  self.data3
					self.sendingdata[6] =  self.data2
					self.sendingdata[7] =  self.data1
					self.send_group(self.motorID2[i-1],self.sendingdata)
				for i in range(0,8,2):			#设置转向电机
		                  	if(( i ==4)or( i== 6)):
						continue
					self.set_s4_8(self.Joint_space_vel[i])
					self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
					self.send_group(self.motorID2[i+1],self.sendingdata)
					self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
					self.send_group(self.motorID2[i+1],self.sendingdata) 
					self.sendingdata=[0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00]			
					self.sendingdata[4] =  self.data4
					self.sendingdata[5] =  self.data3
					self.sendingdata[6] =  self.data2
					self.sendingdata[7] =  self.data1
					self.send_group(self.motorID2[i+1],self.sendingdata)


				self.set_s4_8(self.Joint_space_vel[4])
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
				self.send_group1(self.motorID2[5],self.sendingdata)
				self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
				self.send_group1(self.motorID2[5],self.sendingdata) 
				self.sendingdata=[0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00]			
				self.sendingdata[4] =  self.data4
				self.sendingdata[5] =  self.data3
				self.sendingdata[6] =  self.data2
				self.sendingdata[7] =  self.data1
				self.send_group1(self.motorID2[5],self.sendingdata)


				self.set_s4_8(self.Joint_space_vel[6])
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
				self.send_group1(self.motorID2[7],self.sendingdata)
				self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
				self.send_group1(self.motorID2[7],self.sendingdata) 
				self.sendingdata=[0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00]			
				self.sendingdata[4] =  self.data4
				self.sendingdata[5] =  self.data3
				self.sendingdata[6] =  self.data2
				self.sendingdata[7] =  self.data1
				self.send_group1(self.motorID2[7],self.sendingdata)


if __name__ == '__main__':
	try:
		MotorMoveCANBUS()
	except rospy.ROSInterruptException:
       	# ----- EMUCCloseDevice() -----
		emuc.EMUCCloseDevice(gl_com_port)
        pass






	







