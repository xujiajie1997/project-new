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
		self.pub_tsvelocity = rospy.Publisher('/task_vel',Float64MultiArray,queue_size=10)   
 		self.pub_encoderdata = rospy.Publisher('/data_encoder',Float64MultiArray,queue_size=10)
		self.pub_veldata = rospy.Publisher('/data_vel',Float64MultiArray,queue_size=10)
		emuc.EMUCSetCanOn(gl_com_port,self.baudrate_can1,self.baudrate_can2)
		# ----- start emuc tx/rx thread -----

		while not rospy.is_shutdown():
			temp_time = rospy.get_time()
			self.cnt_1 = self.cnt_1 +1                                                    
			if self.cnt_1>512:
				self.cnt_1=0

			self.sendingdata=[0x2b,0x0b,0x20,0x02,0xff,0x00,0x00,0x00]
			self.send_group(0X605,self.sendingdata)
			self.sendingdata=[0x2b,0x0b,0x20,0x02,0xff,0x00,0x00,0x00]
			self.send_group1(0X605,self.sendingdata)
			

			for i in range(0,8):
				if i%2==0:
					self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
					self.send_group1(self.motorID2[i],self.sendingdata)
					self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
					self.send_group1(self.motorID2[i],self.sendingdata) 
					self.sendingdata=[0x23,0xff,0x60,0x00,0xb8,0x0b,0x00,0x00]			
					self.send_group1(self.motorID2[i],self.sendingdata)
					self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata)
					self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata) 
					self.sendingdata=[0x23,0xff,0x60,0x00,0xb8,0x0b,0x00,0x00]			
					self.send_group(self.motorID2[i],self.sendingdata)
				else:
					self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
					self.send_group1(self.motorID2[i],self.sendingdata)
					self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
					self.send_group1(self.motorID2[i],self.sendingdata) 
					self.sendingdata=[0x23,0xff,0x60,0x00,0xd0,0x07,0x00,0x00]			
					self.send_group1(self.motorID2[i],self.sendingdata)
					self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata)
					self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
					self.send_group(self.motorID2[i],self.sendingdata) 
					self.sendingdata=[0x23,0xff,0x60,0x00,0xd0,0x07,0x00,0x00]			
					self.send_group(self.motorID2[i],self.sendingdata)


			'''for i in range(1,8,2):
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
				self.send_group1(self.motorID2[i],self.sendingdata)
				self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
				self.send_group1(self.motorID2[i],self.sendingdata) 
				self.sendingdata=[0x23,0xff,0x60,0x00,0xd0,0x07,0x00,0x00]			
				self.send_group1(self.motorID2[i],self.sendingdata)

			for i in range(1,8,2):
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
				self.send_group(self.motorID2[i],self.sendingdata)
				self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
				self.send_group(self.motorID2[i],self.sendingdata) 
				self.sendingdata=[0x23,0xff,0x60,0x00,0xd0,0x07,0x00,0x00]			
				self.send_group(self.motorID2[i],self.sendingdata)

			for i in range(0,8,2):
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
				self.send_group1(self.motorID2[i],self.sendingdata)
				self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
				self.send_group1(self.motorID2[i],self.sendingdata) 
				self.sendingdata=[0x23,0xff,0x60,0x00,0xb8,0x0b,0x00,0x00]			
				self.send_group1(self.motorID2[i],self.sendingdata)

			for i in range(0,8,2):
				self.sendingdata=[0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00]
				self.send_group(self.motorID2[i],self.sendingdata)
				self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
				self.send_group(self.motorID2[i],self.sendingdata) 
				self.sendingdata=[0x23,0xff,0x60,0x00,0xb8,0x0b,0x00,0x00]			
				self.send_group(self.motorID2[i],self.sendingdata)

			self.sendingdata=[0x2f,0x60,0x60,0x00,0x01,0x00,0x00,0x00]
			self.send_group1(0x604,self.sendingdata)
	 		self.sendingdata=[0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00]
			self.send_group1(0x604,self.sendingdata)  
			self.sendingdata=[0x23,0x81,0x60,0x00,0x2a,0x00,0x00,0x00]
			self.send_group1(0x604,self.sendingdata) 
	 		self.sendingdata=[0x23,0x7a,0x60,0x00,0x60,0xe3,0x16,0x00]
			self.send_group1(0x604,self.sendingdata)
			self.sendingdata=[0x2b,0x40,0x60,0x00,0x5f,0x00,0x00,0x00]
			self.send_group1(0x604,self.sendingdata) 8-2  2-1'''


	def send_group1(self,canid,data):		
		frame_send_ptr = [CAN_FRAME_INFO(),]
        # ----- setup variable ----
		frame_send_ptr[0].CAN_port = CAN_PORT.EMUC_CAN_2
		frame_send_ptr[0].id_type = ID_TYPE.EMUC_SID
		frame_send_ptr[0].rtr = RTR.EMUC_DIS_RTR
		frame_send_ptr[0].dlc = 8
		frame_send_ptr[0].id = canid 
		for i in range(0,8):
 			frame_send_ptr[0].data[i] = data[i]
		emuc.EMUCSend(gl_com_port, frame_send_ptr[0])


	def send_group(self,canid,data):		
		frame_send_ptr = [CAN_FRAME_INFO(),]
        # ----- setup variable ----
		frame_send_ptr[0].CAN_port = CAN_PORT.EMUC_CAN_1
		frame_send_ptr[0].id_type = ID_TYPE.EMUC_SID
		frame_send_ptr[0].rtr = RTR.EMUC_DIS_RTR
		frame_send_ptr[0].dlc = 8
		frame_send_ptr[0].id = canid 
		for i in range(0,8):
 			frame_send_ptr[0].data[i] = data[i]
		emuc.EMUCSend(gl_com_port, frame_send_ptr[0])

if __name__ == '__main__':
	try:
		MotorMoveCANBUS()
	except rospy.ROSInterruptException:
       	# ----- EMUCCloseDevice() -----
		emuc.EMUCCloseDevice(gl_com_port)
        pass






	







