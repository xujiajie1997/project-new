import os
import platform
import enum
import ctypes
import time
from ctypes import *

if 'posix' in os.name: MAX_COM_NUM = 68
else: MAX_COM_NUM = 256

VER_LEN = 16
CAN_NUM = 2
DATA_LEN = 8
DATA_LEN_ERR = 6
TIME_CHAR_NUM = 13

# ----------------------------------------------------------------------------------------------
class VER_INFO(Structure):
    _fields_ = [
        ('fw', c_char * VER_LEN),
        ('api', c_char * VER_LEN),
        ('model', c_char * VER_LEN),
    ]

# ----------------------------------------------------------------------------------------------
class FILTER_INFO(Structure):
    _fields_ = [
        ('CAN_port', c_int),
        ('flt_type', c_int),
        ('flt_id', c_uint),
        ('mask', c_uint),
    ]

# ----------------------------------------------------------------------------------------------
class CFG_INFO(Structure):
    _fields_ = [
        ('baud', c_ubyte * CAN_NUM),
        ('mode', c_ubyte * CAN_NUM),
        ('flt_type', c_ubyte * CAN_NUM),
        ('flt_id', c_uint * CAN_NUM),
        ('flt_mask', c_uint * CAN_NUM),
        ('err_set', c_ubyte),
    ]

# ----------------------------------------------------------------------------------------------
class CAN_FRAME_INFO(Structure):
    _fields_ = [
        ('CAN_port', c_int),
        ('id_type', c_int),
        ('rtr', c_int),
        ('dlc', c_int),
        ('msg_type', c_int),
        ('recv_time', c_char * TIME_CHAR_NUM),  # e.g., 15:30:58:789 (h:m:s:ms)
        ('id', c_uint),
        ('data', c_ubyte * DATA_LEN),
        ('data_err', c_ubyte * CAN_NUM * DATA_LEN_ERR),
    ]

# ----------------------------------------------------------------------------------------------
class NON_BLOCK_INFO(Structure):
    _fields_ = [
        ('cnt', c_uint),
        ('interval', c_uint),   # [ms]
        ('can_frame_info', POINTER(CAN_FRAME_INFO)),
    ]

    def __init__(self, _cnt, _interval):
        self.cnt = _cnt
        self.interval = _interval
        self.can_frame_info = (CAN_FRAME_INFO * self.cnt)()

# ----------------------------------------------------------------------------------------------
class CAN_PORT(enum.IntEnum):
    EMUC_CAN_1 = 0
    EMUC_CAN_2 = 1

# ----------------------------------------------------------------------------------------------
class BAUDRATE(enum.IntEnum):
    EMUC_BAUDRATE_100K = 4
    EMUC_BAUDRATE_125K = 5
    EMUC_BAUDRATE_250K = 6
    EMUC_BAUDRATE_500K = 7
    EMUC_BAUDRATE_800K = 8
    EMUC_BAUDRATE_1M = 9
    EMUC_BAUDRATE_400K = 10

# ----------------------------------------------------------------------------------------------
class STATUS(enum.IntEnum):
    EMUC_INACTIVE = 0
    EMUC_ACTIVE = 1

# ----------------------------------------------------------------------------------------------
class MODE(enum.IntEnum):
    EMUC_NORMAL = 0
    EMUC_LISTEN = 1

# ----------------------------------------------------------------------------------------------
class ID_TYPE(enum.IntEnum):
    EMUC_SID = 1
    EMUC_EID = 2

# ----------------------------------------------------------------------------------------------
class RTR(enum.IntEnum):
    EMUC_DIS_RTR = 0
    EMUC_EN_RTR = 1

# ----------------------------------------------------------------------------------------------
class ERR_TYPE(enum.IntEnum):
    EMUC_DIS_ALL = 0
    EMUC_EE_ERR = 1
    EMUC_BUS_ERR = 2
    EMUC_EN_ALL = 255

# ----------------------------------------------------------------------------------------------
class MSG_TYPE(enum.IntEnum):
    EMUC_DATA_TYPE = 0
    EMUC_EEERR_TYPE = 1
    EMUC_BUSERR_TYPE = 2
    EMUC_GETBUS_TYPE = 3

# ----------------------------------------------------------------------------------------------
class EMUCClass():
    def __init__(self):
        lib_emuc2_name = None
        if 'posix' in os.name:
            if '32bit' in platform.architecture()[0]: lib_emuc2_name = '/home/robot/CanMotionControl/emucCAN/python_sample/EMUC_Sample_python/lib_emuc2_32.so'
            else: lib_emuc2_name = '/home/robot/CanMotionControl/emucCAN/python_sample/EMUC_Sample_python/lib_emuc2_64.so'
        else: lib_emuc2_name = '/home/robot/CanMotionControl/emucCAN/python_sample/EMUC_Sample_python/lib_emuc2_32.dll'

        self.lib_emuc2_C = ctypes.CDLL(lib_emuc2_name)

        self.lib_emuc2_C.EMUCOpenDevice.argtypes = (c_int,)
        self.lib_emuc2_C.EMUCOpenDevice.restype = c_int

        self.lib_emuc2_C.EMUCCloseDevice.argtypes = (c_int,)
        self.lib_emuc2_C.EMUCCloseDevice.restype = c_int

        self.lib_emuc2_C.EMUCShowVer.argtypes = (c_int, POINTER(VER_INFO))
        self.lib_emuc2_C.EMUCShowVer.restype = c_int

        self.lib_emuc2_C.EMUCResetCAN.argtypes = (c_int,)
        self.lib_emuc2_C.EMUCResetCAN.restype = c_int

        self.lib_emuc2_C.EMUCClearFilter.argtypes = (c_int, c_int)
        self.lib_emuc2_C.EMUCClearFilter.restype = c_int

        self.lib_emuc2_C.EMUCInitCAN.argtypes = (c_int, c_int, c_int)
        self.lib_emuc2_C.EMUCInitCAN.restype = c_int

        self.lib_emuc2_C.EMUCSetBaudRate.argtypes = (c_int, c_int, c_int)
        self.lib_emuc2_C.EMUCSetBaudRate.restype = c_int

        self.lib_emuc2_C.EMUCSetMode.argtypes = (c_int, c_int, c_int)
        self.lib_emuc2_C.EMUCSetMode.restype = c_int

        self.lib_emuc2_C.EMUCSetFilter.argtypes = (c_int, POINTER(FILTER_INFO))
        self.lib_emuc2_C.EMUCSetFilter.restype = c_int

        self.lib_emuc2_C.EMUCSetErrorType.argtypes = (c_int, c_int)
        self.lib_emuc2_C.EMUCSetErrorType.restype = c_int

        self.lib_emuc2_C.EMUCGetCfg.argtypes = (c_int, POINTER(CFG_INFO))
        self.lib_emuc2_C.EMUCGetCfg.restype = c_int

        self.lib_emuc2_C.EMUCExpCfg.argtypes = (c_int, c_char_p)
        self.lib_emuc2_C.EMUCExpCfg.restype = c_int

        self.lib_emuc2_C.EMUCImpCfg.argtypes = (c_int, c_char_p)
        self.lib_emuc2_C.EMUCImpCfg.restype = c_int

        self.lib_emuc2_C.EMUCSend.argtypes = (c_int, POINTER(CAN_FRAME_INFO))
        self.lib_emuc2_C.EMUCSend.restype = c_int

        self.lib_emuc2_C.EMUCReceive.argtypes = (c_int, POINTER(CAN_FRAME_INFO))
        self.lib_emuc2_C.EMUCReceive.restype = c_int

        self.lib_emuc2_C.EMUCReceiveNonblock.argtypes = (c_int, POINTER(NON_BLOCK_INFO))
        self.lib_emuc2_C.EMUCReceiveNonblock.restype = c_int

        self.lib_emuc2_C.EMUCEnableSendQueue.argtypes = (c_int, c_bool, c_uint)
        self.lib_emuc2_C.EMUCEnableSendQueue.restype = c_int

        self.lib_emuc2_C.EMUCGetBusError.argtypes = (c_int,)
        self.lib_emuc2_C.EMUCGetBusError.restype = c_int

        self.lib_emuc2_C.EMUCSetRecvBlock.argtypes = (c_int, c_bool)
        self.lib_emuc2_C.EMUCSetRecvBlock.restype = c_int

        if 'posix' in os.name:
            self.lib_emuc2_C.EMUCOpenSocketCAN.argtypes = (c_int,)
            self.lib_emuc2_C.EMUCOpenSocketCAN.restype = c_int
        
# ----------------------------------------------------------------------------------------------
    # ----- Python data type to C data type
    def __strToCharStr (self, _str):
        return ctypes.c_char_p(bytes(_str, 'ascii'))

# ----------------------------------------------------------------------------------------------
    def EMUCOpenDevice(self, _com_port):
        """ argu: (int) com_port, rtn: (int) """
        return self.lib_emuc2_C.EMUCOpenDevice(_com_port)

# ----------------------------------------------------------------------------------------------
    def EMUCCloseDevice(self, _com_port):
        """ argu: (int) com_port, rtn: (int) """
        return self.lib_emuc2_C.EMUCCloseDevice(_com_port)

# ----------------------------------------------------------------------------------------------
    def EMUCShowVer(self, _com_port, _ver_info):
        """ argu: (int) com_port, (VER_INFO) ver_info, rtn: (int) """
        input_info = [_ver_info,]
        return self.lib_emuc2_C.EMUCShowVer(_com_port, input_info[0])

# ----------------------------------------------------------------------------------------------
    def EMUCResetCAN(self, _com_port):
        """ argu: (int) com_port, rtn: (int) """
        return self.lib_emuc2_C.EMUCResetCAN(_com_port)

# ----------------------------------------------------------------------------------------------
    def EMUCClearFilter(self, _com_port, _CAN_port):
        """ argu: (int) com_port, (int) CAN_port, rtn: (int) """
        return self.lib_emuc2_C.EMUCClearFilter(_com_port, _CAN_port)

# ----------------------------------------------------------------------------------------------
    def EMUCInitCAN(self, _com_port, _CAN1_sts, _CAN2_sts):
        """ argu: (int) com_port, (int) CAN1_sts, (int) CAN2_sts, rtn: (int) """
        return self.lib_emuc2_C.EMUCInitCAN(_com_port, _CAN1_sts, _CAN2_sts)

# ----------------------------------------------------------------------------------------------
    def EMUCSetBaudRate(self, _com_port, _CAN1_baud, _CAN2_baud):
        """ argu: (int) com_port, (int) CAN1_baud, (int) CAN2_baud, rtn: (int) """
        return self.lib_emuc2_C.EMUCSetBaudRate(_com_port, _CAN1_baud, _CAN2_baud)

# ----------------------------------------------------------------------------------------------
    def EMUCSetMode(self, _com_port, _CAN1_mode, _CAN2_mode):
        """ argu: (int) com_port, (int) CAN1_mode, (int) CAN2_mode, rtn: (int) """
        return self.lib_emuc2_C.EMUCSetMode(_com_port, _CAN1_mode, _CAN2_mode)

# ----------------------------------------------------------------------------------------------
    def EMUCSetFilter(self, _com_port, _filter_info):
        """ argu: (int) com_port, (FILTER_INFO) filter_info, rtn: (int) """
        input_info = [_filter_info,]
        return self.lib_emuc2_C.EMUCSetFilter(_com_port, input_info[0])

# ----------------------------------------------------------------------------------------------
    def EMUCSetErrorType(self, _com_port, _err_type):
        """ argu: (int) com_port, (int) err_type, rtn: (int) """
        return self.lib_emuc2_C.EMUCSetErrorType(_com_port, _err_type)

# ----------------------------------------------------------------------------------------------
    def EMUCGetCfg(self, _com_port, _cfg_info):
        """ argu: (int) com_port, (CFG_INFO) cfg_info, rtn: (int) """
        input_info = [_cfg_info]
        return self.lib_emuc2_C.EMUCGetCfg(_com_port, input_info[0])

# ----------------------------------------------------------------------------------------------
    def EMUCExpCfg(self, _com_port, _file_name):
        """ argu: (int) com_port, (char *) file_name, rtn: (int) """
        return self.lib_emuc2_C.EMUCExpCfg(_com_port, self.__strToCharStr(_file_name))

# ----------------------------------------------------------------------------------------------
    def EMUCImpCfg(self, _com_port, _file_name):
        """ argu: (int) com_port, (char *) file_name, rtn: (int) """
        return self.lib_emuc2_C.EMUCImpCfg(_com_port, self.__strToCharStr(_file_name))

# ----------------------------------------------------------------------------------------------
    def EMUCSend(self, _com_port, _can_frame_info):
        """ argu: (int) com_port, (CAN_FRAME_INFO) can_frame_info, rtn: (int) """
        input_info = [_can_frame_info]
        return self.lib_emuc2_C.EMUCSend(_com_port, input_info[0])

# ----------------------------------------------------------------------------------------------
    def EMUCReceive(self, _com_port, _can_frame_info):
        """ argu: (int) com_port, (CAN_FRAME_INFO) can_frame_info, rtn: (int) """
        input_info = [_can_frame_info]
        return self.lib_emuc2_C.EMUCReceive(_com_port, input_info[0])

# ----------------------------------------------------------------------------------------------
    def EMUCReceiveNonblock(self, _com_port, _non_block_info):
        """ argu: (int) com_port, (NON_BLOCK_INFO) non_block_info, rtn: (int) """
        input_info = [_non_block_info]
        return self.lib_emuc2_C.EMUCReceiveNonblock(_com_port, input_info[0])

# ----------------------------------------------------------------------------------------------
    def EMUCEnableSendQueue(self, _com_port, _is_enable, _queue_size):
        """ argu: (int) com_port, (bool) is_enable, (int) queue_size, rtn: (int) """
        return self.lib_emuc2_C.EMUCEnableSendQueue(_com_port, _is_enable, _queue_size)
    
# ----------------------------------------------------------------------------------------------
    def EMUCGetBusError(self, _com_port):
        """ argu: (int) com_port, rtn: (int) """
        return self.lib_emuc2_C.EMUCGetBusError(_com_port)

# ----------------------------------------------------------------------------------------------
    def EMUCSetRecvBlock(self, _com_port, _is_enable):
        """ argu: (int) com_port, (bool) is_enable, rtn: (int) """
        return self.lib_emuc2_C.EMUCSetRecvBlock(_com_port, _is_enable)

# ----------------------------------------------------------------------------------------------
    if 'posix' in os.name:
        def EMUCOpenSocketCAN(self, _com_port): # linux only
            """ argu: (int) com_port, rtn: (int) """
            return self.lib_emuc2_C.EMUCOpenSocketCAN(_com_port)
	
	def EMUCSetCanOn(self,_com_port,_buadrate_can1,_buadrate_can2):
		# ----- EMUCOpenDevice() -----
		self.rtn = self.EMUCOpenSocketCAN(_com_port)
		if self.rtn:
			print 'Open /dev/ttyACM{} failed !'
		else:
			print 'Open /dev/ttyACM{EMUCReceiveNonblock} successfully !'
		# ----- EMUCInitCAN() -----
		self.rtn = self.EMUCInitCAN(_com_port, STATUS.EMUC_INACTIVE, STATUS.EMUC_INACTIVE)
		if self.rtn: 
			print 'EMUC initial CAN failed !' 
		else: 
			print 'EMUC initial CAN successfully !' 
		# ----- EMUCResetCAN() -----
		self.rtn = self.EMUCResetCAN(_com_port)
		if self.rtn: 
			print 'EMUC reset CAN failed !' 
		else: 
			print 'EMUC reset CAN successfully !' 
		# ----- EMUCClearFilter() -----
		self.rtn = self.EMUCClearFilter(_com_port, CAN_PORT.EMUC_CAN_1)
		self.rtn = self.rtn + self.EMUCClearFilter(_com_port, CAN_PORT.EMUC_CAN_2)
		if self.rtn: 
			print 'EMUC clear filter failed !' 
		else: 
			print 'EMUC clear filter successfully !' 
		# ----- EMUCSetBaudRate() -----
		self.rtn = self.EMUCSetBaudRate(_com_port, _buadrate_can1, _buadrate_can2)
		if self.rtn: 
			print 'EMUC set baudrate failed !' 
			time.sleep(0.1)   
		else:
			print 'EMUC set baudrate successfully !' 

		# ----- EMUCSetErrorType() -----
		self.rtn = self.EMUCSetErrorType(_com_port, ERR_TYPE.EMUC_DIS_ALL)
		if self.rtn: 
			print 'EMUC set error type failed !' 
		else: 
			print 'EMUC set error type successfully !' 
		
		# ----- EMUCSetMode() -----
		self.rtn = self.EMUCSetMode(_com_port, MODE.EMUC_NORMAL, MODE.EMUC_NORMAL)
		if self.rtn:
			print "EMUC set mode failed !"
		else:
			print "EMUC set mode successfully ! "
		
		# ----- Send & Receive must initial CAN to EMUC_ACTIVE -----
		self.EMUCInitCAN(_com_port, STATUS.EMUC_ACTIVE, STATUS.EMUC_ACTIVE)
		if self.rtn: 
			print 'EMUC initial CAN failed !' 
		else: 
			print 'EMUC initial CAN successfully !' 
		
		self.send_queue_size=0
		self.use_set_recv_block_flag=False
		# ----- EMUCSetRecvBlock() -----    To enable receive blockCAN_FRAME_INFO: linux is ready (windows not yet)
		if(self.use_set_recv_block_flag): 
			self.EMUCSetRecvBlock(_com_port, True)
		# ----- EMUCEnableSendQueue() -----
		if(self.send_queue_size): 
			self.EMUCEnableSendQueue(_com_port, True, self.send_queue_size)


		


