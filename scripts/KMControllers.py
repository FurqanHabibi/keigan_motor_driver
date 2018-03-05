#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Created on Sun Dec 10 16:50:24 2017

@author: takata@innovotion.co.jp
"""
from __future__ import division
from __future__ import absolute_import
import serial,struct
from bluepy import btle

def float2bytes(float_value):
    float_value=float(float_value)
    return struct.pack(u"!f", float_value)

def bytes2float(byte_array):
    return struct.unpack(u'!f',byte_array)[0]

def uint8_t2bytes(uint8_value):
    uint8_value=int(uint8_value)
    if uint8_value>256-1:
        uint8_value=256-1
    return struct.pack(u"B",uint8_value)

def uint16_t2bytes(uint16_value):
    uint16_value=int(uint16_value)
    if uint16_value>256**2-1:
        uint16_value=256**2-1
    val1=int(uint16_value/256)
    val2=uint16_value-val1*256
    return struct.pack(u"BB",val1,val2)

def bytes2uint16_t(ba):
    return struct.unpack(u"BB",ba)[0]

def bytes2uint8_t(ba):
    return struct.unpack(u"B",ba)[0]

def bytes2int16_t(ba):
    return struct.unpack(u">h",ba)[0]

def uint32_t2bytes(uint32_value):
    uint32_value=int(uint32_value)
    if uint32_value>256**4-1:
        uint32_value=256**4-1
    val1=int(uint32_value/256**3)
    val2=int((uint32_value-val1*256**3)/256**2)
    val3=int((uint32_value-val1*256**3-val2*256**2)/256)
    val4=uint32_value-val1*256**3-val2*256**2-val3*256
    return struct.pack(u"BBBB",val1,val2,val3,val4)

class Controller(object):
    def __init__(self):
        pass

    def run_command(self,val,characteristics):
        print val,characteristics

    # Settings
    def maxSpeed(self,max_speed,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the maximum speed of rotation to the 'max_speed' in rad/sec.
        """
        command='\x02'
        values=float2bytes(max_speed)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def minSpeed(self,min_speed,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the minimum speed of rotation to the 'min_speed' in rad/sec.
        """
        command='\x03'
        values=float2bytes(min_speed)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def curveType(self,curve_type,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the acceleration or deceleration curve to the 'curve_type'.
        typedef enum curveType =
        {
            CURVE_TYPE_NONE = 0, // Turn off Motion control
            CURVE_TYPE_TRAPEZOID = 1, // Turn on Motion control with trapezoidal curve
        }
        """
        
        command='\x05'
        values=uint8_t2bytes(curve_type)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def acc(self,_acc,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the acceleration of rotation to the positive 'acc' in rad/sec^2.
        """
        command='\x07'
        values=float2bytes(_acc)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def dec(self,_dec,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the deceleration of rotation to the positive 'dec' in rad/sec^2.
        """
        command='\x08'
        values=float2bytes(_dec)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def maxTorque(self,max_torque,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the maximum torque to the positive 'max_torque' in N.m.
        """
        command='\x0E'
        values=float2bytes(max_torque)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def qCurrentP(self,q_current_p,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the q-axis current PID controller's Proportional gain to the postiive 'q_current_p'.
        """
        command='\x18'
        values=float2bytes(q_current_p)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def qCurrentI(self,q_current_i,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the q-axis current PID controller's Integral gain to the positive 'q_current_i'.
        """
        command='\x19'
        values=float2bytes(q_current_i)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def qCurrentD(self,q_current_d,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the q-axis current PID controller's Differential gain to the postiive 'q_current_d'.
        """
        command='\x1A'
        values=float2bytes(q_current_d)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def speedP(self,speed_p,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the speed PID controller's Proportional gain to the positive 'speed_p'.
        """
        command='\x1B'
        values=float2bytes(speed_p)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def speedI(self,speed_i,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the speed PID controller's Integral gain to the positive 'speed_i'.
        """
        command='\x1C'
        values=float2bytes(speed_i)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def speedD(self,speed_d,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the speed PID controller's Deferential gain to the positive 'speed_d'.
        """
        command='\x1D'
        values=float2bytes(speed_d)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def positionP(self,position_p,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the position PID controller's Proportional gain to the positive 'position_p'.
        """
        command='\x1E'
        values=float2bytes(position_p)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def resetPID(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Reset all the PID parameters to the firmware default settings.
        """
        command='\x22'
        self.run_command(command+identifier+crc16,u'motor_settings')

    def ownColor(self,red,green,blue,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the own LED color.
        """
        command='\x3A'
        values=uint8_t2bytes(red)+uint8_t2bytes(green)+uint8_t2bytes(blue)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def readRegister(self,register,identifier='\x00\x00',crc16='\x00\x00'):
        u'''
        Read a specified setting (register).
        '''
        command='\x40'
        values=uint8_t2bytes(register)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def saveAllRegisters(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Save all settings (registers) in flash memory.
        """
        command='\x41'
        self.run_command(command+identifier+crc16,u'motor_settings')

    def resetRegister(self,register,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Reset a specified register's value to the firmware default setting.
        """
        command='\x4E'
        values=uint8_t2bytes(register)
        self.run_command(command+identifier+values+crc16,u'motor_settings')

    def resetAllRegisters(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Reset all registers' values to the firmware default setting.
        """
        command='\x4F'
        self.run_command(command+identifier+crc16,u'motor_settings')

    # Motor Action
    def disable(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Disable motor action.
        """
        command='\x50'
        self.run_command(command+identifier+crc16,u'motor_control')

    def enable(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Enable motor action.
        """
        command='\x51'
        self.run_command(command+identifier+crc16,u'motor_control')

    def speed(self,speed,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the speed of rotation to the positive 'speed' in rad/sec.
        """
        command='\x58'
        values=float2bytes(speed)
        self.run_command(command+identifier+values+crc16,u'motor_control')

    def presetPosition(self,position,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Preset the current absolute position as the specified 'position' in rad. (Set it to zero when setting origin)
        """
        command='\x5A'
        values=float2bytes(position)
        self.run_command(command+identifier+values+crc16,u'motor_control')

    def runForward(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Rotate the motor forward (counter clock-wise) at the speed set by 0x58: speed.
        """
        command='\x60'
        self.run_command(command+identifier+crc16,u'motor_control')

    def runReverse(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Rotate the motor reverse (clock-wise) at the speed set by 0x58: speed.
        """
        command='\x61'
        self.run_command(command+identifier+crc16,u'motor_control')

    def moveTo(self,position,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Move the motor to the specified absolute 'position' at the speed set by 0x58: speed.
        """
        command='\x66'
        values=float2bytes(position)
        self.run_command(command+identifier+values+crc16,u'motor_control')

    def moveBy(self,distance,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Move motor by the specified relative 'distance' from the current position at the speed set by 0x58: speed.
        """
        command='\x68'
        values=float2bytes(distance)
        self.run_command(command+identifier+values+crc16,u'motor_control')

    def free(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Stop the motor's excitation
        """
        command='\x6C'
        self.run_command(command+identifier+crc16,u'motor_control')

    def stop(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Decelerate the speed to zero and stop.
        """
        command='\x6D'
        self.run_command(command+identifier+crc16,u'motor_control')

    def holdTorque(self,torque,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Keep and output the specified torque.
        """
        command='\x72'
        values=float2bytes(torque)
        self.run_command(command+identifier+values+crc16,u'motor_control')

    def doTaskSet(self,index,repeating,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Do taskset at the specified 'index' 'repeating' times.
        """
        command='\x81'
        values=uint16_t2bytes(index)+uint32_t2bytes(repeating)
        self.run_command(command+identifier+values+crc16,u'motor_control')

    def preparePlaybackMotion(self,index,repeating,option,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Prepare to playback motion at the specified 'index' 'repeating' times.
        """
        command='\x86'
        values=uint16_t2bytes(index)+uint32_t2bytes(repeating)+uint8_t2bytes(option)
        self.run_command(command+identifier+values+crc16,u'motor_control')

    def startPlaybackMotion(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Start to playback motion in the condition of the last preparePlaybackMotion.
        """
        command='\x87'
        self.run_command(command+identifier+crc16,u'motor_control')

    def stopPlaybackMotion(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Stop to playback motion.
        """
        command='\x88'
        self.run_command(command+identifier+crc16,u'motor_control')

    # Queue
    def pause(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Pause the queue until 0x91: resume is executed.
        """
        command='\x90'
        self.run_command(command+identifier+crc16,u'motor_control')

    def resume(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Resume the queue.
        """
        command='\x91'
        self.run_command(command+identifier+crc16,u'motor_control')

    def wait(self,time,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Wait the queue or pause the queue for the specified 'time' in msec and resume it automatically.
        """
        command='\x92'
        values=uint32_t2bytes(time)
        self.run_command(command+identifier+values+crc16)

    def reset(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Reset the queue. Erase all tasks in the queue. This command works when 0x90: pause or 0x92: wait are executed.
        """
        command='\x95'
        self.run_command(command+identifier+crc16,u'motor_control')

    # Taskset
    def startRecordingTaskset(self,index,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Start recording taskset at the specified 'index' in the flash memory.
        In the case of KM-1, index value is from 0 to 49 (50 in total). 
        """
        command='\xA0'
        values=uint16_t2bytes(index)
        self.run_command(command+identifier+values+crc16)

    def stopRecordingTaskset(self,index,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Stop recording taskset.
        This command works while 0xA0: startRecordingTaskset is executed.
        """
        command='\xA2'
        values=uint16_t2bytes(index)
        self.run_command(command+identifier+values+crc16)

    def eraseTaskset(self,index,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Erase taskset at the specified index in the flash memory.
        In the case of KM-1, index value is from 0 to 49 (50 in total). 
        """
        command='\xA3'
        values=uint16_t2bytes(index)
        self.run_command(command+identifier+values+crc16)

    def eraseAllTaskset(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Erase all tasksets in the flash memory.  
        """
        command='\xA4'
        self.run_command(command+identifier+crc16,u'motor_control')

    # Teaching
    def prepareTeachingMotion(self,index,time,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Prepare teaching motion by specifying the 'index' in the flash memory and recording 'time' in milliseconds.
        In the case of KM-1, index value is from 0 to 9 (10 in total).  Recording time cannot exceed 65408 [msec].
        """
        command='\xAA'
        values=uint16_t2bytes(index)+uint32_t2bytes(time)
        self.run_command(command+identifier+values+crc16)

    def startTeachingMotion(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Start teaching motion in the condition of the last prepareTeachingMotion.
        This command works when the teaching index is specified by 0xAA: prepareTeachingMotion.
        """
        command='\xAB'
        self.run_command(command+identifier+crc16,u'motor_control')

    def stopTeachingMotion(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Stop teaching motion.
        """
        command='\xAC'
        self.run_command(command+identifier+crc16,u'motor_control')

    def eraseMotion(self,index,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Erase motion at the specified index in the flash memory.
        In the case of KM-1, index value is from 0 to 9 (10 in total). 
        """
        command='\xA4'
        values=uint16_t2bytes(index)
        self.run_command(command+identifier+values+crc16)

    def eraseAllMotion(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Erase all motion in the flash memory.
        """
        command='\xAE'
        self.run_command(command+identifier+crc16,u'motor_control')

    # LED
    def led(self,ledState,red,green,blue,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Set the LED state (off, solid, flash and dim) and color intensity (red, green and blue).
        typedef enum ledState =
        {
            LED_STATE_OFF = 0, // LED off 
            LED_STATE_ON_SOLID = 1, // LED solid
            LED_STATE_ON_FLASH = 2, // LED flash 
            LED_STATE_ON_DIM = 3 // LED dim
        } 
        """
        command='\xE0'
        values=uint8_t2bytes(ledState)+uint8_t2bytes(red)+uint8_t2bytes(green)+uint8_t2bytes(blue)
        self.run_command(command+identifier+values+crc16,u"motor_control")

    # IMU
    def enableIMU(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Enable the IMU and start notification of the measurement values.
        This command is only available for BLE (not implemented on-wired.)
        When this command is executed, the IMU measurement data is notified to BLE IMU Measuement characteristics.
        """
        command='\xEA'
        self.run_command(command+identifier+crc16,u'motor_control')

    def disableIMU(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Disable the IMU and stop notification of the measurement values.
        """
        command='\xEB'
        self.run_command(command+identifier+crc16,u'motor_control')

    # System
    def reboot(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Reboot the system.
        """
        command='\xF0'
        self.run_command(command+identifier+crc16,u'motor_control')

    def enterDeviceFirmwareUpdate(self,identifier='\x00\x00',crc16='\x00\x00'):
        u"""
        Enter the device firmware update mode.
        Enter the device firmware update mode or bootloader mode. It goes with reboot.
        """
        command='\xFD'
        self.run_command(command+identifier+crc16,u'motor_control')


class USBController(Controller):
    def __init__(self,port=u'/dev/ttyUSB0'):
        self.port=port
        self.serial=serial.Serial(port,115200,8,u'N',1,None,False,True)

    def run_command(self,val,characteristics=None):
        self.serial.write(val)

class BLEController(Controller):
    def __init__(self,addr):
        self.address=addr
        self.dev=btle.Peripheral(self.address,u'random')
        self.position=0.0
        self.velocity=0.0
        self.torque=0.0
        self.accel_x=0.0
        self.accel_y=0.0
        self.accel_z=0.0
        self.temp=0
        self.gyro_x=0
        self.gyro_y=0
        self.gyro_z=0
        for v in self.dev.getCharacteristics():
            if v.uuid==u'f1400001-8936-4d35-a0ed-dfcd795baa8c':
                self.motor_control_handle=v.getHandle()
            if v.uuid==u'f1400003-8936-4d35-a0ed-dfcd795baa8c':
                self.motor_led_handle=v.getHandle()
            if v.uuid==u'f1400004-8936-4d35-a0ed-dfcd795baa8c':
                self.motor_measurement_handle=v.getHandle()
            if v.uuid==u'f1400005-8936-4d35-a0ed-dfcd795baa8c':
                self.motor_imu_measurement_handle=v.getHandle()
            if v.uuid==u'f1400006-8936-4d35-a0ed-dfcd795baa8c':
                self.motor_settings_handle=v.getHandle()
    def run_command(self,val,characteristics=None):
        if characteristics==u'motor_control':
            self.dev.writeCharacteristic(self.motor_control_handle,val)
        elif characteristics==u'motor_led':
            self.dev.writeCharacteristic(self.motor_led_handle,val)
        elif characteristics==u'motor_settings':
            self.dev.writeCharacteristic(self.motor_settings_handle,val)
        else:
            raise ValueError(u'Invalid Characteristics')
    
    def connect(self):
        u"""
        Establish the BLE connection.
        """
        self.dev.connect(self.address,u'random')
    
    def disconnect(self):
        u"""
        Close the BLE connection.
        """
        self.dev.disconnect()
        
    def read_motor_measurement(self):
        u"""
        Get the position, velocity, and torque and store them to the properties 'position' in rad, 'velocity' in rad/sec, and 'torque' in N.m.
        """
        ba=self.dev.readCharacteristic(self.motor_measurement_handle)
        self.position=bytes2float(ba[0:4])
        self.velocity=bytes2float(ba[4:8])
        self.torque=bytes2float(ba[8:12])
        return (self.position,self.velocity,self.torque)
    
    def read_imu_mesurement(self):
        u"""
        Get the x,y,z axis acceleration, temperature, and anguler velocities around x,y,z axis
        and store them to 'accel_x', 'accel_y', 'accel_z' in g(9.80665 m/s^2), 'temp' in degree Celsius, 'gyro_x', 'gyro_y', and 'gyro_z' in rad/sec.
        """
        self.enableIMU()
        ba=self.dev.readCharacteristic(self.motor_imu_measurement_handle)
        self.accel_x=bytes2int16_t(ba[0:2])* 2.0 / 32767
        self.accel_y=bytes2int16_t(ba[2:4])* 2.0 / 32767
        self.accel_z=bytes2int16_t(ba[4:6])* 2.0 / 32767
        self.temp=bytes2int16_t(ba[6:8])/333.87 + 21.00
        self.gyro_x=bytes2int16_t(ba[8:10])* 0.00013316211
        self.gyro_y=bytes2int16_t(ba[10:12])* 0.00013316211
        self.gyro_z=bytes2int16_t(ba[12:14])* 0.00013316211
        return (self.accel_x,self.accel_y,self.accel_z,self.temp,self.gyro_x,self.gyro_y,self.gyro_z)
    
    def __read_float_data(self,ba):
        return bytes2float(ba[4:8])
    
    def __read_uint8_data(self,ba):
        return bytes2uint8_t(ba[4])
    
    def __read_rgb_data(self,ba):
        return (ba[4],ba[5],ba[6])
    
    def __read_setting_value(self,comm):
        float_value_comms=[0x02,0x03,0x07,0x08,0x0E,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E]
        valid_comms=[0x05,0x3A]
        valid_comms.extend(float_value_comms)
        if not (comm in valid_comms):
            return
        self.readRegister(comm)
        ba=self.dev.readCharacteristic(self.motor_settings_handle)
        while len(ba)==6:
            ba=self.dev.readCharacteristic(self.motor_settings_handle)
        if comm in float_value_comms:
            return self.__read_float_data(ba)
        if comm==0x05:
            return self.__read_uint8_data(ba)
        if comm==0x3A:
            return self.__read_rgb_data(ba)
    
    def read_maxSpeed(self):
        return self.__read_setting_value(0x02)
            
    def read_minSpeed(self):
        return self.__read_setting_value(0x03)
    
    def read_curveType(self):
        return self.__read_setting_value(0x05)
    
    def read_acc(self):
        return self.__read_setting_value(0x07)
    
    def read_dec(self):
        return self.__read_setting_value(0x08)
    
    def read_maxTorque(self):
        return self.__read_setting_value(0x0E)
    
    def read_qCurrentP(self):
        return self.__read_setting_value(0x18)
    
    def read_qCurrentI(self):
        return self.__read_setting_value(0x19)
    
    def read_qCurrentD(self):
        return self.__read_setting_value(0x1A)
    
    def read_speedP(self):
        return self.__read_setting_value(0x1B)
    
    def read_speedI(self):
        return self.__read_setting_value(0x1C)
    
    def read_speedD(self):
        return self.__read_setting_value(0x1D)
    
    def read_positionP(self):
        return self.__read_setting_value(0x1E)
    
    def read_ownColor(self):
        return self.__read_setting_value(0x3A)
