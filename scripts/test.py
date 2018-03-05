#!/usr/bin/env python
# -*- coding: utf-8 -*-
import KMControllers
from time import sleep

dev = KMControllers.BLEController("dd:7a:96:3c:e1:51")
dev.enable()
dev.speed(100.0)
dev.runReverse()
while True:
    sleep(5)
    pos, vel, tor = dev.read_motor_measurement()
    print("Position = " + str(pos))
    print("Velocity = " + str(vel))
    print("Torque = " + str(tor) + "\n")
# dev.speed(1.0)
# dev.runReverse()
# sleep(3)
# pos, vel, tor = dev.read_motor_measurement()
# print("Position = " + str(pos) + "\n")
# print("Velocity = " + str(vel) + "\n")
# print("Torque = " + str(tor) + "\n\n")
#dev.disconnect()
