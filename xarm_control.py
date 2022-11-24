#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy as rp
import numpy as np
import math, time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

class xarm:
    def __init__(self, ip):
        self._arm = XArmAPI(ip) # set ip adress and connect
        self._arm.motion_enable(enable=True)
        self._arm.set_mode(0) # set mode to position control
        self._arm.set_state(state=0)

        self._arm.reset(wait=True)

    def _offset_deg(self,x,y,z):
        offset_deg = 50
        alt_x=x*math.cos(offset_deg*(math.pi/180)) + y*math.sin(offset_deg*(math.pi/180))
        alt_y=-(x*math.sin(offset_deg*(math.pi/180)) - y*math.cos(offset_deg*(math.pi/180)))
        alt_z=z
        return alt_x, alt_y, alt_z
     
    def _set_position(self, x, y, z, roll, pitch, yaw, speed=50):
        ox, oy, oz = self._offset_deg(x,y,z)
        self._arm.set_tool_position(x=ox, y=oy, z=oz, roll=roll, pitch=pitch, yaw=yaw, speed=speed, wait=True)

    def _random_pos(self):
        x = 700
        y = np.random.randint(-700, 700)
        z = np.random.randint(100, 967)
        try:
            # x:300, y:0, z:-300, roll:0, pitch:180, yaw:0
            # self._set_position(0,0,-300,0,180,0)
            print(self._arm.get_position())
        except self._arm.error_code:
            print('error')
            self._reset()
        # self._arm.set_tool_position(0,0,0,90,0,0)


    def _reset(self):
        self._arm.reset(wait=True)
        self._arm.disconnect()

def main():
    
    try:
    #     time.sleep(2)
        arm = xarm('192.168.1.213')
        arm._random_pos()
        arm._reset()
        
    except rp.ROSInterruptException:
        print('yo')
        pass

if __name__ == '__main__':
    main()
