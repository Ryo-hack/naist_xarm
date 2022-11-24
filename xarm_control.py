#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy as rp
import numpy as np
from xarm.wrapper import XArmAPI

class xarm:
    def __init__(self, ip):
        self._arm = XArmAPI(ip) # set ip adress and connect
        self._arm.motion_enable(enable=True)
        self._arm.set_mode(0) # set mode to position control
        self._arm.set_state(state=0)

        self._arm.reset(wait=True)

    def _set_position(self, x, y, z, roll, pitch, yaw, speed=1000):
        self._arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=speed, wait=True)

    def _random_pos(self):
        x = 300
        y = np.random.randint(-700, 700)
        z = np.random.randint(100, 967)

        self._set_position(x,y,z,0,0,0)
        self._arm.set_tool_position(0,0,0,90,0,0)


    def _reset(self):
        self._arm.reset(wait=True)

def main():
    try:
        arm = xarm()
        arm._random_pos()
        
    except rp.ROSInterruptException:
        pass

if __name__ == '__name__':
    main()
