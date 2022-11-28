#!/usr/bin/env python3
# -*- coding: utf-8 -*-

delta_T = 0.1

class HUV:
    def __init__(self, ID, xpos, ypos, vel, angle, length, width, height):
        
        """ Containing Info """
        self.ID = ID
        self.xpos = float(xpos)
        self.ypos = float(ypos)
        self.vel = float(vel)
        self.angle = float(angle)
        self.length = float(length)
        self.width = float(width)
        self.height = float(height)
        self.blinker = int(0)
