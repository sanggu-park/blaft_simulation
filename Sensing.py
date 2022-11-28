#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  8 22:42:38 2022

@author: sanggupark
"""

import numpy as np
import traci
from dataclasses import dataclass
import math
from shapely.geometry import LineString, Point
from SimpleMath import create_vehicle_shape


@dataclass(init = True)
class Object_sensed:
    ID: str
    xpos: float
    ypos: float
    vel: float
    angle: float
    width: float
    length: float
    acc_max: float = 4.0
    dec_max: float = 7.0
    dec_min: float = 2.0
    response: float = 0.2
    blinker: int = 0

def lidar_sensing(ego, veh_other):

    xpos = ego.xpos
    ypos = ego.ypos
    length = ego.length
    rad = np.radians(ego.angle)
    p_tail = Point([xpos-(length)*math.sin(rad), 
                    ypos-(length)*math.cos(rad)])
    
    # FOV control
    if "LF" in ego.behavior:
        angles = np.linspace(rad+(ego.sensor.fov/4), rad-(ego.sensor.fov/4), 100)
    elif "LC_R" in ego.behavior:
        angles = np.linspace(rad+(ego.sensor.fov/2.0), rad-(ego.sensor.fov/2.0), 100)
    elif "LC_L" in ego.behavior:
        angles = np.linspace(rad+(ego.sensor.fov/2.0), rad-(ego.sensor.fov/2.0), 100)

    distance = ego.sensor.radius * 1.00
    lines = []
    for angle in angles:
        line = LineString([[p_tail.x, p_tail.y], [p_tail.x + distance*math.sin(angle), p_tail.y + distance*math.cos(angle)]])
        lines.append(line)

    vehicles_sensed = []
    follower = traci.vehicle.getFollower(ego.ID, dist=5.0)[0]

    """ LIDAR Sensing """
    for veh in veh_other:
        is_detected = False
        poly_veh = create_vehicle_shape(veh)
        if veh.ID != ego.ID:
            for line in lines:
                is_detected = poly_veh.intersects(line)
                if is_detected:
                    break
            if is_detected and not (veh.ID == follower):
                vehicles_sensed.append(veh)

    return vehicles_sensed



def blinker_sensing(ego, vehicles_sensed):
    """ Blinker Sensing """
    for veh in vehicles_sensed:
        blinker = traci.vehicle.getSignals(veh.ID)
        
        # If LF """
        if blinker == 0:
            veh.blinker = 0
        # If LC_R """
        elif blinker == 1:
            veh.blinker = -1
        # If LC_L """
        elif blinker == 2:
            veh.blinker = 1
            
    return vehicles_sensed



def update_prev_info(ego, vehicles_sensed):
    """ Update Old info """
    for veh in vehicles_sensed:
        if 'auv' in veh.ID:
            object_add = Object_sensed(veh.ID, veh.xpos, veh.ypos, veh.vel, veh.angle, veh.width, veh.length, blinker=veh.blinker)
        elif 'huv' in veh.ID:
            blinker = traci.vehicle.getSignals(veh.ID)
            
            if blinker == 1:
                blinker = -1
            elif blinker == -1:
                blinker = 1
            else:
                blinker = 0
            
            object_add = Object_sensed(veh.ID, veh.xpos, veh.ypos, veh.vel, veh.angle, veh.width, veh.length, blinker=-traci.vehicle.getSignals(veh.ID))

        if len(ego.objects_sensed):
            flag = False
            for obj in ego.objects_sensed:
                if obj.ID == object_add.ID:
                    # replacement due to overlaps
                    ego.objects_sensed[np.where(ego.objects_sensed==obj)] = object_add
                    flag = True
            if not flag:
                # add if no overlaps
                ego.objects_sensed = np.append(ego.objects_sensed, object_add)
        else:
            # if the list is empty
            ego.objects_sensed = np.append(ego.objects_sensed, object_add)

    return