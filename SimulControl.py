#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 21 15:25:17 2021

@author: sanggupark
"""
import traci
import random

import Parser
import Router
from AUV import AUV
from HUV import HUV

delta_T = 0.1

def create_vehicles(numAUV, numHUV, route_file, typelist, routelist, MapGraph, Connection):
    """ 0-C. Create vehicles (CAVs) in SUMO and Extract waypoints """
    
    auvs = give_birth_to_veh(numAUV, "auv", typelist, routelist, MapGraph, Connection, route_file)    
    huvs = give_birth_to_veh(numHUV, "huv", typelist, routelist, MapGraph, Connection, route_file)    

    return auvs, huvs



def give_birth_to_veh(numVEH, type_str, typelist, routelist, MapGraph, Connection, route_file):
    
    """ 0-C. Create vehicles (CAVs) in SUMO and Extract waypoints """
    vehicles = []
    vel_init = 0.0

    if 'auv' in type_str:
        for i in range(numVEH):
            vehID, xpos, ypos, src, dst, vel, angle, acc_max, dec_min, dec_max, response, length, width, height = create_vehicle(i, vel_init, typelist, 
                                                                                                                                 routelist, route_file, 
                                                                                                                                 auv=True)
            vel = vel_init
            acc_max = float(acc_max) * delta_T
            dec_min = float(dec_min) * delta_T
            dec_max = float(dec_max) * delta_T
            
            WayPoints, RouteGraph, InterLane = Router.generate_waypoint(src, dst, MapGraph, Connection)
            
            vehicles.append(AUV(vehID, src, dst, xpos, ypos,
                            vel, angle, acc_max, dec_min, dec_max, 
                            response, length, width, height, MapGraph, RouteGraph, Connection, WayPoints, InterLane))
                
            
    elif 'huv' in type_str:
        vel_init = 12.0
        for i in range(numVEH):
            vehID, xpos, ypos, src, dst, vel, angle, acc_max, dec_min, dec_max, response, length, width, height = create_vehicle(i, vel_init, typelist, 
                                                                                                                                 routelist, route_file, 
                                                                                                                                 auv=False)
            vehicles.append(HUV(vehID, xpos, ypos, vel, angle, length, width, height))
        
    return vehicles
        


def create_vehicle(i, vel_init, typelist, routelist, route_file, auv=True):
    if auv == True:
        vehID = "auv" + str(i+1)
        response = 0.1
    else:
        vehID = "huv" + str(i+1)
        response = 0.2
        
    typeID = random.choice(typelist)
    routeID = random.choice(routelist)
    if auv:
        routeID = random.choice([
                                # 'test', 
                                "route1_5_r", "route5_1_r", 
                                "route3_1_r", "route3_2_r",
                                "route4_2_r", "route2_4_r", 
                                "route6_0_r", "route0_6_r",
                                "route2_5_s", "route5_2_s", 
                                "route3_1_s", "route3_2_s", 
                                "route4_1_s", "route1_4_s", 
                                "route6_0_s", "route0_6_s",
                                "route0_3_l", "route3_0_l", 
                                "route1_2_l", "route2_1_l",
                                "route0_3_r", "route3_0_r", 
                                "route1_2_r", "route2_1_r",
                                "route3_6_l", "route6_3_l", 
                                "route4_5_l", "route5_4_l",
                                "route3_6_r", "route6_3_r", 
                                "route4_5_r", "route5_4_r"
                                ])
    
    if not auv:
        routeID = random.choice([
                                # "test"
                                "route_h_1_1",
                                "route_h_1_2",
                                "route_h_2_1", # do not use this
                                "route_h_2_2",
                                "route_h_3_1", # do not use
                                "route_h_3_2",
                                "route_h_4_1",
                                "route_h_4_2"
                                ])

    if routeID == 'route1_5_r':
        src = [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(50, 100, 8)] # 1to3
        dst = [random.randrange(50, 100, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
    elif routeID == 'route5_1_r':
        src = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(300, 350, 8)] # 5to3
        dst = [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] # 0to1
        
    elif routeID == 'route3_1_r':
        src = [random.randrange(300, 350, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 3to4
        dst = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(300, 350, 8)] # 5to3
    elif routeID == 'route3_2_r':
        src = [random.randrange(-200, -150, 8), random.choice([251.6, 254.8, 258.0, 261.2])] # 3to2
        dst = [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(50, 100, 8)] # 1to3
        
    elif routeID == 'route4_2_r':
        src = [random.choice([251.6, 254.8, 258.0, 261.2]), random.randrange(400, 450, 8)] # 4to6
        dst = [random.randrange(-150, -100, 8), random.choice([251.6, 254.8, 258.0, 261.2])] #3to2
    elif routeID == 'route2_4_r':
        src = [random.choice([-251.6, -254.8, -258.0, -261.2]), random.randrange(50, 100, 8)] # 2to0
        dst = [random.randrange(50, 100, 8), random.choice([248.4, 245.2, 242.0, 238.8])] #3to4  

    elif routeID == 'route6_0_r':
        src = [random.randrange(50, 100, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
        dst = [random.choice([-251.6, -254.8, -258.0, -261.2]), random.randrange(50, 100, 8)] # 2to0
    elif routeID == 'route0_6_r':
        src = [random.randrange(-200, -150, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] # 0to1
        dst = [random.choice([251.6, 254.8, 258.0, 261.2]), random.randrange(400, 450, 8)] # 4to6


    elif routeID == 'route2_5_s':
        src = [random.randrange(-200, -150, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 2to3
        dst = [random.randrange(50, 100, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
    elif routeID == 'route5_2_s':
        src = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(400, 450, 8)] # 5to3
        dst = [random.choice([-248.4, -245.2, -242.0, -238.8]), random.randrange(50, 100, 8)] #0to2
        
    elif routeID == 'route3_1_s':
        src = [random.randrange(300, 350, 8), random.choice([248.4, 245.2, 242.0, 238.8])] #3to4  
        dst = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(400, 450, 8)] # 5to3
    elif routeID == 'route3_2_s':
        src = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(50, 100, 8)] # 3to1
        dst = [random.randrange(-150, -100, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 2to3

        
    elif routeID == 'route4_1_s':
        src = [random.choice([251.6, 254.8, 258.0, 261.2]), random.randrange(400, 450, 8)] # 4to6
        dst = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(50, 100, 8)] # 3to1
    elif routeID == 'route1_4_s':
        src = [random.randrange(-200, -150, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] #1to0
        dst = [random.randrange(300, 350, 8), random.choice([248.4, 245.2, 242.0, 238.8])] #3to4  
        
    elif routeID == 'route6_0_s':
        src = [random.randrange(50, 100, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
        dst = [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] #1to0
    elif routeID == 'route0_6_s':
        src = [random.choice([-248.4, -245.2, -242.0, -238.8]), random.randrange(50, 100, 8)] #0to2
        dst = [random.choice([251.6, 254.8, 258.0, 261.2]), random.randrange(300, 350, 8)] # 4to6
        
        
    elif routeID == 'route0_3_l':
        src = [random.randrange(-200, -150, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] # 0to1
        dst = [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(50, 100, 8)] # 1to3
    elif routeID == 'route3_0_l':
        src = [random.randrange(-200, -150, 8), random.choice([251.6, 254.8, 258.0, 261.2])] #3to2
        dst = [random.choice([-251.6, -254.8, -258.0, -261.2]), random.randrange(50, 100, 8)] # 2to0
        
    elif routeID == 'route1_2_l':
        src = [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(50, 100, 8)] # 1to3
        dst = [random.randrange(-150, -100, 8), random.choice([251.6, 254.8, 258.0, 261.2])] #3to2
    elif routeID == 'route2_1_l':
        src = [random.choice([-251.6, -254.8, -258.0, -261.2]), random.randrange(50, 100, 8)] # 2to0
        dst = [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] # 0to1
        
        
    elif routeID == 'route0_3_r':
        src = [random.choice([-248.4, -245.2, -242.0]), random.randrange(50, 100, 8)] # 0to2
        dst = [random.randrange(-150, -100, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 2to3
    elif routeID == 'route3_0_r':
        src = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(50, 100, 8)] # 3to1
        dst = [random.randrange(-150, -100, 8), random.choice([1.6, 4.8, 8.0, 11.2])] # 1to0
        
    elif routeID == 'route1_2_r':
        src = [random.randrange(-200, -150, 8), random.choice([1.6, 4.8, 8.0, 11.2])] # 1to0
        dst = [random.choice([-248.4, -245.2, -242.0]), random.randrange(50, 100, 8)] # 0to2
    elif routeID == 'route2_1_r':
        src = [random.randrange(-200, -150, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 2to3
        dst = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(50, 100, 8)] # 3to1
        
        
    elif routeID == 'route3_6_l':
        src = [random.randrange(400, 450, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 3to4  
        dst = [random.choice([251.6, 254.8, 258.0, 261.2]), random.randrange(400, 450, 8)] # 4to6
    elif routeID == 'route6_3_l':
        src = [random.randrange(50, 100, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
        dst = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(400, 450, 8)] # 5to3
        
    elif routeID == 'route4_5_l':
        src = [random.choice([251.6, 254.8, 258.0, 261.2]), random.randrange(400, 450, 8)] # 4to6
        dst = [random.randrange(50, 100, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
    elif routeID == 'route5_4_l':
        src = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(400, 450, 8)] # 5to3
        dst = [random.randrange(300, 350, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 3to4  
        
        
    elif routeID == 'route3_6_r':
        src = [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(400, 450, 8)] # 3to5
        dst = [random.randrange(50, 100, 8), random.choice([498.4, 495.2, 492.0, 488.8])] # 5to6
    elif routeID == 'route6_3_r':
        src = [random.choice([248.4, 245.2, 242.0, 238.8]), random.randrange(400, 450, 8)] # 6to4
        dst = [random.randrange(50, 100, 8), random.choice([251.6, 254.8, 258.0, 261.2])] # 4to3
        
    elif routeID == 'route4_5_r':
        src = [random.randrange(50, 100, 8), random.choice([251.6, 254.8, 258.0, 261.2])] # 4to3
        dst = [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(400, 450, 8)] # 3to5
    elif routeID == 'route5_4_r':
        src = [random.randrange(50, 100, 8), random.choice([498.4, 495.2, 492.0, 488.8])] # 5to6
        dst = [random.choice([248.4, 245.2, 242.0, 238.8]), random.randrange(400, 450, 8)] # 6to4
        
        
        
        
    elif routeID == 'route_h_1_1':
        src = [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(50, 100, 8)] # 1to3
        dst = [random.randrange(150, 200, 8), random.choice([498.4, 495.2, 492.0, 488.8])] # 5to6
    elif routeID == 'route_h_1_2':
        src = [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(50, 100, 8)] # 1to3
        dst = [random.randrange(150, 200, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
        
    elif routeID == 'route_h_2_1':
        src = [random.randrange(-220, -180, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 2to3
        dst = [random.randrange(50, 100, 8), random.choice([498.4, 495.2, 492.0, 488.8])] # 5to6
    elif routeID == 'route_h_2_2':
        src = [random.randrange(-200, -150, 8), random.choice([248.4, 245.2, 242.0, 238.8])] # 2to3
        dst = [random.randrange(50, 100, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
        
        
    elif routeID == 'route_h_3_1':
        src = [random.randrange(150, 200, 8), random.choice([251.6, 254.8, 258.0, 261.2])] # 4to3
        dst = [random.randrange(-150, -100, 8), random.choice([1.6, 4.8, 8.0, 11.2])] # 1to0
    elif routeID == 'route_h_3_2':
        src = [random.randrange(150, 200, 8), random.choice([251.6, 254.8, 258.0, 261.2])] # 4to3
        dst = [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] # 0to1
        
    elif routeID == 'route_h_4_1':
        src = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(400, 450, 8)] # 5to3
        dst = [random.randrange(-150, -100, 8), random.choice([1.6, 4.8, 8.0, 11.2])] # 1to0
    elif routeID == 'route_h_4_2':
        src = [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(400, 450, 8)] # 5to3
        dst = [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] # 0to1
        
    elif routeID == 'test':
        if auv:
            src = [-200.0, 245.2]
            dst = [-30.0, 242.0]
        elif not auv:
            src = [random.randrange(-165, -155, 16), random.choice([242.0, 248.4])]
            dst = [random.randrange(-50, -30, 8), random.choice([245.2])]

        
    depart = 0
    vel_init = vel_init * delta_T

    if 'huv' in vehID:
        depart = 2.5
        vel_init = 12.0

    _, departPos, departLane = traci.simulation.convertRoad(src[0], src[1])
    _, arrivalPos, arrivalLane = traci.simulation.convertRoad(dst[0], dst[1])

    traci.vehicle.add(vehID=vehID, 
                      typeID=typeID,                          
                      routeID=routeID,
                      depart=depart, 
                      departPos=departPos, 
                      departLane=departLane,
                      arrivalPos =arrivalPos,
                      arrivalLane = arrivalLane,
                      departSpeed=str(vel_init), 
                      arrivalSpeed='0')
    if auv == True:
        if vehID == 'auv1':
            traci.vehicle.setColor(vehID, (80,188,223))
        else:
            traci.vehicle.setColor(vehID, (255,255,255))
    else:
        traci.vehicle.setColor(vehID, (128,128,128))

    """ 0-D. Create global vehicle information in python """
    xpos, ypos, vel, angle, acc_max, dec_max, dec_min, length, width, height = Parser.vehicle_info(route_file, vehID)
    
    return vehID, xpos, ypos, src, dst, vel, angle, acc_max, dec_min, dec_max, response, length, width, height