#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import optparse

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
from sumolib import checkBinary  # Checks for the binary in environ vars
import traci
import numpy as np

import SimulControl
import Parser
import TLS

""" Global Variables """
numAUV = 1
numHUV = 10

global delta_T
delta_T = 0.1
vel_init = 12.0


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


def run():
    step = 0

    """ 0-A. Initial geographic information from netfile """
    net_file = '/Users/parks_d2nhiic/Desktop/BlaFT/blaft-main/blaft-main/intersection_net.net.xml'
    MapGraph, Connection = Parser.geo_info(net_file)
    
    """ 0-B. Route file to parse vehicle types """ 
    route_file = '/Users/parks_d2nhiic/Desktop/BlaFT/blaft-main/blaft-main/intersection_route.rou.xml'
    typelist = ['sedan']
    routelist = ['route1_5', 'route5_1', 'route2_5', 'route5_2']
    

    """ 0-C. Create vehicles (CAVs) in SUMO and Extract waypoints """
    vel_init_auv = vel_init * delta_T
    vel_init_huv = vel_init
    auvs, huvs = SimulControl.create_vehicles(numAUV, numHUV, route_file, typelist, routelist, MapGraph, Connection)
    
    
    
    """ 1) Disable SUMO's automation (lane changing + front-car following) / 2) Initial speed set-up """
    for i in range(numAUV):
        traci.vehicle.setSpeedMode(auvs[i].ID, 32)      # 32: All checks off / 31: All checks on
        traci.vehicle.setSpeed(auvs[i].ID, vel_init_auv)
        
    for i in range(numHUV):
        traci.vehicle.setSpeedMode(huvs[i].ID, 27)      # 32: All checks off / 31: All checks on
        traci.vehicle.setSpeed(huvs[i].ID, vel_init_huv)

    """ Create initial graph and waypoints before the loop """
    for i in range(numAUV):
        auvs[i].generate_waypoints()

    """ Simulation Loop """
    while traci.simulation.getMinExpectedNumber() > 0:
        step += 1
        traci.simulationStep()
        
        print("###################" + "Step: " + str(step * delta_T) + "####################")
        
        """ Update polygons """
        for polyID in traci.polygon.getIDList():
            traci.polygon.remove(polyID)
        
        """ Create and update Traffic Light Signal (TLS) information """
        TLSignals = TLS.create_and_update_TLS()
        
        """ 1-A. Check the surviving ones and the counts """ 
        vehID = traci.vehicle.getIDList()
        
        auvID = []
        huvID = []
        for ID in vehID:
            if 'auv' in ID:
                auvID.append(ID)
            elif 'huv' in ID:
                huvID.append(ID)
                
        auvCnt = len(auvID)

        """ 1-B. Update a list of vehicles alive """
        auv_alive = []
        huv_alive = []
        for i in range(auvCnt):
            if auvs[i].ID in auvID:
                auv_alive.append(auvs[i])
        
        # print('here: ', huvCnt, huvID, huvs)
        for i in range(len(huvs)):
            if huvs[i].ID in huvID:
                huv_alive.append(huvs[i])

        veh_alive = np.append(auv_alive, huv_alive)
        """ 1-C. Update current vehicle status (real-time values: xy-pos, vel, angle) """
        for auv in auv_alive: 
            auv.xpos, auv.ypos = traci.vehicle.getPosition(auv.ID)
            auv.vel = traci.vehicle.getSpeed(auv.ID)
            auv.angle = traci.vehicle.getAngle(auv.ID)
            
        # print('huvTest: ', huvCnt, huvID, huv_alive)
        for huv in huv_alive:
            huv.xpos, huv.ypos = traci.vehicle.getPosition(huv.ID)
            huv.vel = traci.vehicle.getSpeed(huv.ID)
            huv.angle = traci.vehicle.getAngle(huv.ID)
            
        """ Infinitive rerouting here! """
        for auv in auv_alive:
            auv.infinite_rerouting()
            
        """ 2. Behavior Decision """
        for auv in auv_alive:
            auv.update_waypoints()
            auv.generate_or_update_plan()
            auv.decide_behavior()
            
        """ 3. Sense, Plan, and Maneuver """
        for auv in auv_alive:
            """ Conservative Sensing """
            auv.conservative_sensing(veh_alive)
            TLSignal = auv.detect_TLS(TLSignals)
            
            """ Collision Detection """
            auv.detect_collision(TLSignal)
            
            """ Collision Avoidance and Mitigation """
            response = auv.avoid_or_mitigate_collision()
            auv.plan = response.plan

            """ Maneuver in Real-World """
            auv.maneuver(response)
            
            if auv.ID == 'auv1':
                auv.check_status()
            
    traci.close()
    sys.stdout.flush()


if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    sumoCmd = [sumoBinary, "-c", "intersection_config.sumocfg",
                             "--tripinfo-output", "my_tripinfo.xml",
                             "--collision.action", "warn",
                             "--step-length", str(delta_T)]
    
    traci.start(sumoCmd)

    run()
