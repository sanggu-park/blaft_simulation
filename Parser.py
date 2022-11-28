#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
import numpy as np
import traci
import networkx as nx
from Router import extractPoints

def geo_info(net_file):
    """net_file contains edge, node, and lane info"""
    """This method transforms SUMO's geographic information to Graph (nodes and edges)."""
    tree = ET.parse(net_file)
    root = tree.getroot()
    
    C = nx.DiGraph()  # SUMO Road Connection Information
    G = nx.DiGraph()  # Map Location Information

    pntID = int(0)
    
    for conn in root.findall('connection'):
        fromLane = conn.attrib['from'] + '_' + conn.attrib['fromLane']
        if ":n" in fromLane:
            toLane = conn.attrib['to'] + '_' + conn.attrib['toLane']
        elif "to" in fromLane:
            toLane = conn.attrib['via']
        
        C.add_edge(fromLane, toLane)
            
    for conn in root.findall('connection'):
        fromEdge = conn.attrib['from']
        if ":n" in fromEdge:
            toEdge = conn.attrib['to']
        elif "to" in fromEdge:
            toEdge = conn.attrib['via'][:-2]
        C.add_edge(fromEdge, toEdge)


    for edge in root.findall('edge'):
        fromEdge = edge.attrib['id']
        # print('fromEdge_test: ', fromEdge)
        for lane in edge.findall('lane'):
            laneID = lane.attrib['id']
            shape = lane.attrib['shape']
            shape_split = shape.split(' ')
            
            for k in range(len(shape_split)):
                shape_split[k] = shape_split[k].split(',')
                
            shape = np.array(shape_split, dtype=float)
            G, pntID = extractPoints(G, fromEdge, laneID, pntID, shape)

    return G, C


def vehicle_info(routefile, vehID):
    """This method returns one vehicle instance"""
    tree = ET.parse(routefile)
    root = tree.getroot()
    
    xpos, ypos = traci.vehicle.getPosition(vehID)
    vel = traci.vehicle.getSpeed(vehID)
    angle = traci.vehicle.getAngle(vehID)
    
    # Put Values In
    for vtype in root.findall('vType'):
        # if veh.attrib['type'] == vtype.attrib['id']:
        acc_max = vtype.attrib['accel']  # accel
        dec_min = vtype.attrib['decel']  # decel
        dec_max = vtype.attrib['emergencyDecel']  # emergencyDecel
        length = vtype.attrib['length']
        width = vtype.attrib['width']
        height = vtype.attrib['height']
                    
    return xpos, ypos, vel, angle, acc_max, dec_min, dec_max, length, width, height


