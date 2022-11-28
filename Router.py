#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import traci
import math
import numpy as np
import sys
import networkx as nx
from SimpleMath import closestNode
from shapely.geometry import LineString, Point, MultiPoint

np.set_printoptions(threshold=sys.maxsize)
delta_T = 0.1
        
def generate_waypoint(src, dst, MapGraph, Connection):
    
    src_edge, _, _ = traci.simulation.convertRoad(src[0], src[1])
    dst_edge, _, _ = traci.simulation.convertRoad(dst[0], dst[1])
    
    interEdge = nx.shortest_path(Connection, src_edge, dst_edge)
    
    if not len(interEdge):
        return None, None, None
    
    RouteGraph = nx.DiGraph()
    for edge in interEdge:
        for node in MapGraph:
            if edge == MapGraph.nodes[node]['edge']:
                RouteGraph.add_node(int(node), 
                                    edge=MapGraph.nodes[node]['edge'], 
                                    lane=MapGraph.nodes[node]['lane'], 
                                    x=round(MapGraph.nodes[node]['x'],2), 
                                    y = round(MapGraph.nodes[node]['y'],2))
                    
    RouteGraph, interLane = addGraphEdge(RouteGraph, Connection)

    # Find the shortest path
    srcWP = closestNode(src, RouteGraph)
    dstWP = closestNode(dst, RouteGraph)
    WayPoint = nx.shortest_path(RouteGraph, srcWP, dstWP, weight='weight', method = 'dijkstra')
    
    return WayPoint, RouteGraph, interLane



def generate_trajectory(vehicle, MapGraph):
    if vehicle.__class__.__name__ == 'CAV' or vehicle.__class__.__name__ == 'AUV' :
        wps = vehicle.wp_ego
        xpos = vehicle.xpos
        ypos = vehicle.ypos
        vel = vehicle.vel / delta_T
        angle = vehicle.angle
        G = MapGraph

        
    tail = Point(round(xpos,1), round(ypos,1))
    _,pos_veh,_ = traci.simulation.convertRoad(xpos, ypos)

    # If refinement is done without communication
    if len(wps) == 0:
        head = Point(round(xpos + (3.0 * vel)*math.sin(math.radians(angle)),2), 
                        round(ypos + (3.0 * vel)*math.cos(math.radians(angle)),2))    # 3-second rule
        trajectory = LineString([tail, head])
        
    else:
        wp_temp = []
        wp_temp.append(Point(xpos, ypos))
        for wp in wps:
            _,pos_wp,_ = traci.simulation.convertRoad(G.nodes[wp]['x'], G.nodes[wp]['y'])
            
            if pos_wp > pos_veh: 
                wp_temp.append(Point(G.nodes[wp]['x'], G.nodes[wp]['y']))
        
        if len(wp_temp) > 1:
            trajectory = LineString(MultiPoint(wp_temp))
            return trajectory
        else:
            return None
        

def generate_crossing(traj_ego, traj_obj):
    pnt_crossing = traj_ego.intersection(traj_obj)

    if pnt_crossing.geom_type == 'MultiLineString':
        lines = list(pnt_crossing)
        pnt_crossing = Point(lines[0].coords[0])
    elif pnt_crossing.geom_type == 'GeometryCollection':
        lines = list(pnt_crossing)
        pnt_crossing = Point(lines[0].coords[0])
    elif len(list(pnt_crossing.coords)) >= 2:
        pnt_crossing = Point(pnt_crossing.coords[0])
        
    return pnt_crossing



def addGraphEdge(G, C):
    interLane = []
    for u in G:
        for v in G:
            if u == v:
                continue
            D = math.dist([G.nodes[u]['x'], G.nodes[u]['y']], [G.nodes[v]['x'], G.nodes[v]['y']])
            
            _, pos_u, _ = traci.simulation.convertRoad(G.nodes[u]['x'], G.nodes[u]['y'])
            _, pos_v, _ = traci.simulation.convertRoad(G.nodes[v]['x'], G.nodes[v]['y'])
            
            fromEdge = G.nodes[u]['edge']
            toEdge = G.nodes[v]['edge']
            
            fromLane = G.nodes[u]['lane']
            toLane = G.nodes[v]['lane']
            
            if fromEdge == toEdge:
                if fromLane == toLane and (u-v) == -1:
                    if ':n' in fromEdge:
                        G.add_edge(u, v, weight=D)
                    elif 'to' in fromEdge:
                        G.add_edge(u, v, weight=D)
                # If two vertex are in same edge but different lane and the lanes are adjacent to each other
                elif (abs(int(fromLane[-1]) - int(toLane[-1])) == 1) and 20 < D < 22 and pos_u < pos_v and 'to' in fromEdge:
                    if int(u)%3 == 1:
                        D = D * 2
                    if int(u)%3 == 2:
                        D = D * 3 
                    if (pos_u < 140 and pos_v < 140) or (pos_u > 40 or pos_v > 40):
                        G.add_edge(u, v, weight=D)
                            
            elif fromLane != toLane and C.has_edge(fromLane, toLane) and D < 14:
                G.add_edge(u, v, weight=D)
                if (fromLane, toLane) not in interLane and (toLane,fromLane) not in interLane:
                    interLane.append((fromLane,toLane))    
                    print('interlane: ', interLane)
            else:
                continue
    
    return G, interLane


    
    
    
def findInterEdge(src, dst, C):
    src_edge,_,_ = traci.simulation.convertRoad(src[0], src[1])
    dst_edge,_,_ = traci.simulation.convertRoad(dst[0], dst[1])
    interEdge = nx.shortest_path(C, src_edge, dst_edge)
    
    return interEdge
        

    
def extractPoints(G, edgeID, laneID, pntID, shape, spacing=20):
    """"Return a list of nb_points equally spaced points
    between p1 and p2"""
    # If we have 8 intermediate points, we have 8+1=9 spaces
    # between p1 and p2
    
    if 'to' in edgeID:
        nb_points = int(math.dist(shape[0], shape[1]) / spacing)
        x_spacing = (shape[1][0] - shape[0][0]) / (nb_points)
        y_spacing = (shape[1][1] - shape[0][1]) / (nb_points)
        
        nextID = pntID
        for i in range(0, nb_points+1):
            G.add_node(int(pntID+i), edge=str(edgeID), lane=str(laneID), x=float(shape[0][0] + i * x_spacing), y=float(shape[0][1]+ i * y_spacing))
            nextID += 1
            
        return G, nextID
    
    elif ':n' in edgeID:
        nextID = pntID
        for i in range(len(shape)):
            G.add_node(int(pntID+i), edge=str(edgeID), lane=str(laneID), x=shape[i][0], y=shape[i][1])
            nextID += 1
        
        return G, nextID