#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 15 10:35:17 2022

@author: sanggupark
"""
import math
import numpy as np
import traci
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import split

from SimpleMath import create_lane_shape, create_vehicle_shape, calc_clockwise_angle, cut_linestring, visualize_polygon, project_shape_over_centerline
from dataclasses import dataclass
from numpy import cos, sin


delta_T = float(0.1)
tlsID = 'n3'
lanes_left_turn = [':n3_4_0', ':n3_9_0', ':n3_14_0', ':n3_19_0']
lanes_right_turn = [':n3_0_0', ':n3_5_0', ':n3_10_0', ':n3_15_0']
egoID = 'auv1'

@dataclass(init = True)
class Collision:
    ID: str = None
    area: Polygon() = None
    ts: float = np.inf



def search_LE_overlap(ego):

    # visualize_polygon(self, list(self.ae.exterior.coords), 'AE')
    edge_ego, _, lane_ego = traci.simulation.convertRoad(ego.xpos, ego.ypos)
    lane_cnt = traci.edge.getLaneNumber(edge_ego)
    
    lane_target = int(lane_ego) + int(ego.blinker)
    laneID_target = str(edge_ego) + '_' + str(lane_target)
    
    poly_edge = Polygon()
    for i in range(0,lane_cnt):
        laneID_i = str(edge_ego) + '_' + str(i)
        lane_i_shape = Polygon(create_lane_shape(laneID_i))
        poly_edge = poly_edge.union(lane_i_shape)

    LE_ego, t_enter = create_LE(ego, laneID=laneID_target)
    ego.le = LE_ego


    collision = Collision()
    for obj in ego.objects_sensed:
        # edge_obj, pos_obj, lane_obj = traci.simulation.convertRoad(obj.xpos, obj.ypos)
        LE_obj = None
        # if not (pos_obj < pos_ego and lane_obj == lane_ego):
        LE_obj, _ = create_LE_other(obj, laneID_target, t_enter)
        print('ee_obj is here: ', LE_obj)
        if LE_obj != None:
            print('crazy: ', list(LE_obj.exterior.coords))
            visualize_polygon(ego, obj, list(LE_obj.exterior.coords), 'EE')
            
        if LE_obj != None and LE_obj.intersects(LE_ego):
            collision.ID = 'EE'
            collision.area = LE_ego
            collision.ts = traci.simulation.getTime()
            break

    if collision.ID == None:
        return
    else:
        return collision


def search_RE_overlap(ego):
    de_ego = ego.de
    poly_ego = create_vehicle_shape(ego)
    # visualize_polygon(self, list(self.ae.exterior.coords), 'AE')
    
    edge_ego, pos_ego, lane_ego = traci.simulation.convertRoad(ego.xpos, ego.ypos)
    lane_cnt = traci.edge.getLaneNumber(edge_ego)

    laneID = str(edge_ego) + '_' + str(lane_ego)
    laneID_R = None
    laneID_L = None
    
    if lane_ego > 0:
        laneID_R = str(edge_ego) + '_' + str(lane_ego-1)
    if lane_ego < lane_cnt - 1:
        laneID_L = str(edge_ego) + '_' + str(lane_ego+1)
        
        
    InterLane = np.array(ego.InterLane)
    
    # print('laneID: ', laneID)
    laneID_F = None
    for edge in InterLane:
        # print('EDGE TEST: ', edge, edge[0], edge[1])
        if edge[0] == laneID:
            laneID_F = edge[1]
            break
        
    poly_lane = Polygon(create_lane_shape(laneID))
    poly_Rlane = Polygon()
    poly_Llane = Polygon()
    poly_Flane = Polygon()
    
    if laneID_R != None:
        poly_Rlane = Polygon(create_lane_shape(laneID_R))
    if laneID_L != None:
        poly_Llane = Polygon(create_lane_shape(laneID_L))
    if laneID_F != None:
        poly_Flane = Polygon(create_lane_shape(laneID_F))
            
    poly_lane_entire = poly_lane.union(poly_Rlane).union(poly_Llane).union(poly_Flane)
            
    
    collision = Collision()
    D = np.inf
    for obj in ego.objects_sensed:
        edge_obj, pos_obj, lane_obj = traci.simulation.convertRoad(obj.xpos, obj.ypos)
        poly_obj = create_vehicle_shape(obj)
        
        is_worth_cautious = False 
        if poly_obj.intersects(poly_lane_entire) and not poly_obj.intersects(poly_lane):
            is_worth_cautious = True
        elif poly_obj.intersects(poly_lane) and pos_obj > pos_ego:
            is_worth_cautious = True
        
        # print(obj.ID, 'is_worth_cautious: ', is_worth_cautious)
        if is_worth_cautious:
            de_obj, D_obj = create_RE_other(ego, obj, ego.Connection)
            
            visualize_polygon(ego, obj, list(de_obj.exterior.coords), 'DE')

            # print('de_ego: ', de_ego)
            # print('de_obj: ', de_obj)
            
            A_collision = de_ego.intersection(de_obj)
            is_overlapping = de_ego.intersects(de_obj)
            is_dangerous = is_overlapping and A_collision.intersects(poly_lane)

            # print('is_dangerous: ', is_dangerous)
            if is_dangerous:
                A_collision = de_ego.intersection(de_obj)
                D_collision = poly_ego.distance(A_collision)
                # print('D_collision: ', D_collision)
                if D_collision < D:
                    collision.ID = obj.ID
                    collision.area = de_obj.intersection(de_ego)
                    collision.ts = traci.simulation.getTime()
                    D = D_collision
                
    if collision.ID == None:
        # if ego.ID == 'auv1': # or 'auv2':
        #     print(ego.ID, ' has No LF collision!')
        return
    else:
        # print(ego.ID, 'has LF collision: ', collision.ID, list(collision.area.exterior.coords))
        if 'huv' in collision.ID or 'auv' in collision.ID:
            try:
                visualize_polygon(ego, collision, list(collision.area.exterior.coords), 'DANGER')
            except:
                print('could not visualize')
                
        return collision


    
def create_RE(ego, plan_prime=[], acc_prime=None, ay_max=3.0):
    xpos = ego.xpos
    ypos = ego.ypos
    angle = ego.angle
    rad = math.radians(angle)
    acc = ego.acc_max/delta_T
    width = ego.width
    length = ego.length
    
    if not acc_prime:
        vel = ego.vel / delta_T
    else:
        vel = ego.vel / delta_T + acc_prime * delta_T
        
    dec_min = ego.dec_min / delta_T
    rho = ego.response
    
    p_rr = Point([xpos-(length)*math.sin(rad)+(width/2)*math.cos(rad), 
                  ypos-(length)*math.cos(rad)-(width/2)*math.sin(rad)])
    p_lr = Point([xpos-(length)*math.sin(rad)-(width/2)*math.cos(rad), 
                  ypos-(length)*math.cos(rad)+(width/2)*math.sin(rad)])

    plan = np.insert(ego.plan, 0, [(p_lr.x + p_rr.x) / 2, (p_lr.y + p_rr.y) / 2], axis=0)
    if len(plan_prime):
        plan = np.insert(plan_prime, 0, [(p_lr.x + p_rr.x) / 2, (p_lr.y + p_rr.y) / 2], axis=0)

    """ envelope length calculation """
    D_response = max((vel*rho) + (acc*rho**2)/2 + ((vel+rho*acc)**2)/(2*dec_min), 0)
    linestring_danger = cut_linestring(LineString(plan), D_response+length)[0]

    RE = linestring_danger.buffer(width/2, cap_style=2, single_sided=False)
    
    return RE



def create_CE(ego, plan_prime=[], acc_prime=None, ay_max=3.0):
    xpos = ego.xpos
    ypos = ego.ypos
    angle = ego.angle
    rad = math.radians(angle)
    acc = ego.acc_max/delta_T
    width = ego.width
    length = ego.length

    if not acc_prime:
        vel = ego.vel / delta_T
    else:
        vel = ego.vel / delta_T + acc_prime * delta_T
        
    dec_max = ego.dec_max / delta_T
    rho = ego.response
    
    p_rr = Point([xpos-(length)*math.sin(rad)+(width/2)*math.cos(rad), 
                  ypos-(length)*math.cos(rad)-(width/2)*math.sin(rad)])
    p_lr = Point([xpos-(length)*math.sin(rad)-(width/2)*math.cos(rad), 
                  ypos-(length)*math.cos(rad)+(width/2)*math.sin(rad)])


    plan = np.insert(ego.plan, 0, [(p_lr.x + p_rr.x) / 2, (p_lr.y + p_rr.y) / 2], axis=0)
    if len(plan_prime):
        plan = np.insert(plan_prime, 0, [(p_lr.x + p_rr.x) / 2, (p_lr.y + p_rr.y) / 2], axis=0)

    """ envelope length calculation """
    D_crash = max((vel*rho) + (acc*rho**2)/2 + ((vel+rho*acc)**2)/(2*dec_max), 0)
    
    linestring_accident = cut_linestring(LineString(plan), D_crash+length)[0]
    CE = linestring_accident.buffer(width/2, cap_style=2, single_sided=False)

    return CE



def create_LE(ego, plan_prime=[], acc_prime=None, laneID=None, ay_max=3.0):
    def merge_lines(lines):
        lines = list(lines)
        last = None
        points = []
        for line in lines:
            current = line.coords[0]
    
            if last is None:
                points.extend(line.coords)
            else:
                if last == current or Point(last).almost_equals(Point(current),0):
                    points.extend(line.coords[1:])
                else:
                    print('Skipping to merge {} {}'.format(last, current))
                    # return None
            last = line.coords[-1]
        return LineString(points)

    # print('laneID_target: ', laneID)
    
    ee = Polygon()
    t_enter = 0.0
    acc = ego.acc_max
    width = ego.width
    if not acc_prime:
        vel = ego.vel / delta_T
    else:
        vel = ego.vel / delta_T + acc_prime
        
    dec_min = ego.dec_min/delta_T
    rho = ego.response
    
    # print('laneID_target: ', laneID)
    edge_now, pos_now, lane_now = traci.simulation.convertRoad(ego.xpos, ego.ypos)
    
    poly_lane_now = Polygon(create_lane_shape(edge_now + '_' + str(lane_now), width=3.2))
    poly_lane_target = Polygon(create_lane_shape(laneID, width=3.2))
    # print('poly_lane_target: ', poly_lane_target)
    if not len(plan_prime):
        plan = ego.plan
    else:
        plan = plan_prime
    

    D_response = max((vel*rho) + (acc*rho**2)/2 + ((vel+rho*acc)**2)/(2*dec_min), 0)
    # line = LineString(plan).intersection(poly_lane_target)
    
    line_in_current = LineString(plan).intersection(poly_lane_now)
    line_in_target = LineString(plan).intersection(poly_lane_target)
    line_in_target = cut_linestring(line_in_target, D_response)[0]
    
    line = line_in_current.union(line_in_target)
    
    if line.geom_type == 'MultiLineString':
        line = merge_lines(line)
    
        
    LE = line.buffer(width/2, cap_style=2, single_sided=False).intersection(poly_lane_target)
        
    if vel != 0:
        t_enter = line_in_current.length / vel
    elif vel < 0.1:
        t_enter = delta_T
        
    # print('t_enter: ', t_enter)
    return LE, t_enter
        


def create_RE_other(ego, other, C, N=5, ay_max=3.0):
    # print('create de_other!')
    xpos = other.xpos
    ypos = other.ypos
    angle = other.angle
    rad = math.radians(angle)
    vel = other.vel / delta_T
    width = other.width
    length = other.length
    acc_max = other.acc_max
    dec_min = other.dec_min
    rho = other.response
    blinker = other.blinker
    if 'huv' in other.ID:
        vel = other.vel
        
    
    edge_moved, pos_moved, lane_moved = traci.simulation.convertRoad(xpos, ypos)
    laneID = str(edge_moved) + '_' + str(lane_moved)

    is_entering_node = False
    if pos_moved > traci.lane.getLength(laneID) * 0.90:
        is_entering_node = True

    """ envelope length calculation """
    D_response = max((vel*rho) + (acc_max*rho**2)/2 + ((vel+rho*acc_max)**2)/(2*dec_min), 0)
    
    R = (vel**2) / ay_max
    # print('R is: ', R)
    if R <= 0.1:
        R = 99999
    # Rmin = max(R*cos(10) - width/2, 0)
    # Rmax = np.sqrt((Rmin+width)**2 + (length)**2)
    # print('RMAX / RMIN: ', Rmax, Rmin)

    p_tail = Point([xpos-(length)*math.sin(rad), ypos-(length)*math.cos(rad)])
    X_f = xpos
    Y_f = ypos
    
    if blinker != 0 and not is_entering_node:
        
        fwheel_rad = 0.0
        delta_rad = 0.0
        
        
        if blinker == -2:
            fwheel_rad = -math.radians(5)
            delta_rad = - (D_response / R)
        elif blinker == -1:
            fwheel_rad = math.radians(5)
            delta_rad = D_response / R
        
        """ Steering """
        # plan_other = [[xpos, ypos]]
        rear_rad = math.radians(angle)
        # print('blinker / fwheel_rad: ', blinker, fwheel_rad)
        front_rad = rear_rad + fwheel_rad
        arc_front_rad = np.linspace(front_rad, front_rad + delta_rad, N)
        # vel_inner = np.sqrt(Rmin/R) * vel
        D_seg = D_response / N

        coords_plan = [[p_tail.x, p_tail.y], [X_f, Y_f]]
            
        for rad in arc_front_rad:
            X_f = X_f + D_seg * sin(rad)
            Y_f = Y_f + D_seg * cos(rad)
            pnt = [X_f, Y_f]
            coords_plan = np.vstack([coords_plan, pnt])
            
        RE = LineString(coords_plan).buffer(width/2, cap_style=2, single_sided=False)
        # de_right = LineString(coords_plan).buffer(+width/2, single_sided=True)
        # de = de_left.union(de_right)
        
        # print('de shape is: ', list(de.exterior.coords))

        return RE, D_response

    elif blinker == 0 or is_entering_node:
        """ Steering but Straight-move"""
        shape_lane = traci.lane.getShape(laneID)
        
        laneID_next = None
        for (u,v) in C.edges:
            if u == laneID:
                # print('u,v:', u, v)
                laneID_next = v
                break
        
        if laneID_next != None:
            shape_next = traci.lane.getShape(laneID_next)
            shape_base = np.vstack([shape_lane, shape_next])
        else:
            shape_base = shape_lane
        # print('shape_base: ', shape_base)
        
        shape_base = np.array(shape_base[1:])
        shape_plan = np.insert(shape_base, 0, [xpos, ypos], axis=0)
        shape_plan = np.insert(shape_base, 0, [p_tail.x, p_tail.y], axis=0)
                
        linestring_plan = LineString(shape_plan)
        linestring_response = cut_linestring(linestring_plan, length+D_response)[0]
        
        RE = linestring_response.buffer(width/2, cap_style=2, single_sided=False)
        
        visualize_polygon(ego, other, list(RE.exterior.coords), 'DE')
        # print('de shape is: ', list(de.exterior.coords))

        return RE, D_response


    
def create_LE_other(other, target_lane, t_enter=0.0, N=3, ay_max=3.0):
    def merge_lines(lines):
        last = None
        points = []
        for line in lines:
            current = line.coords[0]
    
            if last is None:
                points.extend(line.coords)
            else:
                if last == current:
                    points.extend(line.coords[1:])
                else:
                    print('Skipping to merge {} {}'.format(last, current))
                    return None
            last = line.coords[-1]
        return LineString(points)

    if t_enter == None:
        t_enter = 0.0
    
    xpos = other.xpos
    ypos = other.ypos
    angle = other.angle
    vel = other.vel
    length = other.length
    acc_max = other.acc_max
    dec_min = other.dec_min
    rho = other.response + t_enter
    
    rad = np.radians(angle)
    p_tail = Point([xpos-(length)*math.sin(rad), ypos-(length)*math.cos(rad)])

    
    edge_moved, pos_moved, lane_moved = traci.simulation.convertRoad(xpos, ypos)
    laneID = str(edge_moved) + '_' + str(lane_moved)
    shape_lane = traci.lane.getShape(laneID)
    vector_lane = [shape_lane[1][0]-shape_lane[0][0], shape_lane[1][1]-shape_lane[0][1]]
    angle_lane = calc_clockwise_angle([0,99999], vector_lane)
    rad_lane = math.radians(angle_lane)
    
    rad_other = math.radians(angle)
    print('angle and rad_other: ', rad_lane, rad_other)
    print('t_enter: ', t_enter)

    print('target? ', target_lane)
    linestring_target = LineString(traci.lane.getShape(target_lane))
    polygon_target = linestring_target.buffer(1.6, cap_style=2, single_sided=False)
    D_response = max(((vel*rho) + (acc_max*rho**2))/2 + ((vel+rho*acc_max)**2)/(2*dec_min), 0)
    
    R = ((vel + acc_max*t_enter)**2) / ay_max
    
    if R < 0.1:
        delta_rad = 0.0 
    else:
        delta_rad = D_response / R
        
    rad_steer = rad_other - rad_lane
    print('rad_steer: ', rad_steer)
    if rad_steer % np.pi > 0.1:
        delta_rad = + delta_rad
    elif rad_steer % np.pi  < -0.1:
        delta_rad = - delta_rad
    else:
        delta_rad = 0.0
    
    coords = [[p_tail.x, p_tail.y], [xpos, ypos]]
    start_rad = rad_other
    end_rad = start_rad + delta_rad
    arc_rad = np.linspace(start_rad, end_rad, N)
    D_seg = D_response / N
    X = xpos
    Y = ypos
    for rad in arc_rad:
        X = X + D_seg * sin(rad)
        Y = Y + D_seg * cos(rad)
        pnt = [X, Y]
        coords = np.row_stack([coords, pnt])

    linestring_plan = LineString(coords)
    if linestring_plan.intersects(polygon_target):
        print(other.ID, 'is intersecting!!!')
        print(list(linestring_plan.coords))
        print(list(polygon_target.exterior.coords))


        ### HERE WE FIX! ###
        linestrings_splitted = split(linestring_plan, polygon_target)
        
        linestring_plan = LineString([])
        for splitted in list(linestrings_splitted):
            if splitted.intersects(polygon_target):
                # print('hahaha', splitted)
                linestring_plan = linestring_plan.union(splitted)
        
        if linestring_plan.geom_type == 'MultiLineString':
            linestring_plan = merge_lines(linestring_plan)
            
        print('here: ', linestring_plan.geom_type)
        # print('ls test #2', list(linestring_plan.coords))
        linestring_plan = project_shape_over_centerline(linestring_plan, linestring_target)
        print('linestring_plan: ', linestring_plan)
        
        if linestring_plan == None or len(list(linestring_plan.coords)) == 0 or linestring_plan.length == 0.0:
            return None, None
        
        de_prime = linestring_plan.buffer(1.6, cap_style=2, single_sided=False)
        print('de_prime, D_response: ', de_prime, D_response)
        return de_prime, D_response
    else:
        return None, None


def create_evasive_DE(ego, plan, acc, ay_max=3.0):
    xpos = ego.xpos
    ypos = ego.ypos
    angle = ego.angle
    rad = math.radians(angle)
    vel = ego.vel / delta_T + acc
    width = ego.width
    length = ego.length
    acc_max = ego.acc_max/delta_T
    dec_min = ego.dec_min/delta_T
    rho = ego.response
    
    """ envelope length calculation """        
    p_tail = Point([xpos-(length)*math.sin(rad), ypos-(length)*math.cos(rad)])
    plan = np.insert(ego.plan, 0, [p_tail.x, p_tail.y], axis=0)

    """ envelope length calculation """
    D_response = max((vel*rho) + (acc_max*rho**2)/2 + ((vel+rho*acc_max)**2)/(2*dec_min), 0)
    linestring_response = cut_linestring(LineString(plan), D_response+length)[0]
    # de_left = linestring_response.buffer(width/2, single_sided=True)
    # de_right = linestring_response.buffer(-width/2, single_sided=True)
    de = linestring_response.buffer(width/2, cap_style=2, single_sided=False)
        
    return de



# def create_evasive_TE(ego, plan, acc, poly_lane=None, ay_max=3.0):
#     # print('TE check: ', ego, plan, acc)
#     te = Polygon()
    
#     def update_plan(plan, pnt_future):
#         if not len(plan):
#             return None
#         else:
#             pnt_erase = np.ndarray((0,2))
#             edge_veh, pos_veh, _ = traci.simulation.convertRoad(pnt_future.x, pnt_future.y)

#             for i, pnt in enumerate(plan):
#                 edge_pnt, pos_pnt, _ = traci.simulation.convertRoad(pnt[0], pnt[1])   
#                 if edge_veh == edge_pnt and pos_pnt < pos_veh:
#                     pnt_erase = np.row_stack([pnt_erase, pnt])
              
#             for pnt in pnt_erase:
#                 row = np.where((plan==pnt).all(axis=1))
#                 plan = np.delete(plan, row, axis=0)
                
#             plan = np.insert(plan, 0, [pnt_future.x, pnt_future.y], axis=0)
#         return plan
    
#     xpos = ego.xpos
#     ypos = ego.ypos
#     angle = ego.angle
#     vel = ego.vel / delta_T + acc
#     width = ego.width
#     length = ego.length
#     acc_max = ego.acc_max/delta_T
#     dec_min = ego.dec_min/delta_T
#     rho = ego.response
    
#     line_plan = cut_linestring(LineString(plan), 500)[0]
#     # print('line_plan_te_other: ', line_plan)
    
#     D_limit = ego.sensor.radius
#     D_response = max((vel*rho) + (acc_max*rho**2)/2 + ((vel+rho*acc_max)**2)/(2*dec_min), 0)
#     # print('D_response: ', D_response)

#     D_list = np.linspace(0, D_limit, 30)
    
        
#     if poly_lane == None:
#         edge_moved, pos_moved, lane_moved = traci.simulation.convertRoad(ego.xpos, ego.ypos)
#         lane_target = int(lane_moved) + int(ego.blinker)
#         laneID = str(edge_moved) + '_' + str(lane_target)
#         poly_lane = Polygon(create_lane_shape(laneID))
    
#     for D in D_list:
#         pos = line_plan.interpolate(D)
#         # print('pos is: ', pos)
#         xpos = pos.x
#         ypos = pos.y
#         rad, _, _ = Control.convert_plan_to_w_R(ego, ego.plan)
#         angle = np.rad2deg(rad)
        
#         poly_ego = create_future_vehicle_shape(xpos, ypos, angle, width, length)
#         # print('poly_future_ego: ', poly_ego)
        
#         is_fully_included = False
#         if poly_ego.intersection(poly_lane).area == poly_ego.area:
#             is_fully_included = True
        
#         if is_fully_included:
#             pnt_future = Point([xpos, ypos])
#             # print('pnt_future_other_te: ', pnt_future)
#             plan_future = update_plan(ego.plan, pnt_future)
#             # print('pnt_future_other_te: ', plan_future)
            
#             line_plan = LineString(plan_future)
#             line_center = cut_linestring(line_plan, D_response)[0]
#             # print('line_future_other_te: ', list(line_center.coords))
            
#             # te_left = line_center.buffer(width/2, single_sided=True)
#             # te_right = line_center.buffer(-width/2, single_sided=True)
#             te = line_center.buffer(width/2, cap_style=2, single_sided=False)
#             break
            
#     return te


