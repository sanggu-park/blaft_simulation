#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 20 15:29:06 2021

@author: sanggupark
"""
import math
from SimpleMath import calc_clockwise_angle
from shapely.geometry import Point, LineString
import traci

from numpy import cos, sin, arccos
import numpy as np
from scipy import interpolate


delta_T = 0.1
vel_max = 12.0

def move2XY(vehicle, plan):    

    vel = vehicle.vel
    xpos = vehicle.xpos
    ypos = vehicle.ypos
    angle = vehicle.angle
    G = vehicle.MapGraph
    wp_current = vehicle.wp_ego
    
    xpos_next = xpos
    ypos_next = ypos
    angle_next = angle
    
    if plan == None or plan == 1:
        return xpos_next, ypos_next, angle_next
    
    if len(wp_current):
        wp_next = Point(G.nodes[wp_current[0]]['x'], G.nodes[wp_current[0]]['y'])
    else:
        return xpos_next, ypos_next, angle_next
    
    D_init = math.dist(plan[0], [xpos, ypos])
    D_driving = vel
    
    D_driven = D_init
    for i in range(len(plan)-1):
        D_interval = math.dist(plan[i], plan[i+1])
        if D_driving <=0:
            break
        elif D_driving < D_interval and D_driving > 0:
            xpos_next = xpos_next + (plan[i+1][0] - plan[i][0]) * (D_driving / D_interval)
            ypos_next = ypos_next + (plan[i+1][1] - plan[i][1]) * (D_driving / D_interval)
            D_driving -= D_interval
            D_driven += D_interval
            break
        elif D_driving >= D_interval:
            xpos_next = plan[i+1][0]
            ypos_next = plan[i+1][1]

            D_driving -= D_interval
            D_driven += D_interval
            continue
    
    
    
    angle_next = calc_clockwise_angle([0,100], [wp_next.x-xpos, wp_next.y-ypos])
        
    # print('[Motion Controller]')
    # print('<vehicle>: ', vehicle.ID)
    # print('<velocity>: ', vehicle.vel / delta_T)
    # print('<Plan>: ', plan)
    # print('<x_prev and y_prev>: ', xpos, ypos)
    # print('<x_next and y_next and theta_next>: ', xpos_next, ypos_next, angle_next)
        
    return xpos_next, ypos_next, angle_next


def translate_reaction_into_state(ego, response):
    vel = ego.vel + response.a * delta_T
    theta, dst, _ = convert_plan_to_w_R(ego, response.plan, response.a)
    x = dst.x
    y = dst.y

    return x, y, vel, theta


def convert_plan_to_w_R(ego, plan, a_prime=0.0):
    
    edge_moved, pos_moved, lane_moved = traci.simulation.convertRoad(ego.xpos, ego.ypos)
    laneID = str(edge_moved) + '_' + str(lane_moved)
    shape_lane = traci.lane.getShape(laneID)
    vec1 = [shape_lane[1][0]-shape_lane[0][0], shape_lane[1][1]-shape_lane[0][1]]
    
    xpos = ego.xpos
    ypos = ego.ypos
    length = ego.length
    width = ego.width
    rad = math.radians(ego.angle)
    
    p_tail = Point([xpos-(length)*math.sin(rad), ypos-(length)*math.cos(rad)])

    linestring_plan = LineString(plan)
    linestring_tail = LineString(np.insert(plan, 0, [p_tail.x, p_tail.y], axis=0))
    
    # plan = LineString(plan)
    # plan = linestring_steer
    vel = max(ego.vel / delta_T + a_prime, 0)
    
    D_move = vel * delta_T 
    pnt_head = linestring_plan.interpolate(D_move)
    print('x and y and vel now: ', xpos, ypos, vel)
    # print('plan: ', list(linestring_tail.coords)[:10])
    print('D_move, pnt_head: ', D_move, D_move+length, pnt_head)
    
    print('hihi#1')
    pnt_tail = linestring_tail.interpolate(D_move)

    vec0 = [0,99999]
    vec1 = [pnt_head.x - pnt_tail.x, pnt_head.y - pnt_tail.y]
    theta = np.deg2rad(calc_clockwise_angle(vec0, vec1))
    
    if D_move == 0.0 or linestring_plan.length == 0.0:
        theta = np.deg2rad(ego.angle)
    
    # print('vector test: ', vec0, vec1, theta)
    R = LineString(plan).length / abs(theta)
    dst = pnt_head
    
    return theta, dst, R



def convert_waypoints_to_plan(ego, wps=None, N=3, ay_comfort=1.0, tb=3.0):
    G = ego.MapGraph
    if wps == None:
        wps = ego.wp_ego
    # print("####################### CONVERSION CHECK ###########################")
    # print(ego.ID, 'WPS: ', wps)
    
    edge_now, pos_now, lane_now = traci.simulation.convertRoad(ego.xpos, ego.ypos)
    # pnt_veh = [ego.xpos, ego.ypos]
    # coords = [pnt_veh]
    
    def linear_interpolation(x, y, resolution):
        param = np.linspace(0, 1, len(x))
        spl = interpolate.make_interp_spline(param, np.c_[x,y], k=2) #(1)

        # spl = interpolate.make_interp_spline(param, np.c_[x,y], k=2) #(1)
        out_x, out_y = spl(np.linspace(0, 1, int(round(len(x) * 1.5)))).T #(2)
        return out_x, out_y

    x_wps = []
    y_wps = []    
    for i, wp in enumerate(wps):
        if i == len(wps)-1:
            break
        if Point([G.nodes[wps[i+1]]['x'], G.nodes[wps[i+1]]['y']]).distance(Point([G.nodes[wps[i]]['x'], G.nodes[wps[i]]['y']])) > 1.0:
            x_wps.append(G.nodes[wp]['x'])
            y_wps.append(G.nodes[wp]['y'])
    
    # print('wps len: ', len(x_wps), len(y_wps))
    # print('x_wps: ', x_wps)
    x_plan, y_plan = linear_interpolation(x_wps, y_wps, N*len(x_wps))
    plan = np.column_stack((x_plan, y_plan))
    # print('I just fixed the plan 2 : ', plan)
    
    return plan



def calculate_rotation_info(vehicle, start_point, end_point, is_right_turn, ay_comfort=1.0):
    
    _, _, lane_start = traci.simulation.convertRoad(start_point.x, start_point.y)
    # _, _, lane_end = traci.simulation.convertRoad(end_point.x, end_point.y)

    # print('se#2: ', start_point, end_point)
    vec_std = [end_point.x - start_point.x, end_point.y - start_point.y]
    # vec_map = [0,99999]
    
    R = (vel_max**2) / ay_comfort
    # print('R:', R)
    circle_1 = LineString(list(start_point.buffer(R).exterior.coords))
    circle_2 = LineString(list(end_point.buffer(R).exterior.coords))
    
    """ Must check here.. maybe the direction matters!"""
    # print('circle_1: ', list(start_point.buffer(R).exterior.coords))
    # print('circle_2: ', list(end_point.buffer(R).exterior.coords))
    # print('circle_3: ', circle_1.intersection(circle_2))


    candidate1, candidate2 = circle_1.intersection(circle_2)
    # candidate2 = candidate1[1]    
    # print('candidate check: ', candidate1, candidate2)
    
    vec1 = [candidate1.x - start_point.x, candidate1.y - start_point.y]
    vec2 = [candidate2.x - start_point.x, candidate2.y - start_point.y]

    # print('vector test: ', vec1, vec2)
    
    angle1 = calc_clockwise_angle(vec_std, vec1)
    angle2 = calc_clockwise_angle(vec_std, vec2)
    
    # print('angle test: ', angle1, angle2)
    
    """ Let's Fuck this up!!! (16:30 MAR 20)"""
    if is_right_turn:
        if angle1 < 0 and angle2 > 0:
            candidate = candidate2
        elif angle1 > 0 and angle2 < 0:
            candidate = candidate1
    elif not is_right_turn:
        if angle1 < 0 and angle2 > 0:
            candidate = candidate1
        elif angle1 > 0 and angle2 < 0:
            candidate = candidate2
        
        
    xc = candidate.x
    yc = candidate.y
    
    pnt_center = Point(xc, yc)
    
    # print('Decided pnt_center: ', pnt_center.x, pnt_center.y)
    return pnt_center, R



def parametric_circle(t, xc, yc, R, is_right_turn):
    x = xc + R*cos(t)
    y = yc + R*sin(t)

    if not is_right_turn:
        y = yc + R*sin(-t)

    return x,y


def inv_parametric_circle(x, xc, R):
    t = arccos((x-xc)/R)

    return t
    
