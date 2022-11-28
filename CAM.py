#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 16 10:36:44 2022

@author: sanggupark
"""
import networkx as nx
from shapely.geometry import LineString, Point
import traci
import numpy as np
from SimpleMath import closestNode, cut_linestring
from dataclasses import dataclass
from Control import convert_waypoints_to_plan
import Detection

delta_T = 0.1
vel_max = 12.0


@dataclass(init=True)
class Response:
    plan: np.ndarray = np.empty((0, 1))
    a: float = 0.0
    # k: float = 0.0


def extract_alternative_plans(ego):
    G = ego.RouteGraph.copy()
    wps = ego.wp_ego
    collision = ego.collision
    plans_prime = []

    if collision == None or 'uv' not in collision.ID or 'LC' in ego.behavior:
        return []
    if ego.de.intersects(collision.area):
        return []
    if ego.vel < 0.1:
        return []

    edge_ego, _, lane_ego = traci.simulation.convertRoad(
        collision.area.centroid.x, collision.area.centroid.y)
    laneID_ego = str(edge_ego) + '_' + str(lane_ego)
    poly_lane_ego = LineString(traci.lane.getShape(laneID_ego)).buffer(
        1.6, cap_style=2, single_sided=False)
    edge_col, _, _ = traci.simulation.convertRoad(
        collision.area.centroid.x, collision.area.centroid.y)
    lane_cnt = traci.edge.getLaneNumber(edge_col)

    poly_lanes_invaded = None
    for i in range(0, lane_cnt):
        laneID_i = str(edge_col) + '_' + str(i)
        poly_lane_i = LineString(traci.lane.getShape(laneID_i)).buffer(
            1.6, cap_style=2, single_sided=False)
        if poly_lane_i.intersects(collision.area):
            poly_lanes_invaded = poly_lane_i
            break

    if poly_lanes_invaded == None:
        return []

    wp_start = closestNode([ego.xpos, ego.ypos], G)
    wp_target = wps[-1]

    is_erasing = False
    for i, wp in enumerate(wps):
        if wp == wp_start:
            continue

        pnt_wp = Point(G.nodes[wp]['x'], G.nodes[wp]['y'])
        edge_wp, pos_wp, _ = traci.simulation.convertRoad(pnt_wp.x, pnt_wp.y)

        # print('wp and pnt_wp, ', wp, G.nodes[wp]['x'], G.nodes[wp]['y'])

        if pnt_wp.intersects(poly_lanes_invaded) and 'uv' in collision.ID:
            if pnt_wp.intersects(poly_lane_ego) and pnt_wp.distance(collision.area) < 10:
                G.remove_node(wp)
                is_erasing = True
            elif not pnt_wp.intersects(poly_lane_ego) and pnt_wp.distance(collision.area) < 1.6:
                G.remove_node(wp)
                is_erasing = True

        elif is_erasing:
            wp_target = wps[i+1]
            break

    # print('wp_target', wp_target, G.nodes[wp_target]['x'], G.nodes[wp_target]['y'])
    wps_inter = nx.all_simple_paths(G, source=wp_start, target=wp_target)
    # print('wps_inter: ', wps_inter)
    wp_append = wps
    # print('ego wp: ', ego.wp_ego)
    for wp_inter in wps_inter:
        wp_new = wp_inter.append(wp_inter)
        plan_prime = convert_waypoints_to_plan(ego, wp_new)
        plans_prime.append(plan_prime)

    return plans_prime


def create_safe_response(ego, plans_prime):
    acc_list = np.linspace(+ego.acc_max, -ego.dec_max, 10)
    behavior = ego.behavior
    acc_available = -ego.dec_max
    response = Response(ego.plan, acc_available)
    for acc_prime in acc_list:
        if ego.vel / delta_T + acc_prime > vel_max:
            continue
        else:
            acc_available = acc_prime

        is_collision = detect_candidate_collision(
            ego, ego.plan, acc_available, behavior)
        if not is_collision:
            response.a = acc_available
            break

    return response


def detect_TL_violation(ego, plan, acc):

    TLSignal = ego.collision
    RE_candi = Detection.create_evasive_DE(ego, plan, acc)

    if RE_candi.intersects(TLSignal.area):
        return True
    else:
        return False


def decide_emergency_behavior(ego, plan_prime, acc_prime, tb=3.0):
    behavior = "LF"
    vel_prime = ego.vel / delta_T + acc_prime
    if not len(plan_prime):
        return
    else:
        linestring_plan = cut_linestring(
            LineString(plan_prime), vel_prime*tb)[0]

        # print('emergency behavior!')
        edge_prev, _, lane_prev = traci.simulation.convertRoad(
            ego.xpos, ego.ypos)

        laneID_prev = str(edge_prev) + str(lane_prev)

        for pnt in list(linestring_plan.coords):
            edge_plan, _, lane_plan = traci.simulation.convertRoad(
                pnt[0], pnt[1])
            laneID_plan = str(edge_plan) + str(lane_plan)

            if laneID_prev != laneID_plan:
                if int(lane_plan) == int(lane_prev) + 1:
                    behavior = "LC_L"
                elif int(lane_plan) == int(lane_prev) - 1:
                    behavior = "LC_R"
                break

    return behavior


def detect_candidate_collision(ego, plan, acc, behavior=None):
    if ego.collision != None:
        collision = ego.collision.area
    else:
        collision = None

    if not behavior:
        behavior = ego.behavior

    de_prime_ego = Detection.create_RE(ego, plan, acc)

    if not len(ego.objects_sensed) and collision == None:
        return False

    if collision != None and de_prime_ego.intersects(collision):
        return True

    for obj in ego.objects_sensed:
        RE_obj, _ = Detection.create_RE_other(ego, obj, ego.Connection)
        if de_prime_ego.intersects(RE_obj):
            return True

    if 'LC' in behavior:
        if not len(ego.objects_sensed):
            return False

        edge_ego, _, lane_ego = traci.simulation.convertRoad(
            ego.xpos, ego.ypos)

        if behavior == 'LC_L':
            blinker = +1
        elif behavior == 'LC_R':
            blinker = -1
        else:
            blinker = 0

        lane_target = int(lane_ego) + int(blinker)
        laneID_target = str(edge_ego) + '_' + str(lane_target)

        ee_ego, t_enter = Detection.create_LE(
            ego, plan, acc, laneID=laneID_target)

        for obj in ego.objects_sensed:
            LE_obj, _ = Detection.create_LE_other(obj, laneID_target, t_enter)

            if LE_obj != None and LE_obj.intersects(ee_ego) and de_prime_ego.intersects(ee_ego):
                return True

        return False
