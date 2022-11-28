#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random
import traci
from dataclasses import dataclass
import numpy as np
import math
from shapely.geometry import LineString, Point, Polygon
from SimpleMath import cut_linestring, create_lane_shape, create_vehicle_shape, visualize_polygon, closestNode
import networkx as nx

import Router
import Sensing
import Detection
import CAM
import Control


delta_T = 0.1
vel_max = 12.0
egoID = 'auv1'

    
@dataclass(init = True)
class Sensor:
    radius: float
    fov: float  # field of view
    rate: float
    delay: float
    
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
    response: float = 1.0
    blinker: int = 0

@dataclass(init = True)
class Collision:
    ID: str = None
    area: Polygon() = None
    ts: float = np.inf
    
    
@dataclass(init = True)
class Response:
    plan: np.ndarray = np.empty((0,1))
    a: float = 0.0
    k: float = 0.0



class AUV:
    def __init__(self, ID, src, dst, xpos, ypos, vel, angle, 
                 acc_max, dec_max, dec_min, response, 
                 length, width, height, MapGraph, RouteGraph, Connection, WayPoints, InterLane):
        
        """ Containing Info """
        self.ID = ID
        self.src = src
        self.dst = dst        
        self.xpos = float(xpos)
        self.ypos = float(ypos)
        self.vel = float(vel)
        self.angle = float(angle)
        self.acc_max = float(acc_max)
        self.dec_max = float(dec_max)
        self.dec_min = float(dec_min)
        self.response = float(response)
        self.length = float(length)
        self.width = float(width)
        self.height = float(height)
        self.behavior = str
        self.blinker = int(0)
        
        self.sensor = Sensor(radius = 60.00, fov = 2 * np.pi, rate = 1.00, delay = 0.001)

        self.objects_sensed = np.empty((0,1))
        self.collision: Collision() = None
        
        self.wp_ego = WayPoints    #(t,x,y)
        self.plan: np.ndarray = np.empty((0,1))
        self.de: Polygon()
        self.ee: Polygon()
        self.ae: Polygon()

        self.t_enter = 0.0
        self.waiting = 0.0
        self.MapGraph = MapGraph
        self.RouteGraph = RouteGraph
        self.Connection = Connection
        self.InterLane = InterLane
        
        
        
    def infinite_rerouting(self):
        # routelist = [
        #              "route1_5_r", "route5_1_r", 
        #              "route6_0_r", "route0_6_r",
        #              "route2_5_s", "route5_2_s", 
        #              "route6_0_s", "route0_6_s",
        #              "route0_3_l", "route3_0_l", 
        #              "route1_2_l", "route2_1_l",
        #              "route0_3_r", "route3_0_r", 
        #              "route1_2_r", "route2_1_r",
        #              "route3_6_r", "route6_3_r", 
        #              "route4_5_r", "route5_4_r"]
        
        
        # routelist = [
        #              # "route_h_1_1",
        #             "route_h_1_2",
        #              # "route_h_2_1",
        #             "route_h_2_2",
        #              # "route_h_3_1",
        #             "route_h_3_2",
        #              # "route_h_4_1",
        #             "route_h_4_2"
        #             ]
  
        edge_now, pos_now, _ = traci.simulation.convertRoad(self.xpos, self.ypos)
        edge_dst, pos_dst, _ = traci.simulation.convertRoad(self.dst[0], self.dst[1])
        
        print('rerouting started!')
        print('edge test: ', edge_now, edge_dst, self.dst)
        if edge_now != edge_dst or (edge_now == edge_dst and pos_now < 20):
            return
        
        if edge_now == '0to1':
            dst_new = random.choice([
                                    # [random.choice([251.6, 254.8, 258.0, 261.2]), random.randrange(350, 400, 8)], # 4to6
                                    # [random.randrange(100, 150, 8), random.choice([498.4, 495.2, 492.0, 488.8])], # 5to6
                                    # [random.choice([-251.6, -254.8, -258.0, -261.2]), random.randrange(100, 150, 8)],  # 2to0
                                    [random.randrange(100, 150, 8), random.choice([498.4, 495.2, 492.0, 488.8])] # 5to6
                                    ])
        elif edge_now == '0to2':
            dst_new = random.choice([[random.randrange(100, 150, 8), random.choice([501.6, 504.8, 508.0, 511.2])], # 4to6
                                    [random.randrange(100, 150, 8), random.choice([498.4, 495.2, 492.0, 488.8])], # 5to6
                                    [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])]]) # 1to0   
        elif edge_now == '1to0':
            dst_new = random.choice([
                                    # [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(350, 400, 8)], # 3to5
                                    # [random.randrange(350, 400, 8), random.choice([248.4, 245.2, 242.0, 238.8])], # 3to4
                                    # [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(100, 150, 8)],  # 3to1
                                    [random.randrange(100, 150, 8), random.choice([501.6, 504.8, 508.0, 511.2])] # 6to5
                                    ])
        elif edge_now == '1to3':
            dst_new = random.choice([[random.choice([248.4, 245.2, 242.0, 238.8]), random.randrange(350, 400, 8)], # 6to4
                                    [random.randrange(100, 150, 8), random.choice([501.6, 504.8, 508.0, 511.2])], # 6to5
                                    [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])]]) # 0to1
        elif edge_now == '2to0':
            dst_new = random.choice([[random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(350, 400, 8)], # 3to5
                                    [random.randrange(150, 200, 8), random.choice([248.4, 245.2, 242.0, 238.8])], # 3to4
                                    [random.randrange(-150, -100, 8), random.choice([251.6, 254.8, 258.0, 261.2])]]) #3to2
        elif edge_now == '2to3':
            dst_new = random.choice([[random.randrange(100, 150, 8), random.choice([501.6, 504.8, 508.0, 511.2])], # 6to5
                                    [random.choice([248.4, 245.2, 242.0, 238.8]), random.randrange(350, 400, 8)], # 6to4
                                    [random.choice([-248.4, -245.2, -242.0, -238.8]), random.randrange(100, 150, 8)]]) # 0to2  
        
        elif edge_now == '3to1':
            dst_new = random.choice([[random.randrange(-150, -100, 8), random.choice([248.4, 245.2, 242.0, 238.8])]]) # 2to3
        elif edge_now == '3to2':
            dst_new = random.choice([[random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(100, 150, 8)]]) # 1to3
        elif edge_now == '3to4':
            dst_new = random.choice([[random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(300, 350, 8)]]) # 5to3
        elif edge_now == '3to5':
            dst_new = random.choice([[random.randrange(100, 150, 8), random.choice([251.6, 254.8, 258.0, 261.2])]]) # 4to3            
            
        elif edge_now == '4to3':
            dst_new = random.choice([[random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])], # 0to1
                                    [random.choice([-248.4, -245.2, -242.0, -238.8]), random.randrange(100, 150, 8)], # 0to2
                                    [random.choice([248.4, 245.2, 242.0, 238.8]), random.randrange(350, 400, 8)]]) # 6to4
        elif edge_now == '4to6':
            dst_new = random.choice([[random.randrange(100, 150, 8), random.choice([248.4, 245.2, 242.0, 238.8])], # 3to4
                                    [random.randrange(-150, -100, 8), random.choice([251.6, 254.8, 258.0, 261.2])], # 3to2
                                    [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(100, 150, 8)]]) # 3to1   
        elif edge_now == '5to3':
            dst_new = random.choice([[random.choice([-248.4, -245.2, -242.0, -238.8]), random.randrange(100, 150, 8)], # 0to2
                                    [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])], # 0to1
                                    [random.randrange(100, 150, 8), random.choice([501.6, 504.8, 508.0, 511.2])]]) # 6to5
        elif edge_now == '5to6':
            dst_new = random.choice([
                                    # [random.choice([1.6, 4.8, 8.0, 11.2]), random.randrange(350, 400, 8)], # 3to5
                                    # [random.randrange(-150, -100, 8), random.choice([251.6, 254.8, 258.0, 261.2])], # 3to2
                                    # [random.choice([-1.6, -4.8, -8.0, -11.2]), random.randrange(100, 150, 8)]
                                    [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])] # 0to1
                                    ]) # 3to1  
        elif edge_now == '6to4':
            dst_new = random.choice([[random.choice([-251.6, -254.8, -258.0, -261.2]), random.randrange(100, 150, 8)], # 2to0
                                    [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])], # 1to0
                                    [random.randrange(100, 150, 8), random.choice([498.4, 495.2, 492.0, 488.8])]]) # 5to6
        elif edge_now == '6to5':
            dst_new = random.choice([
                                    # [random.choice([-251.6, -254.8, -258.0, -261.2]), random.randrange(100, 150, 8)], # 2to0
                                    [random.randrange(-150, -100, 8), random.choice([-1.6, -4.8, -8.0, -11.2])], # 1to0
                                    # [random.randrange(100, 150, 8), random.choice([501.6, 504.8, 508.0, 511.2])]
                                    ]) # 4to6              


        src_new = [self.xpos, self.ypos]
        # print('new test: ', src_new, dst_new)
        # print('MAPGRAPH test: ', self.MapGraph)

        wp_new, RouteGraph_new, interLane = Router.generate_waypoint(src_new, dst_new, self.MapGraph, self.Connection)
        print('ROUTED!')
        self.src = src_new
        self.dst = dst_new
        self.wp_ego = wp_new
        self.RouteGraph = RouteGraph_new
        self.InterLane = interLane
        self.plan = Control.convert_waypoints_to_plan(self)
        return
        
        
    """ 0. Generate waypoints """
    def generate_waypoints(self):
        if traci.simulation.getTime() == 0:
            if not len(self.wp_ego):
                # print('selftest: ', self.src, self.dst, self.MapGraph, self.Connection)
                wp_new, RouteGraph_new, interLane = Router.generate_waypoint(self.src, self.dst, self.MapGraph, self.Connection)
                # print('TEST HH: ', RouteGraph_new.edges)
                self.wp_ego = wp_new
                self.RouteGraph = RouteGraph_new
                self.InterLane = interLane
        else:
            _, pos_now, _ = traci.simulation.convertRoad(self.xpos, self.ypos)
            _, pos_dst, _ = traci.simulation.convertRoad(self.dst[0], self.dst[1])
            
            if pos_now < pos_dst:
                wp_new, RouteGraph_new, interLane = Router.generate_waypoint([self.xpos, self.ypos], self.dst, self.MapGraph, self.Connection)
                self.wp_ego = wp_new
                self.RouteGraph = RouteGraph_new
                self.InterLane = interLane
        return
        
            
        
    def update_waypoints(self):
        G = self.RouteGraph
        length = self.length
        width = self.width
        xpos = self.xpos
        ypos = self.ypos
        rad = np.radians(self.angle)
        
        p_rr = Point([xpos-(length)*math.sin(rad)+(width/2)*math.cos(rad), 
                      ypos-(length)*math.cos(rad)-(width/2)*math.sin(rad)])
        p_lr = Point([xpos-(length)*math.sin(rad)-(width/2)*math.cos(rad), 
                      ypos-(length)*math.cos(rad)+(width/2)*math.sin(rad)])
        p_tail = Point([(p_lr.x + p_rr.x) / 2, (p_lr.y + p_rr.y) / 2])
        
        edge_now, pos_now, lane_now = traci.simulation.convertRoad(self.xpos, self.ypos)
        source = closestNode([p_tail.x, p_tail.y], G)
        reachables = nx.descendants(G, source)
        
        wp_new = []
        for wp in self.wp_ego:
            if wp in reachables or wp == source:
                wp_new.append(wp)
        
        self.wp_ego = wp_new
        return
    
     

    def generate_or_update_plan(self):
        if not len(self.plan):
            self.plan = Control.convert_waypoints_to_plan(self)
        if len(self.plan):
            self.update_plan()
        # print('self.plan: ', self.plan)
        return
    
    
    
    def update_plan(self):
        if not len(self.plan):
            self.plan = Control.convert_waypoints_to_plan(self)
        else:
            try:
                D_moved = LineString(self.plan).project(Point(self.xpos,self.ypos))
            except:
                D_moved = 0.0
                
            if abs(D_moved) <= 0.001:
                return
            
            linestring_new = cut_linestring(LineString(self.plan), D_moved)[1]
            self.plan = list(linestring_new.coords)
        return
    
    
    
        
        
    """ 1-2. decide behavior """
    def decide_behavior(self, plan=None, CAM=False, tb=3.0):
        behavior = "LF"
        waypoints = LineString(self.plan)

        if not waypoints.length:
            behavior = "Arrived"
            return

        D_blink = vel_max * tb
        poly_self = create_vehicle_shape(self)
        linestring_blink = cut_linestring(waypoints, D_blink)[0]

        BL_left = linestring_blink.parallel_offset(self.width/2, 'left')
        BL_right = linestring_blink.parallel_offset(self.width/2, 'right')
        pnt_BL_left = BL_left.boundary[1]
        pnt_BL_right = BL_right.boundary[0]

        # print('BL_TEST: ', pnt_BL_left.x, pnt_BL_left.y, pnt_BL_right.x, pnt_BL_right.y)

        BL = LineString([pnt_BL_left, pnt_BL_right])
        edge_now, _, lane_now = traci.simulation.convertRoad(self.xpos, self.ypos)
        
        for lane_i in range(traci.edge.getLaneNumber(edge_now)) :
            poly_lane = Polygon(create_lane_shape(str(edge_now)+'_'+str(lane_i)))
            if BL.intersects(poly_lane) and not poly_self.intersects(poly_lane) :
                if lane_i > lane_now :
                    print('lane is: ', str(edge_now)+'_'+str(lane_i))
                    behavior = "LC_L"
                elif lane_i < lane_now :
                    print('lane is: ', str(edge_now)+'_'+str(lane_i))
                    behavior = "LC_R"
                break
        
        if behavior == "LF":
            self.blinker = 0
            traci.vehicle.setSignals(self.ID, 0)
        elif behavior == "LC_R":
            self.blinker = -1
            traci.vehicle.setSignals(self.ID, 1)
        elif behavior == "LC_L":
            self.blinker = 1
            traci.vehicle.setSignals(self.ID, 2)

        self.behavior = behavior

        return
        
        
        
    """ 2. Conservative Sensing """
    def conservative_sensing(self, veh_other):
        print('InterLane: ', self.InterLane)
        
        if self.behavior == 'Arrived':
            return
            
        veh_other = np.delete(veh_other, np.where(veh_other==self))
        self.SZ = Point(self.xpos, self.ypos).buffer(self.sensor.radius)
        visualize_polygon(self, self, list(self.SZ.exterior.coords), 'SZ')
        self.objects_sensed = np.empty((0,1))
        
        """ Lidar Sensing """
        vehicles_sensed = Sensing.blinker_sensing(self, Sensing.lidar_sensing(self, veh_other))
        Sensing.update_prev_info(self, vehicles_sensed)



    def detect_TLS(self, TLSignals):
        edge_now, pos_now, lane_now = traci.simulation.convertRoad(self.xpos, self.ypos)
        
        fromLane = str(edge_now) + '_' + str(lane_now)
        toLane = None
        
        for src, dst in self.InterLane:
            if src == fromLane:
                toLane = dst
        
        # print('TLSignals: ', TLSignals)
        TLSignal = None
        if edge_now in ['1to3', '2to3', '4to3', '5to3']:
            for signal in TLSignals:
                # print('Intersect?: ', self.SZ.intersects(signal.boundary))
                # print('signal and toLane: ', signal.ID, toLane)
                # print('boundary test: ', list(signal.boundary.coords))
                if self.SZ.intersects(signal.boundary) and signal.ID == toLane:
                    TLSignal = signal
                    break
        
        # print('TLSIGNAL IS: ', TLSignal)
        return TLSignal
            
    
    
    """ 3. Collision Detection """
    def detect_collision(self, tlsignal):
        
        if self.behavior == None:
            return None

        self.de = Detection.create_RE(self)
        self.ae = Detection.create_CE(self)
        poly_ego = create_vehicle_shape(self)

        self.ee = Polygon()
        collision_RE = Detection.search_RE_overlap(self)
        collision_LE = None
        if 'LC' in self.behavior:
            collision_LE = Detection.search_LE_overlap(self)
            if collision_LE != None and not self.de.intersects(collision_LE.area):
                collision_LE = None
            
        """ Check distance of coll and coll_EE from self's position!"""
        collision = collision_RE
        D_DE = np.inf
        D_EE = np.inf
        if collision_RE != None:
            D_DE = poly_ego.distance(collision_RE.area)
        if collision_LE != None:
            D_EE = poly_ego.distance(collision_LE.area) 
            
        if D_DE < D_EE:
            collision = collision_RE
        elif D_DE >= D_EE:
            collision = collision_LE
        else:
            collision = None

        """ HERE We consider Traffic Light Signal """
        if collision != None and tlsignal:
            if poly_ego.distance(tlsignal.boundary) < poly_ego.distance(collision.area):
                # print('TLS is here!!!')
                if tlsignal.signal == 'r' and self.de.intersects(tlsignal.boundary):
                    collision = Collision(ID=tlsignal.ID, area=tlsignal.boundary, ts=tlsignal.ts)
                elif tlsignal.signal == 'y' and self.de.intersects(tlsignal.boundary) and not self.ae.intersects(tlsignal.boundary):
                    collision = Collision(ID=tlsignal.ID, area=tlsignal.boundary, ts=tlsignal.ts)
                        
                # print('HERE collision: ', collision)
            
        elif not collision and tlsignal:
            if tlsignal.signal == 'r' and self.de.intersects(tlsignal.boundary):
                collision = Collision(ID=tlsignal.ID, area=tlsignal.boundary, ts=tlsignal.ts)
                    
            elif tlsignal.signal == 'y' and self.de.intersects(tlsignal.boundary) and not self.ae.intersects(tlsignal.boundary):
                collision = Collision(ID=tlsignal.ID, area=tlsignal.boundary, ts=tlsignal.ts)


        """ VISUALIZATION """
        if self.ae.geom_type == 'Polygon':
            visualize_polygon(self, self, list(self.ae.exterior.coords), 'AE')
        if self.de.geom_type == 'Polygon':
            visualize_polygon(self, self, list(self.de.exterior.coords), 'DE')
        if self.ee.geom_type == 'Polygon':
            visualize_polygon(self, self, list(self.ee.exterior.coords), 'EE')
        if collision != None:
            if 'uv' in collision.ID and collision.area.geom_type == 'Polygon':
                visualize_polygon(self, collision, list(collision.area.exterior.coords), 'DANGER')
            elif ':n' in collision.ID and collision.area.geom_type == 'LineString':
                visualize_polygon(self, collision, list(collision.area.coords), 'DANGER')


        print('collision result: ', collision)
        self.collision = collision
        return
            
    
    
    
    """ 4. Collision Avoidance and Mitigation """
    def avoid_or_mitigate_collision(self):
        plans_prime = CAM.extract_alternative_plans(self)
        print('plans_prime: ', plans_prime)
        # if plans_prime == None:
            # plans_prime = [Response(self.plan, a=-self.dec_max)]
        
        response = CAM.create_safe_response(self, plans_prime)
        # optimal_response = CAM.find_optimal_response(self, candidate_responses)
    
        return response
    
    
    
    """ 5. Maneuver by Motion Control """
    def maneuver(self, response):
        # if self.ID == 'auv1':
        #     print('check reaction: ', reaction, self.vel)
        
        x, y, vel, phi = Control.translate_reaction_into_state(self, response)
        # print('check state: ', x, y, vel, phi)
        self.vel = vel
        self.plan = response.plan
        traci.vehicle.setSpeed(self.ID, self.vel)

        # print('phi: ', phi, np.rad2deg(phi))
        edge_next, pos_next, lane_next = traci.simulation.convertRoad(x, y)
        print('x, y, edge_next, pos_next, lane_next: ', x, y, edge_next, pos_next, lane_next)
        traci.vehicle.moveToXY(self.ID, edge_next, lane_next, x, y, angle=np.rad2deg(phi), keepRoute=2)
            
        
    def check_status(self):
        # print("############### CHECK STATUS ###############")
        # print('ID / src / dst:  ', self.ID, self.src, self.dst)
        # print('acc: ', self.acc_max, self.dec_max)
        # print('X / Y / VEL / angle: ', self.xpos, self.ypos, self.vel, self.angle)
        # print('behavior / blinker: ', self.behavior, self.blinker)
        # print('waypoints: ', self.wp_ego)
        # for wp in self.wp_ego:
            # print([self.MapGraph.nodes[wp]['x'], self.MapGraph.nodes[wp]['y']])
        # print('sensed objects: ', self.objects_sensed)
        # print('plan: ', self.plan)
        # print('DE: ', self.de)
        # print('TE: ', self.ee)
        # print('AE: ', self.ae)
        # print('Collision: ', self.collision)
        return
