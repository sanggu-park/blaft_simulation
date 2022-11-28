"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""
import math
import random
import matplotlib.pyplot as plt
import numpy as np
import traci
from dataclasses import dataclass
from SimpleMath import calcPnt, create_poly_lane, create_poly_search
from shapely.geometry import Point

delta_T = 0.1

@dataclass(init = True)
class Object_TTC:
    ID: int
    xpos: float
    ypos: float
    radius: float
    pnt_collision: Point
    ttc_ego: float
    ttc_obs: float
    tgap_RSS: float
    
    
def generate_plan_points(vehicle):
    points = []
    G = vehicle.MapGraph
    pnt_now = [vehicle.xpos, vehicle.ypos]
    
    edge_now, pos_now, lane_now = traci.simulation.convertRoad(vehicle.xpos, vehicle.ypos)
    
    
    points.append(np.array(pnt_now))
    
    if not len(vehicle.wp_ego):
        return points
    else:
        for i in range(len(vehicle.wp_ego)):
            edge_wp, pos_wp, lane_wp = traci.simulation.convertRoad(G.nodes[vehicle.wp_ego[i]]['x'], G.nodes[vehicle.wp_ego[i]]['y'])
            
            if edge_wp == edge_now and pos_wp >= pos_now:
                if math.dist([G.nodes[vehicle.wp_ego[i]]['x'], G.nodes[vehicle.wp_ego[i]]['y']], pnt_now) <= vehicle.sensor.radius:
                    points.append([G.nodes[vehicle.wp_ego[i]]['x'], G.nodes[vehicle.wp_ego[i]]['y']])
        
    return points



def reactivePlanning(vehicle, object_list=[], delta=0.2):
    
    G = vehicle.MapGraph
    plan = []
    rad = math.radians(vehicle.angle)

    poly_lane = create_poly_lane(vehicle.ID)
    pos_now = [vehicle.xpos, vehicle.ypos]
    wp_prev = pos_now
    
    wp_current = vehicle.wp_ego
    # print('Remaining waypoints: ', wp_current)
    
    if not len(wp_current):
        if math.dist(pos_now, vehicle.dst) < 0.1:
            return -1   # When arrived, brake
        else:
            return 1    # when there's no more waypoints left

    else:
        wp_next = np.array([G.nodes[wp_current[0]]['x'], G.nodes[wp_current[0]]['y']])
        _, _, lane_prev = traci.simulation.convertRoad(wp_prev[0], wp_prev[1])
        _, _, lane_next = traci.simulation.convertRoad(wp_next[0], wp_next[1])

    if not lane_prev == lane_next:
        poly_search = create_poly_search(wp_prev, wp_next, delta, rad)
        poly_overlaps = poly_search.intersection(poly_lane)
        
        if not poly_overlaps.is_empty:
            rand_area = poly_overlaps
        else:
            rand_area = poly_search
            
        # print('object list: ', object_list)
        plan_init = RRT(start=wp_prev, goal=wp_next, object_list=object_list, rand_area=rand_area).planning()

        if len(plan_init):
            plan.extend(plan_init)
    else:
        plan_init = calcPnt(wp_prev, wp_next)
        plan.extend(plan_init)
        
    """ Finding trajectory points from first wp to the last wp """
    # print('TEST2: ', len(wp_current), wp_current)
    
    for i in range(len(wp_current)-1):
        wp_prev = np.array([G.nodes[wp_current[i]]['x'], G.nodes[wp_current[i]]['y']])
        wp_next = np.array([G.nodes[wp_current[i+1]]['x'], G.nodes[wp_current[i+1]]['y']])
        
        _, _, lane_prev = traci.simulation.convertRoad(wp_prev[0], wp_prev[1])
        _, _, lane_next = traci.simulation.convertRoad(wp_next[0], wp_next[1])
        
        if not lane_prev == lane_next:
            poly_search = create_poly_search(wp_prev, wp_next, delta, rad)
            poly_overlaps = poly_search.intersection(poly_lane)
            
            if not poly_overlaps.is_empty:
                rand_area = poly_overlaps
            else:
                rand_area = poly_search
            
            plan_i = RRT(start=wp_prev, goal=wp_next, object_list=object_list, rand_area=rand_area).planning()
            if len(plan_i):
                plan.extend(plan_i)
                
        else:
            plan_i = calcPnt(wp_prev, wp_next)
            plan.extend(plan_i)
    
    return plan


def calculate_blame(vehicle, pnt_intersection):
    
    if pnt_intersection.is_empty:
        return
    
    xpos_ego = vehicle.xpos
    ypos_ego = vehicle.ypos
    vel_ego = vehicle.vel
    acc_ego = vehicle.acc_max
    dec_min_ego = vehicle.dec_min
    dec_max_ego = vehicle.dec_max
    rho = vehicle.response
    edge_ego, pos_ego, lane_ego = traci.simulation.convertRoad(xpos_ego, ypos_ego)
    edge_collision, pos_collision, lane_collision = traci.simulation.convertRoad(pnt_intersection.x, pnt_intersection.y)    
    
    D_RSS_init = max((vel_ego/delta_T*rho) + (acc_ego*rho**2)/2 + ((vel_ego/delta_T+rho*acc_ego)**2)/(2*dec_min_ego), 0)
    D_innocent = max((vel_ego/delta_T*rho) + (acc_ego*rho**2)/2 + ((vel_ego/delta_T+rho*acc_ego)**2)/(2*dec_max_ego), 0)
    
    
    for obj in vehicle.objects_sensed:
        distance = math.dist([xpos_ego, ypos_ego], [obj.xpos, obj.ypos])
        if lane_collision != lane_ego:
            # print('The vehicle has no priority!')
            # print('lane check', lane_collision, lane_ego)
            blame_rate = min(max(-(1/(D_innocent-D_RSS_init)) * (distance - D_RSS_init), 0), 1)
            
        elif lane_collision == lane_ego:
            # print('The vehicle has a priority!')
            blame_rate = min(max((1/(D_innocent-D_RSS_init)) * (distance - D_innocent), 0), 1)
            
            
    # print(vehicle.ID, 'blame_rate: ', blame_rate)
    return blame_rate


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 object_list,
                 rand_area,
                 expand_dis=4.0,
                 path_resolution=4.0,
                 goal_sample_rate=5,
                 max_iter=1000):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        objectList:object Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.rand_area = rand_area
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.object_list = object_list
        self.node_list = []

    def planning(self, animation=False):
        """
        rrt path planning

        animation: flag for animation on or off
        """
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.object_list):
                self.node_list.append(new_node)

            # if animation and i % 5 == 0:
                # self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.object_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            # if animation and i % 5:
            #     self.draw_graph(rnd_node)

        return []  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        path = np.flip(path, axis=0)
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)
    
    def get_random_node(self):
        # bounds = [self.p1, self.p2, self.p3, self.p4]
        rand_pnt = return_rand_point(self.rand_area)
        
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(rand_pnt[0], rand_pnt[1])
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.object_list:
            self.plot_circle(ox, oy, size)


    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, objectList):
        
        if node is None:
            return False
        
        for (ox, oy, size) in objectList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def return_rand_point(polygon):
    xmin, ymin, xmax, ymax = polygon.bounds
    while True:
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        
        if Point(x, y).within(polygon):
            # if this condition is true, add to a dataframe here
            return np.array((x, y))
            break
