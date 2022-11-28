#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import traci
import numpy as np
import math
import numpy.linalg as LA
from math import atan2,degrees
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import linemerge, unary_union, polygonize
from scipy import interpolate

egoID = 'auv1'
def calculate_plan_curvature(plan):
    
    k = 0.0
    
    def calculate_curvature(x,y):
        t_a = LA.norm([x[1]-x[0],y[1]-y[0]])
        t_b = LA.norm([x[2]-x[1],y[2]-y[1]])
        
        M = np.array([
            [1, -t_a, t_a**2],
            [1, 0,    0     ],
            [1,  t_b, t_b**2]
        ])
        
        try:
            a = np.matmul(LA.inv(M),x)
            b = np.matmul(LA.inv(M),y)
            kappa = 2*(a[2]*b[1]-b[2]*a[1])/(a[1]**2.+b[1]**2.)**(1.5)
            return kappa
        except:
            return 0.0
    
    if len(plan) < 3:
        return k
    
    for i in range(0,len(plan)-2):
        x_i = [plan[i][0], plan[i+1][0], plan[i+2][0]]
        y_i = [plan[i][1], plan[i+1][1], plan[i+2][1]]
        k_i = calculate_curvature(x_i, y_i)
        k_i = abs(k_i)
        if k_i > k:
            k = k_i
            
    return k
        


def linear_interpolateion(x, y):

    points = np.array([x, y]).T  # a (nbre_points x nbre_dim) array

    # Linear length along the line:
    distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
    distance = np.insert(distance, 0, 0)

    alpha = np.linspace(distance.min(), int(distance.max()), 100)
    interpolator =  interpolate.interp1d(distance, points, kind='quadratic', axis=0)
    interpolated_points = interpolator(alpha)

    out_x = interpolated_points.T[0]
    out_y = interpolated_points.T[1]

    return out_x, out_y

    


def visualize_polygon(ego, target, shape, type_shape):
    
    if ego.ID != egoID:
        return
    
    if type_shape == 'AE':
        if ego.ID != target.ID:
            return
        type_shape = 'AE_'
        color = (0,0,255)
        layer = 5

    elif type_shape == 'DE':
        type_shape = 'DE_'
        color = (0,255,0)
        layer = 6

    elif type_shape == 'EE':
        type_shape = 'EE_'
        color = (255,255,255)
        if ego.ID != target.ID:
            color = (255,255,255)
        layer = 5

    elif type_shape == 'SZ':
        type_shape = 'SZ_'
        color = (255,128,0)
        layer = 6

    elif type_shape == 'DANGER':
        type_shape = 'DANGER_'
        color = (255,0,0)
        layer = 5

    elif type_shape =='EGO':
        type_shape = 'EGO_'
        color = (128,128,0)
    
    opacity = 1.0
    
    # print('ID LIST: ', traci.polygon.getIDList())
    idlist = traci.polygon.getIDList()
    
    if type_shape+str(target.ID) not in idlist:
        flag=False
        if type_shape == 'DANGER_' or type_shape == 'AE_':
            flag=True
            
        try:
            if not len(shape):
                return
            else:
                traci.polygon.add(type_shape+str(target.ID), shape, (color[0],color[1],color[2],int(255*opacity)), fill=flag, layer=layer, lineWidth=0.1)
            return
        except:
            return
    else:
        try:
            traci.polygon.setShape(type_shape+str(target.ID), shape)
            return
        except:
            return
        
        
        
def hanging_line(point1, point2):

    a = (point2[1] - point1[1])/(np.cosh(point2[0]) - np.cosh(point1[0]))
    b = point1[1] - a*np.cosh(point1[0])
    x = np.linspace(point1[0], point2[0], 100)
    y = a*np.cosh(x) + b

    return (x,y)



def project_shape_over_centerline(shape, centerline, step=float(3.0)):
    if shape.geom_type == "Polygon":
        boundary = shape.boundary
    elif shape.geom_type == "LineString":
        boundary = shape
    elif shape.geom_type == "MultiLineString":
        shape = linemerge(shape)
        boundary = shape
    elif shape.geom_type == "MultiPolygon":
        shape = unary_union(shape)
        boundary = shape.boundary

    max_length = boundary.length
    step = int(max_length / 10)

    if step > max_length or step == 0:
        return shape
    
    projected_points = []
    distance = 0
    while distance < max_length:
        print('distance / max_length: ', distance, max_length)
        vertex = boundary.interpolate(distance)
        projected_distance = centerline.project(vertex)
        projected_point = centerline.interpolate(projected_distance)
        
        if len(projected_points) < 2:
            projected_points.append(projected_point)
        elif not projected_point.intersects(LineString(projected_points)):
            projected_points.append(projected_point)

        distance += step

    try:
        projected_shape = LineString(projected_points)
        return projected_shape
    except:
        return None
    



def perpendicular(a) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


def normalize(a):
    a = np.array(a)
    return a/np.linalg.norm(a)


def define_circle(p1, p2, p3):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

    if abs(det) < 1.0e-6:
        return (None, np.inf)

    # Center of circle
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det
    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)

    return ((cx, cy), radius)



def cut_linestring(line, distance):
    # Cuts a line in two at a distance from its starting point
    if distance <= 0.0 or distance >= line.length:
        # print('line: ', line)
        return [line]
    coords = list(line.coords)
    for i, p in enumerate(coords):
        pd = line.project(Point(p))
        if pd == distance:
            return [
                LineString(coords[:i+1]),
                LineString(coords[i:])]
        if pd > distance:
            cp = line.interpolate(distance)
            return [LineString(coords[:i] + [(cp.x, cp.y)]),
                    LineString([(cp.x, cp.y)] + coords[i:])]


def sort_counterclockwise(points, center):
    if not len(points):
        return []
    
    temp = points[-1]
    points = points[0:-1]

    center_x = center.x
    center_y = center.y 
    
    angles = [math.atan2(point.y - center_y, point.x - center_x) for point in points]
    counterclockwise_indices = sorted(range(len(points)), key=lambda i: angles[i])
    counterclockwise_points = [points[i] for i in counterclockwise_indices]
    counterclockwise_points.append(temp)
    
    return counterclockwise_points


def find_nearest(array, pnt):
    array_dist = []
    for i in range(len(array)):
        array_dist.append(math.dist([array[i].x, array[i].y], [pnt.x, pnt.y]))
    array_dist = np.asarray(array_dist)
    idx = (array_dist).argmin()
    
    return array[idx]


def counter_clockwise_sort(points):
    return sorted(points, key=lambda point: point.x * (-1 if point.y >= 0 else 1))


def divide_AOI(vehicle, sensing_range, D_safety):
    
    polygon = vehicle.AOI.intersection(sensing_range)

    laneID = traci.vehicle.getLaneID(vehicle.ID)
    shape_lane = traci.lane.getShape(laneID)
    vector_lane = [shape_lane[1][0]-shape_lane[0][0], shape_lane[1][1]-shape_lane[0][1]]
    
    vector_norm = vector_lane / np.linalg.norm(vector_lane)
    
    a = [vehicle.xpos, vehicle.ypos]
    b = [vehicle.xpos + (D_safety * vector_norm[0]), vehicle.ypos + (D_safety * vector_norm[1])]
    
    length = 200
    
    line_safety = LineString([a, b])
    
    left = line_safety.parallel_offset(length / 2, 'left')
    right = line_safety.parallel_offset(length / 2, 'right')
    
    c = left.boundary[1]
    d = right.boundary[0]  # note the different orientation for right offset
    
    cd = LineString([c, d])
    
    line = cd
    merged = linemerge([polygon.boundary, line])
    borders = unary_union(merged)
    polygons = polygonize(borders)
    
    result = list(polygons)
    return result


def create_poly_search(wp_prev, wp_next, delta, rad):
    p1 = np.array([wp_prev[0]-delta*math.cos(rad), wp_prev[1]+delta*math.sin(rad)])
    p2 = np.array([wp_prev[0]+delta*math.cos(rad), wp_prev[1]-delta*math.sin(rad)])
    p3 = np.array([wp_next[0]+delta*math.cos(rad), wp_next[1]-delta*math.sin(rad)])
    p4 = np.array([wp_next[0]-delta*math.cos(rad), wp_next[1]+delta*math.sin(rad)])
    
    poly_search = Polygon([p1, p2, p3, p4])
    
    return poly_search


def create_poly_lane(vehID):
    shape = traci.lane.getShape(traci.vehicle.getLaneID(vehID))
    vector = [shape[1][0]-shape[0][0], shape[1][1]-shape[0][1]]
    angle = calc_clockwise_angle([0,99999], vector)
    rad = math.radians(angle)
    
    p1 = np.array([shape[0][0]-1.6*math.cos(rad), shape[0][1]+1.6*math.sin(rad)])
    p2 = np.array([shape[0][0]+1.6*math.cos(rad), shape[0][1]-1.6*math.sin(rad)])
    p3 = np.array([shape[1][0]+1.6*math.cos(rad), shape[1][1]-1.6*math.sin(rad)])
    p4 = np.array([shape[1][0]-1.6*math.cos(rad), shape[1][1]+1.6*math.sin(rad)])
    
    poly_lane = Polygon([p1, p2, p3, p4])
    
    return poly_lane


def create_lane_shape(laneID, width=3.2):
    width = width/2
    shape = traci.lane.getShape(laneID)
    vector = [shape[1][0]-shape[0][0], shape[1][1]-shape[0][1]]
    angle = calc_clockwise_angle([0,99999], vector)
    rad = math.radians(angle)
    
    p1 = [round(shape[0][0]-width*math.cos(rad),1), round(shape[0][1]+width*math.sin(rad),1)]
    p2 = [round(shape[0][0]+width*math.cos(rad),1), round(shape[0][1]-width*math.sin(rad),1)]
    p3 = [round(shape[1][0]+width*math.cos(rad),1), round(shape[1][1]-width*math.sin(rad),1)]
    p4 = [round(shape[1][0]-width*math.cos(rad),1), round(shape[1][1]+width*math.sin(rad),1)]
    
    shape_lane = [p1, p2, p3, p4]
    
    return shape_lane

def create_lane_area(laneID):
    shape = traci.lane.getShape(laneID)
    vector = [shape[1][0]-shape[0][0], shape[1][1]-shape[0][1]]
    angle = calc_clockwise_angle([0,99999], vector)
    rad = math.radians(angle)
    
    p1 = [round(shape[0][0]-1.6*math.cos(rad),1), round(shape[0][1]+1.6*math.sin(rad),1)]
    p2 = [round(shape[0][0]+1.6*math.cos(rad),1), round(shape[0][1]-1.6*math.sin(rad),1)]
    p3 = [round(shape[1][0]+1.6*math.cos(rad),1), round(shape[1][1]-1.6*math.sin(rad),1)]
    p4 = [round(shape[1][0]-1.6*math.cos(rad),1), round(shape[1][1]+1.6*math.sin(rad),1)]
    
    area_lane = Polygon([p1, p2, p3, p4])
    
    return area_lane


def is_member(A,B):
    flag = 0
    if sum([ np.sum(a == B) for a in A ]) != 0:
        flag = 1
        
    return flag



def is_overlap(bools):
    try:
        ints  = bools.flatten().prod()
        fltn_bools = np.hstack(bools)
    except: # should not pass silently.
        fltn_bools = np.array(is_overlap(a) for a in bools)
        
    ints = fltn_bools.prod()
    
    return bool(ints)


def find_intersect(wp_ego, wp_other, G):
    """ returns a (x, y) tuple or None if there is no intersection """
    traj_ego = []
    traj_other = []

    for wp in wp_ego:
        temp = np.array([G.nodes[wp]['x'], G.nodes[wp]['y']])
        traj_ego.append(temp)
        
    for wp in wp_other:
        temp = np.array([G.nodes[wp]['x'], G.nodes[wp]['y']])
        traj_other.append(temp)

    line_ego = LineString(traj_ego)
    line_other = LineString(traj_other)
    
    intersect = line_ego.intersection(line_other)
    
    return intersect


def closestNode(pnt, G):
    # print('pnt: ', pnt)
    # print('G.nodes: ', G.nodes)
    dist_arr = np.zeros((len(G), 2))
    
    idx = 0
    for node in G:
        dist = math.dist([pnt[0], pnt[1]], [G.nodes[node]['x'], G.nodes[node]['y']])
        dist_arr[idx] = [int(node), dist]
        idx += 1
    # print('dist_arr#1: ', dist_arr)
    # print('dist_arr#2: ', dist_arr[np.where(dist_arr==min(dist_arr[:,1]))[0]])
    
    min_idx = dist_arr[np.where(dist_arr==min(dist_arr[:,1]))[0]][0][0]
    min_idx = int(min_idx)
    
    return min_idx
    

def calcRad(vec1, vec2):
    inner = np.inner(vec1, vec2)
    norms = LA.norm(vec1) * LA.norm(vec2)

    cos = inner / norms
    rad = np.arccos(np.clip(cos, -1.0, 1.0))
    # deg = np.rad2deg(rad)
    
    return rad
    
    # print(rad)  # 1.35970299357215
    # print(deg)  # 77.9052429229879
        
    
    
def rotatePoint(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


def calc_clockwise_angle(vec1, vec2):
    """
    vec1 is an origin vector (ex: x-axis) and vec2 is a target vector

    """
    angle = degrees(atan2(vec2[0]*vec1[1] - vec2[1]*vec1[0], vec2[0]*vec1[0] + vec2[1]*vec1[1]))
    
    return angle


def calcCounterClockwiseAngle(vec1, vec2):
    """
    vec1 is an origin vector (ex: x-axis) and vec2 is a target vector

    """
    angle = -degrees(atan2(vec2[0]*vec1[1] - vec2[1]*vec1[0], vec2[0]*vec1[0] + vec2[1]*vec1[1]))
    
    return angle


def calcPnt(p1, p2, step=0.5):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    
    t = int(1/step)
    
    plan = []
    for i in range(0, t):
        x = x1 + (x2-x1) * i * step
        y = y1 + (y2-y1) * i * step
        plan.append([x,y])
        
    plan = np.array(plan)
    return plan


def create_vehicle_shape(vehicle):
    xpos = vehicle.xpos
    ypos = vehicle.ypos
    angle = vehicle.angle
    width = vehicle.width
    length = vehicle.length
    
    rad = math.radians(angle)
    
    p1 = Point([xpos-(width/2)*math.cos(rad), 
                ypos+(width/2)*math.sin(rad)])
    p2 = Point([xpos+(width/2)*math.cos(rad), 
                ypos-(width/2)*math.sin(rad)])
    p3 = Point([xpos-(length)*math.sin(rad)+(width/2)*math.cos(rad), 
                ypos-(length)*math.cos(rad)-(width/2)*math.sin(rad)])
    p4 = Point([xpos-(length)*math.sin(rad)-(width/2)*math.cos(rad), 
                ypos-(length)*math.cos(rad)+(width/2)*math.sin(rad)])
    
    return Polygon([p1, p2, p3, p4])


def create_future_vehicle_shape(xpos, ypos, angle, width, length):

    rad = math.radians(angle)
    
    p1 = Point([xpos-(width/2)*math.cos(rad), 
                ypos+(width/2)*math.sin(rad)])
    p2 = Point([xpos+(width/2)*math.cos(rad), 
                ypos-(width/2)*math.sin(rad)])
    p3 = Point([xpos-(length)*math.sin(rad)+(width/2)*math.cos(rad), 
                ypos-(length)*math.cos(rad)-(width/2)*math.sin(rad)])
    p4 = Point([xpos-(length)*math.sin(rad)-(width/2)*math.cos(rad), 
                ypos-(length)*math.cos(rad)+(width/2)*math.sin(rad)])
    
    return Polygon([p1, p2, p3, p4])



def calc_lane_vector(laneID):
    shape = traci.lane.getShape(laneID)
    vector_lane = [shape[1][0]-shape[0][0], shape[1][1]-shape[0][1]] 
    origin_lane = shape[0]
    
    return vector_lane, origin_lane


def get_vector_angle(line):
    vector_1 = [0, 1]   # Standard
    vector_2 = list([line.coords[1][0]-line.coords[0][0], line.coords[1][1]-line.coords[0][1]])
    
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = math.degrees(np.arccos(dot_product))
    
    return angle


def sector(center, start_angle, end_angle, radius, steps=100):
    def polar_point(origin_point, angle,  distance):
        return [origin_point.x + math.sin(math.radians(angle)) * distance, origin_point.y + math.cos(math.radians(angle)) * distance]

    if start_angle > end_angle:
        temp = end_angle
        end_angle = start_angle
        start_angle = temp
    else:
        pass
    step_angle_width = (end_angle-start_angle) / steps
    sector_width = (end_angle-start_angle) 
    segment_vertices = []

    segment_vertices.append(polar_point(center, 0,0))
    segment_vertices.append(polar_point(center, start_angle,radius))

    for z in range(1, steps):
        segment_vertices.append((polar_point(center, start_angle + z * step_angle_width,radius)))
    segment_vertices.append(polar_point(center, start_angle+sector_width,radius))
    segment_vertices.append(polar_point(center, 0,0))
    
    return Polygon(segment_vertices)

        


def shrink_or_swell_shapely_polygon(my_polygon, factor=0.05, swell=True):
    ''' returns the shapely polygon which is smaller or bigger by passed factor.
        If swell = True , then it returns bigger polygon, else smaller '''
    from shapely import geometry

    #my_polygon = mask2poly['geometry'][120]
    # shrink_factor = 0.05 #Shrink by 10%
    xs = list(my_polygon.exterior.coords.xy[0])
    ys = list(my_polygon.exterior.coords.xy[1])
    x_center = 0.5 * min(xs) + 0.5 * max(xs)
    y_center = 0.5 * min(ys) + 0.5 * max(ys)
    min_corner = geometry.Point(min(xs), min(ys))
    # max_corner = geometry.Point(max(xs), max(ys))
    center = geometry.Point(x_center, y_center)
    shrink_distance = center.distance(min_corner)*factor

    if swell:
        my_polygon_resized = my_polygon.buffer(shrink_distance) #expand
    else:
        my_polygon_resized = my_polygon.buffer(-shrink_distance) #shrink

    
    return my_polygon_resized