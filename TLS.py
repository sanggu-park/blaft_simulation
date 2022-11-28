#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 21 16:41:33 2022

@author: sanggupark
"""
import traci
from dataclasses import dataclass
import numpy as np
from shapely.geometry import LineString



tlsID = 'n3'
lane_index = [':n3_0_0', ':n3_1_0', ':n3_1_1', ':n3_3_0',
              ':n3_4_0', ':n3_5_0', ':n3_5_1', ':n3_7_0',
              ':n3_8_0', ':n3_9_0', ':n3_9_1', ':n3_11_0',
              ':n3_12_0', ':n3_13_0', ':n3_13_1', ':n3_15_0']


shape_index = [LineString([[-12.8, 285.0], [-9.6, 285.0]]),
               LineString([[-9.6, 285.0], [-6.4, 285.0]]),
               LineString([[-6.4, 285.0], [-3.2, 285.0]]),
               LineString([[-3.2, 285.0], [0.0, 285.0]]),
               
               LineString([[35.0, 262.8], [35.0, 259.6]]),
               LineString([[35.0, 259.6], [35.0, 256.4]]),
               LineString([[35.0, 256.4], [35.0, 253.2]]),
               LineString([[35.0, 253.2], [35.0, 250.0]]),
               
               LineString([[12.8, 215.0], [9.6, 215.0]]),
               LineString([[9.6, 215.0], [6.4, 215.0]]),
               LineString([[6.4, 215.0], [3.2, 215.0]]),
               LineString([[3.2, 215.0], [0.0, 215.0]]),

               LineString([[-35.0, 237.2], [-35.0, 240.4]]),
               LineString([[-35.0, 240.4], [-35.0, 243.6]]),
               LineString([[-35.0, 243.6], [-35.0, 246.8]]),
               LineString([[-35.0, 246.8], [-35.0, 250.0]])]
               



@dataclass(init = True)
class TLS:
    ID: str = None
    signal: str = None
    boundary: LineString() = None
    ts: float = np.inf



def create_and_update_TLS():
    TLS_info = []
    GYR = traci.trafficlight.getRedYellowGreenState(tlsID)
    
    for i, signal in enumerate(GYR):
        TLS_info.append(TLS(ID=lane_index[i], signal=signal, boundary=shape_index[i], ts=traci.simulation.getTime()))
        
    return TLS_info