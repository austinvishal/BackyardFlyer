# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 23:04:29 2020

@author: austi
"""


import csv
import datetime
import time
import pprint
pp = pprint.PrettyPrinter(indent=4)

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

def load_telemetry(file):
    """
    Loads the Telemetry data file
    """
    long_lat_parser = lambda r: { 'longitude': float(r[2]), 'latitude': float(r[3]), 'altitude': float(r[4]) }
    local_parser = lambda r: { 'north': float(r[2]), 'east': float(r[3]), 'down': float(r[4]) }
    parser = {
        'STATE' : lambda r: { 'armed' : bool(row[2]), 'guided' : bool(row[3]) },
        'GLOBAL_POSITION' : long_lat_parser,
        'GLOBAL_HOME' : long_lat_parser,
        'LOCAL_POSITION' : local_parser,
        'LOCAL_VELOCITY' : local_parser
    }
    messages = []
    with open(file) as dataFile:
        dataReader = csv.reader(dataFile)
        for row in dataReader:
            message = {}
            type = row[0][6:]
            message['type'] = type
            message['timestamp'] = time.mktime((row[1].timetuple())
            #message['timestamp'] = time.strftime('%Y-%m-%d %H:%M:%S')
            #message['timestamp'] = datetime.datetime.fromtimestamp(float(row[1]))
            #time.strftime('%Y-%m-%d %H:%M:%S')
            this_parser = parser[type]
            rest = {}
            if this_parser:
                rest = this_parser(row)
            else:
                rest['row'] = row
            messages.append({**message, **rest})
        
    return messages

def get_north_east_down(type, tl):
    """
    Returns `[north, east, down, timestamp]` telemetry for the `type` on the telemetry `tl`
    """
    north = []
    east = []
    down = []
    timestamp = []
    epoch = datetime.datetime.utcfromtimestamp(0)
    for t in tl:
        if t['type'] == type:
            north.append(t['north'])
            east.append(t['east'])
            down.append(t['down'])
            timestamp.append(1000 * (t['timestamp'] - epoch).total_seconds())
    return [np.array(north), np.array(east), np.array(down), np.array(timestamp)]

def plot_3d_trajectory(east, north, down, title):
    """
    Plot the trajectory defined by `[x, y, z]`
    """
    fig = plt.figure(figsize=(20,10))
    ax = fig.gca(projection='3d')
    up = -down
    ax.plot(east, north, up)
    ax.plot(east, north, up, 'ro')
    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.set_zlabel('-Down [m]')
    ax.set_title(title)
    plt.show()
    
def plot_xy_trajectory(east, north, down, timestamp, title):
    """
    Plot the `[x, y]` trajectory and altitude
    """
    fig, axes = plt.subplots(1, 2, figsize=[20,10])
    axP = axes[0]
    axP.plot(east, north)
    axP.plot(east, north, 'ro')
    axP.set_xlabel('East [m]')
    axP.set_ylabel('North [m]')
    axP.grid()
    axP.set_title('Ground Trajectory')
    
def plot_velocity(east, north, down, timestamp, title):
    """
    Plot velocity on the 3D axis
    """
    fig, axes = plt.subplots(1, 3, figsize=[20,10])
    values = [east, north, -down]
    sub_titles = ['East', 'North', '-Down']
    for ax, value, sub_title in zip(axes, values, sub_titles):
        ax.plot(timestamp, value)
        ax.plot(timestamp, value, 'ro')
        ax.set_xlabel('Time')
        ax.set_ylabel('Velocity [m/s]')
        ax.grid()
        ax.set_title(sub_title)
    
    plt.suptitle(title, fontsize=24)

def plot_local_position_and_velocity(file, title):
    """
    Plot local position and velocity for the telemetry file `file`
    """    
    tl = load_telemetry(file)
    local_north, local_east, local_down, local_timestamp = get_north_east_down('LOCAL_POSITION', tl)
    v_local_north, v_local_east, v_local_down, v_timestamp = get_north_east_down('LOCAL_VELOCITY', tl)
    plot_3d_trajectory(local_east, local_north, local_down, title + ' -> Trajectory')
    plot_xy_trajectory(local_east, local_north, local_down, local_timestamp, title + ' -> Trajectory')
    plot_velocity(v_local_east, v_local_north, v_local_down, v_timestamp, title + ' -> Velocity')
    

plot_local_position_and_velocity('../Logs/TLog.txt', 'TL_01: Square')