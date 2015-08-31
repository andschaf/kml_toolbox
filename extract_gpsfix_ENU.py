#!/usr/bin/env python

# -------------------------------------------------------------------------
# Author   : ETH (www.eth.ch), Timo Hinzmann
# Date     : 24.04.2015
# Version  : V01 04.04.2015 Initial version.
# Usage    : python viz.py -i inbag.bag
# -------------------------------------------------------------------------

import getopt
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import roslib
import rosbag
import rospy
import sys
import time
import csv
import geo

from std_msgs.msg import String
from mpl_toolkits.mplot3d import Axes3D
from pylab import *

def main(argv):
    inputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print 'viz.py -i <inputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'viz.py -i <inputfile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
    print 'Input file is ', inputfile
    plt.close("all")

    global_position_time_ns = []
    global_position_lat    = []
    global_position_lon    = []
    global_position_alt    = []
    global_position_covs    = []

    for topic, msg, t in rosbag.Bag(inputfile).read_messages():
	#if topic == '/mavros/global_position/global':
        if topic == '/mavros/gps/fix':
            # sensor_msgs/NavSatFix
            global_position_time_ns.append(msg.header.stamp.nsecs * float(1.0e-9) + msg.header.stamp.secs)
            global_position_lat.append(msg.latitude)
            global_position_lon.append(msg.longitude)
            global_position_alt.append(msg.altitude)
	    global_position_covs.append(msg.position_covariance) # This is ENU!
    # Transform global_position_llh to global_position_ecef and global_position_ecef to global_position_ned.

    wgs84 = geo.WGS84()
    global_position = geo.GPS() 
    global_position_ecef_x = [] 
    global_position_ecef_y = [] 
    global_position_ecef_z = []
    global_position_ned_n  = []
    global_position_ned_e  = []
    global_position_ned_d  = []

    print("Found %d global_position data points" % (len(global_position_lat)))
    is_origin_set = 0
    for i in range(len(global_position_lat)):
        global_position_ecef_tmp = global_position.lla2ecef((global_position_lat[i], global_position_lon[i], global_position_alt[i]))
        if is_origin_set == 0:
           origin_ecef = global_position_ecef_tmp
           is_origin_set = 1
        global_position_ned_tmp = global_position.ecef2ned(global_position_ecef_tmp, origin_ecef)
       
        global_position_ecef_x.append(global_position_ecef_tmp[0])
        global_position_ecef_y.append(global_position_ecef_tmp[1])
        global_position_ecef_z.append(global_position_ecef_tmp[2])
        global_position_ned_n.append(global_position_ned_tmp[0])
        global_position_ned_e.append(global_position_ned_tmp[1])
        global_position_ned_d.append(global_position_ned_tmp[2])

    writer = csv.writer(open("./gpsfix_positions.csv", 'w'), delimiter=',')
    for i in range(len(global_position_ned_n)):
     writer.writerow((global_position_time_ns[i], global_position_ned_e[i], global_position_ned_n[i], -global_position_ned_d[i], 0.0, 0.0, 0.0, 1.0))

    writer = csv.writer(open("./gpsfix_covariances.csv", 'w'), delimiter= ',')
    for i in range(len(global_position_covs)):
     global_position_cov = global_position_covs[i]
     writer.writerow((global_position_time_ns[i], global_position_cov[0], global_position_cov[1], global_position_cov[2], global_position_cov[3], global_position_cov[4], global_position_cov[5], global_position_cov[6], global_position_cov[7], global_position_cov[8]))

if __name__ == "__main__":
   main(sys.argv[1:])
