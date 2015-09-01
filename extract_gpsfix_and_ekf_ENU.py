#!/usr/bin/env python

# -------------------------------------------------------------------------
# Authors  : ETH (www.eth.ch), Timo Hinzmann, Andreas Schaffner (andschaf@ethz.ch)
# Date     : 31.08.2015
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

    ekf_position_time_ns = []
    ekf_position_lat = []
    ekf_position_lon = []
    ekf_position_alt = []
    ekf_position_covs = []

    for topic, msg, t in rosbag.Bag(inputfile).read_messages():
	if topic == '/mavros/global_position/global':
 	    ekf_position_time_ns.append(msg.header.stamp.nsecs * float(1.0e-9) + msg.header.stamp.secs) 
            ekf_position_lat.append(msg.latitude)
            ekf_position_lon.append(msg.longitude)
            ekf_position_alt.append(msg.altitude)
	    ekf_position_covs.append(msg.position_covariance)

 	if topic == '/mavros/gps/fix':
            # sensor_msgs/NavSatFix
            global_position_time_ns.append(msg.header.stamp.nsecs * float(1.0e-9) + msg.header.stamp.secs)
            global_position_lat.append(msg.latitude)
            global_position_lon.append(msg.longitude)
            global_position_alt.append(msg.altitude)
	    global_position_covs.append(msg.position_covariance)

 	if topic == '/gps/fix':
            # sensor_msgs/NavSatFix
            global_position_time_ns.append(msg.header.stamp.nsecs * float(1.0e-9) + msg.header.stamp.secs)
            global_position_lat.append(msg.latitude)
            global_position_lon.append(msg.longitude)
            global_position_alt.append(msg.altitude)
	    global_position_covs.append(msg.position_covariance)  
    

    wgs84 = geo.WGS84()

    print("Found %d gpsfix data points" % (len(global_position_lat)))
    if len(global_position_lat) > 1 :

	    global_position = geo.GPS() 
	    global_position_ecef_x = [] 
	    global_position_ecef_y = [] 
	    global_position_ecef_z = []
	    global_position_ned_n  = []
	    global_position_ned_e  = []
	    global_position_ned_d  = []

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

   	    print "write gpsfix_positions.csv"
	    writer = csv.writer(open("kml_extraction/gpsfix_positions.csv", 'w'), delimiter=',')
	    for i in range(len(global_position_ned_n)):
	     writer.writerow((global_position_time_ns[i], global_position_ned_e[i], global_position_ned_n[i], -global_position_ned_d[i], 0.0, 0.0, 0.0, 1.0))

 	    print "write gpsfix_covariances.csv"
	    writer = csv.writer(open("kml_extraction/gpsfix_covariances.csv", 'w'), delimiter= ',')
	    for i in range(len(global_position_covs)):
	     global_position_cov = global_position_covs[i]
	     writer.writerow((global_position_time_ns[i], global_position_cov[0], global_position_cov[1], global_position_cov[2], global_position_cov[3], global_position_cov[4], global_position_cov[5], global_position_cov[6], global_position_cov[7], global_position_cov[8]))

	    print "write gpsfix_positions_llh.csv"
	    writer = csv.writer(open("kml_extraction/gpsfix_positions_llh.csv", 'w'), delimiter=',')
 	    for i in range(len(global_position_lat)) :
	     writer.writerow((global_position_time_ns[i], global_position_lat[i], global_position_lon[i], global_position_alt[i]))

    print ("Found %d EKF solution data points" % (len(ekf_position_lat)))
    if len(ekf_position_lat) > 1 :
	    ekf_position = geo.GPS()
	    ekf_position_ned_n = []
	    ekf_position_ned_e = []
	    ekf_position_ned_d = []

	    is_ekf_origin_set = 0
	    for i in range(len(ekf_position_lat)):
		ekf_position_ecef_tmp = ekf_position.lla2ecef((ekf_position_lat[i], ekf_position_lon[i], ekf_position_alt[i]))
	  	if is_ekf_origin_set == 0:
		    origin_ekf_ecef = ekf_position_ecef_tmp
	    	    is_ekf_origin_set = 1
		ekf_position_ned_tmp = ekf_position.ecef2ned(ekf_position_ecef_tmp, origin_ekf_ecef)
   	    
	    print "Write ekf_positions.csv"
	    writer = csv.writer(open("kml_extraction/ekf_positions.csv", 'w'), delimiter = ',')
	    for i in range(len(ekf_position_ned_n)):
	     writer.writerow((ekf_position_time_ns[i], ekf_position_ned_e[i], ekf_position_ned_n[i], -ekf_position_ned_d[i], 0.0, 0.0, 0.0, 1.0))

            print "Write ekf_covariances.csv"
	    writer = csv.writer(open("kml_extraction/ekf_covariances.csv", 'w'), delimiter = ',')
	    for i in range(len(ekf_position_covs)):
	     ekf_position_cov = ekf_position_covs[i]
	     writer.writerow((ekf_position_time_ns[i], ekf_position_cov[0], ekf_position_cov[1], ekf_position_cov[2], ekf_position_cov[3], ekf_position_cov[4], ekf_position_cov[5], ekf_position_cov[6], ekf_position_cov[7], ekf_position_cov[8]))

	    print "Write ekf_positions_llh.csv"
	    writer = csv.writer(open("kml_extraction/ekf_positions_llh.csv", 'w'), delimiter = ",")
	    for i in range(len(ekf_position_lat)):
	     writer.writerow((ekf_position_time_ns[i], ekf_position_lat[i], ekf_position_lon[i], ekf_position_alt[i]))
	    

if __name__ == "__main__":
   main(sys.argv[1:])
