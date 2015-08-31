#!/usr/bin/env python

# usage: python kml_from_rosbag.py -i <your_rosbag.bag>

import csv
import getopt
import numpy as np
import subprocess
import sys

import geo

def write_KML_from_CSV(csv_filename, kml_filename):

    data = csv.reader(open(csv_filename), delimiter = ',')
    f = open(kml_filename, 'w')  

    f.write("<?xml version='1.0' encoding='UTF-8'?>\n")
    f.write("<kml xmlns='http://earth.google.com/kml/2.1'>\n")
    f.write("<Document>\n")
    f.write("   <name>" + kml_filename +"</name>\n")
    for row in data:
	    f.write("   <Placemark>\n")
	    f.write("       <name>" + str(row[0]) + "</name>\n")
	    f.write("       <description>" + str(row[0]) + "</description>\n")
	    f.write("       <Point>\n")
	    f.write("           <coordinates>" + str(row[2]) + "," + str(row[1]) + "," + str(row[3]) + "</coordinates>\n")
	    f.write("       </Point>\n")
	    f.write("   </Placemark>\n")
    f.write("</Document>\n")
    f.write("</kml>\n")

def main(argv):
    ### Define Inputfile

    try: 
  	opts, args = getopt.getopt(argv, "hr:f:m:", ["rfile=","ffile=","model="])

    except getopt.GetoptError:
        print 'kml_from_rosbag.py -r <rosbagfile> -f <flagfile> -m <model>' 
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'kml_from_rosbag.py -r <rosbagfile> -f <flagfile> -m <model>' 
            sys.exit()
        elif opt in ("-r", "--rfile"):
            inputfile = arg
	elif opt in ("-f", "--ffile"):
 	    flagfile = arg
	    print "Flagfile: " + flagfile
        elif opt in ("-m", "--model"):
	    model = str(arg)
	    print "Model = " + model

    flags = __import__(flagfile.replace('.py',''))
    ### Extract gpsfix and EKF if available
    print "Extracting gpsfix and ekf solutions from rosbag ..."
    command_extract_gpsfix_and_ekf_ENU = "python extract_gpsfix_and_ekf_ENU.py -i " + str(inputfile)
    #subprocess.call(command_extract_gpsfix_and_ekf_ENU, shell=True)  

    ### Run SWE with parameters from flagfile?
    print "Running SWE app"
    command_run_swe = "rosrun swe_app swe_app cam0/image_raw:=cam1/image_raw --swe_rosbag_filename=" + str(inputfile)
    for key, value in flags.swe_app_standard.iteritems():
	command_run_swe += " --" + key + "=" + value
    for key, value in flags.swe_app_optional.iteritems():
 	command_run_swe += " --" + key + "=" + value

    if model == "TECHPOD" :
        for key, value in flags.swe_app_Techpod.iteritems():
	    command_run_swe += " --" + key + "=" + value
            n_points_to_align = 200
   	    gps_rate = 5

    if model == "SKATE" :
        for key, value in flags.swe_app_Skate.iteritems():
	    command_run_swe += " --" + key + "=" + value
	    n_points_to_align = 100
            gps_rate = 4

    else :
	print "Error: no model has been defined"
	n_points_to_align = 0

    command_run_swe += " --swe_n_points_to_align=" + str(n_points_to_align)
    #subprocess.call(command_run_swe, shell=True)

    ### Synchronize and align trajectories
    print "Synchronize and align trajectories"

    command_evaluation_basic = "rosrun swe_dataset_evaluation swe_dataset_evaluation --v=10"
    command_evaluation_basic += " --n_points_used_to_align=1000" #TODO
    command_evaluation_swe = command_evaluation_basic
    for key, value in flags.swe_dataset_evaluation.iteritems():
        command_evaluation_swe += " --" + key + "=" + value

    #subprocess.call(command_evaluation_swe, shell=True)
    
    ### Write swe_positions_llh.csv  
    gps_positions_llh = np.loadtxt(open("gpsfix_positions_llh.csv", "rb"), delimiter=",", skiprows=1)
    swe_positions_enu = np.loadtxt(open("swe_aligned-estimates.csv", "rb"), delimiter=",", skiprows=1)

    swe_positions_enu_start = swe_positions_enu[0,:]
    gps_positions_llh_start = gps_positions_llh[0,:]
    
    t_start_swe = swe_positions_enu_start[0] * 1.0e-9
    t_gps = gps_positions_llh_start[0]
    idt = 0

    while t_gps < t_start_swe :
	idt+=1
	gps_idt = gps_positions_llh[idt,:]
	t_gps = gps_idt[0]

    idt -= 1
    origin_llh = gps_positions_llh[idt]
    origin_llh = [origin_llh[1], origin_llh[2], origin_llh[3]]
    origin_ecef = geo.GPS()
    origin_ecef = origin_ecef.lla2ecef(origin_llh)

    swe_positions_llh = []

    for idt in range(len(swe_positions_enu)) :
	swe_positions_enu_temp = swe_positions_enu[idt]
	swe_ned = [swe_positions_enu_temp[2], swe_positions_enu_temp[1], -swe_positions_enu_temp[3]]
	swe_ecef = geo.GPS()
        swe_ecef = swe_ecef.ned2ecef(swe_ned, origin_ecef)
        swe_llh = geo.GPS()
	swe_llh = swe_llh.ecef2lla(swe_ecef)
        swe_positions_llh_temp = [swe_positions_enu_temp[0]*1.0e-9, swe_llh[0], swe_llh[1], swe_llh[2]]
	swe_positions_llh.append(swe_positions_llh_temp)

    writer = csv.writer(open("./swe_positions_llh.csv", 'w'), delimiter = ',')
    for i in range(len(swe_positions_llh)):
     swe_position_llh = swe_positions_llh[i]
     writer.writerow((swe_position_llh[0], swe_position_llh[1], swe_position_llh[2], swe_position_llh[3]))

    ### Write KML
    write_KML_from_CSV("./swe_positions_llh.csv", "swe_positions_llh.kml")  
    write_KML_from_CSV("./gpsfix_positions_llh.csv", "gps_positions_llh.kml")
    if model == "TECHPOD" :
     write_KML_from_CSV("./ekf_positions_llh.csv", "ekf_positions_llh.kml")

if __name__ == "__main__":
    main(sys.argv[1:])
