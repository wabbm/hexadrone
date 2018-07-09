#!/usr/bin/env python

from __future__ import print_function
from dronekit import connect, VehicleMode
from datetime import datetime
import time

#global variable
serialRate = 57600
serialPort = "/dev/ttyS0"

#global functions
def log(msg, indent = 0):
	print(datetime.now().strftime('[%Y-%m-%d %H:%M:%S]: '), end='')
	print("\t" * indent, end='')
	print(msg)

def commands_ready(drone):
	cmds = drone.commands
	cmds.download()
	cmds.wait_ready()

print("Connectin to Hexacopter on device=%s@%d" % (serialPort, serialRate))
#Setting up connection
drone = connect(serialPort, wait_ready=True, baud=serialRate)

print("=" * 50)
log("Current Vehicle Status.")
log("Drone version: %s.%s.%s" % (drone.version.major, drone.version.minor, drone.version.patch))
log("System Status: %s" % drone.system_status.state)
log("Current MODE: %s" % drone.mode.name)
log("Arm Status: %s, is Armable: %s" % (drone.armed, drone.is_armable))
print("=" * 50)
log("Current Vehicle Data.")
log("Attitude: %s" % drone.attitude)
log("Velocity: %s" % drone.velocity)
log("GPS: %s" % drone.gps_0)
log("Battery: %s" % drone.battery)
log("EKF GYRO Status: %s" % drone.ekf_ok)
log("Heading: %s" % drone.heading)
log("Ground Speed: %s" % drone.groundspeed)
log("%s" % drone.location.global_frame)

print("=" * 50)
print(" " * 15, "READY FOR INSTRUCTION", " " * 15)
print("=" * 50)

def prearmCheck():
	indent_level = 0
	log("Autopilot Initalization.")

	indent_level += 1

	#Ensure GPS is functional
	log("GPS Check:", indent_level)
	indent_level += 1
	#Getting GPS Status stat >= 3. 3D Lock
	log("Current GPS Status: %s" % drone.gps_0.fix_type, indent_level)
	while drone.gps_0.fix_type < 3:
		log("GPS not fixed: %s" % drone.gps_0.fix_type, indent_level + 1)
		time.sleep(5)
	log("Current GPS Status: OKAY.", indent_level)

	indent_level -= 1

	log("Home location:", indent_level)
	#checking for home location
	if not drone.home_location:
		indent_level += 1
		log("Home location not found. Using current position. %s" % drone.location.global_frame, indent_level)
		#Getting current location
		currentLocation = drone.location.global_frame
		#Setting current location
		drone.home_location = currentLocation
		#Confirmation
		commands_ready(drone)
		log("New Location: %s" % drone.home_location, indent_level)
		indent_level -= 1
	else:
		log("Pre-existing location: %s" % drone.home_location)

	indent_level -= 1

	#Set Autopilot Mode
	log("Set GUIDED Mode for Take Off...")
	drone.mode = VehicleMode("GUIDED")
	while not drone.mode == "GUIDED":
		time.sleep(0.5)

	log("Mode set.")

	
#TODO: TEMPORARY INTERFACE
#USER COMMAND HANDLER

while True:
	inp = raw_input(">>> ")
	
	if inp == "prearm":
		prearmCheck()
	elif inp == "exit":
		#TODO: Ensure Vehicle is on the ground.
		log("EXIT.")
	else:
		log("INVALID Command.")
	

drone.close()
