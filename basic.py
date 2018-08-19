from __future__ import print_function
from datetime import datetime
import time
from dronekit import *

def log(msg, indent = 0):
	print (datetime.now().strftime('[%Y-%m-%d %H:%M:%S]: '), end='')
	print ("\t" * indent, end='')
	print (msg)

def commands_ready(drone):
	cmds = drone.commands
	cmds.download()
	cmds.wait_ready()

def show_status(drone):
	print ("=" * 50)
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

	
def prearming(drone):
	log("Prearming starts...")
	#autopilot check
	
	print("Basic pre-arm checks")
	while not drone.is_armable:
		print("Waiting for vehicle to initialise...")
        	time.sleep(5)
	
	log("3D GPS check...") 
	while(drone.gps_0.fix_type < 3):
		print("Checking GPS", vehicle.gps_0.fix_type)
		time.sleep(3)
	print("GPS 3D locked")
	log("Check for home location...")
	if not drone.home_location:
		print("Home location not found. Using current position.", drone.location.global_frame)
		currentLocation = drone.location.global_frame
		drone.home_location = currentLocation
		commands_ready(drone)
		print("New Location: ", drone.home_location)
	else:
		print("Pre-existing location: ", drone.home_location)
	print("Prearming done")
	
def arming(drone):
	log("arming demand received")
	prearming(drone)
	print("passed prearming")
	print("arming motor")
	drone.mode = VehicleMode("GUIDED")
	drone.armed = True
	while not drone.armed:
		print("waiting to be armed...")
		time.sleep(1)
	
def landing(drone):
	log("landing command received")
	drone.mode = VehicleMode("RTL")
	while not drone.mode == "RTL":
		time.sleep(0.5)
	print("Land mode set")
	while not drone.location.global_relative_frame.alt <= 0:
		print(" Altitude: ", drone.location.global_relative_frame.alt)
		time.sleep(1)
	print("landed")

	
	
def takeoff(aTargetAltitude, drone):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
	log("takeoff command received")
	while not drone.armed:
		print(" Waiting for arming...")
		time.sleep(1)

    	 print("Taking off!") 
   	 drone.simple_takeoff(aTargetAltitude)  # Take off to target altitude
	
    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
	while True:
		print(" Altitude: ", drone.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        	if drone.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
           	 	print("Reached target altitude")
            		break
        	time.sleep(1)
	
	
def turning(direction, angle, drone):
	log("turning command received")
	goal = drone.heading
	if angle > 0 and angle < 360:
		goal += angle
		goal = goal % 360
	else：
		angle = (int)input("invalid value, please re enter")
	drone.mode = VehicleMode("CIRCLE")
	if drone.heading == goal:
		log("expected position reached")
		drone.mode = VehicleMode("GUIDED")
		
	#end turning
	
	

	
	
	
	
	
#TESTING	
# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
drone = connect(connection_string, wait_ready=True)
show_status(drone)
arming(drone)
takeoff(10, drone)
landing(drone)
