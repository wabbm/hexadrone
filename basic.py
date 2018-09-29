from __future__ import print_function
from datetime import datetime
import time
from dronekit import *

#from pymavlink import mavutil, mavwp
#from pymavlink.dialects.v10 import ardupilotmega


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
	drone.mode = VehicleMode("LAND")
	while not drone.mode == "LAND":
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
	
	
def turning(angle = 0, drone):
	log("turning command received")
	goal = drone.heading
	if isinstance(angle, float):
		if angle > 0 and angle < 360:
			goal += angle
			goal = goal % 360
		else：
			angle = (float)input("invalid value, please re enter")
	else:
		angle = (float)input("not a float number, please re enter")
	drone.mode = VehicleMode("CIRCLE")
	while True:
		if drone.heading <= goal + 5 or drone.heading >= goal - 5:
			log("expected position reached")
			drone.mode = VehicleMode("GUIDED")
			break
	#end turning
	
def condition_yaw(heading, relative=False, turningSpeed = 0, cw 1):
	log("turning command received")
	if relative:
		is_relative=1 #yaw relative to direction of travel
	else:
		is_relative=0 #yaw is an absolute angle
	if not isinstance(heading, float):
		heading = (float)input("not a float number, please re enter")
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		heading,    # param 1, yaw in degrees
		turningSpeed,          # param 2, yaw speed deg/s
		cw,          # param 3, direction -1 ccw, 1 cw
		is_relative, # param 4, relative offset 1, absolute angle 0
		0, 0, 0)    # param 5 ~ 7 not used
	# send command to vehicle
	vehicle.send_mavlink(msg)
	log("expected position reached")
	
def changeheight(height = None, drone, w, x, y , z, roll = 0, pitch = 0, yaw = 0):
	log("changing height")
	if height is not None:
		heightChange = float(height)
		while math.isinf(heightChange) or math.isnan(heightChange) or heightChange <= 0:
			heightChange = (float)input("invalid value, please re enter")
		print("valid value received ")
		msg = vehicle.message_factory.set_attitude_target_encode(
				0, #time_boot_ms	uint32_t	Timestamp in milliseconds since system boot. Used to avoid duplicate commands. 0 to ignore.)
				0,#System ID
				0,#Component ID
				0,#Mappings: If any of these bits are set, the corresponding input should be ignored: (LSB is bit 1) bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle bit 8: attitude 
				  #Currently, throttle and attitude must be set to 0, i.e. not ignored
				  #type uint32_t, uint8_t, int8_t, int8_t
				w,
				x,
				y,
				z,# in quaternion location	
				roll, pitch, yaw,
				0) # thrust
		drone.send_mavlink(msg)
		#still need to change
	
def goto(locationX = 0, locationY = 0, height = 0, drone, groundspeed = None, airspeed = None):
	log("going toward point Global Location (relative)")
	target = LocationGlobal(float(locationX), float(locationY), float(height))
	if drone.mode != "GUIDED":
	    ans = (String)input("Drone mode isn't in guided, change or not y/n")
	    if ans is "y":
	    	drone.mode = VehicleMode("GUIDED")
	vehicle.simple_goto(target)
	log("heading toward the target location")
	
	
def set_new_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode( 
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
	    
	    
	    
	    
	    
	    
	    
	

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
