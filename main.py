#!/usr/bin/env python

from __future__ import print_function
from dronekit import connect, VehicleMode
from datetime import datetime

def log(msg):
	print(datetime.now().strftime('[%Y-%m-%d %H:%M:%S]: '), end='')
	print(msg)    

serialRate = 57600
serialPort = "/dev/ttyS0"

print("Connectin to Hexacopter on device=%s@%d" % (serialPort, serialRate))

#Setting up connection
drone = connect(serialPort, wait_ready=True, baud=serialRate)

print("=" * 50)

log("Drone version: %s.%s.%s" % (drone.version.major, drone.version.minor, drone.version.patch))
log("System Status: %s" % drone.system_status.state)
log("Current MODE: %s" % drone.mode.name)
log("Arm Status: %s, is Armable: %s" % (drone.armed, drone.is_armable))

log("TESTING: %s" % drone.parameters['RC3_MIN']);

drone.close()
