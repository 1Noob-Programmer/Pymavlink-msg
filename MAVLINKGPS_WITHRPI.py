from serial import Serial
from pyubx2 import UBXReader
from pymavlink import mavutil
import time

# Create the connection
#the same device --> mavproxy.py --master=udpin:127.0.0.1:14555
#master = mavutil.mavlink_connection(device="udpout:127.0.0.1:14555")
#on the serial device -1.TTL TO USB[USB ON DEVICES USING MAVPROXY]
master = mavutil.mavlink_connection(device="/dev/serial0",baudrate=115200)
#master = mavutil.mavlink_connection(device="udpout:192.168.1.127:14555")

#connection for gps data 
#stream = Serial('/dev/serial0', 115200, timeout=3)
#ubr = UBXReader(stream)
#(raw_data, parsed_data) = ubr.read()
#while True:
 #   print(parsed_data.hMSL)
while True:
    #if master:
        #print("sending heartbeat")
# Send heartbeat from a MAVLink application. 
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
#send gps data    
    master.mav.gps_input_send(
        0,  # Timestamp (micros since boot or Unix epoch)
        0,  # ID of the GPS for multiple GPS inputs
        # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
        # All other fields must be provided.
        (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
         mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY),
        100,  # GPS time (milliseconds from start of GPS week)
        0,  # GPS week number
        3,#int(parsed_data.fixType),  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        1,#int(parsed_data.lat),  # Latitude (WGS84), in degrees * 1E7
        1,#int(parsed_data.lon),  # Longitude (WGS84), in degrees * 1E7
        0,  # Altitude (AMSL, not WGS84), in m (positive for up)
        1,  # GPS HDOP horizontal dilution of position in m
        1,  # GPS VDOP vertical dilution of position in m
        0,#int(parsed_data.velN),  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0,#int(parsed_data.velE),  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0,#int(parsed_data.velD),  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0,  # GPS speed accuracy in m/s
        0,#int(parsed_data.hAcc),  # GPS horizontal accuracy in m
        0,#int(parsed_data.vAcc),  # GPS vertical accuracy in m
        7   # Number of satellites visible.
    )
    
    