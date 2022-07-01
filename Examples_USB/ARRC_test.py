import time
import math
from pymavlink import mavutil

ARRC_mav_connection = mavutil.mavlink_connection('udp:localhost:14570', source_system=1, source_component=191) #'udpin:127.0.0.1:14551'
#ARRC_mav_connection = mavutil.mavserial('/dev/serial0', baud=115200, source_system=1, source_component=191)

yay = ARRC_mav_connection.wait_heartbeat()
print("Heartbeat system: sysID %u compID %u" % (ARRC_mav_connection.target_system, ARRC_mav_connection.target_component))

ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

#ARRC_mav_connection.mav.request_data_stream_send(ARRC_mav_connection.target_system, ARRC_mav_connection.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,1,1)
print("Mavlink connection: "+ str(yay))

last_beat = time.time()
while(True):
    if(time.time() - last_beat > 0.95):
        ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_beat = time.time()

    ARRC_mav_connection.mav.arrc_sensor_raw_send(10,0,2,1,1)
    #ARRC_mav_connection.mav.uavionix_adsb_transceiver_health_report_send(1)
    time.sleep(0.1)