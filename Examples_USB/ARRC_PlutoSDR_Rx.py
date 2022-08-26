import time
import math
import numpy
import cmath
import numpy as np
import adi
from pymavlink import mavutil

# Start ADI connection
sdr = adi.Pluto('ip:192.168.2.1')

time.sleep(5)

# Start connection with Pixhawk through Mavlink
ARRC_mav_connection = mavutil.mavserial('/dev/serial0', baud=115200, source_system=1, source_component=191)

# Wait for hearbeat from Pixhawk
PX4_beat = ARRC_mav_connection.wait_heartbeat(timeout=10)
print("Mavlink connection: "+ str(PX4_beat))
print("Heartbeat system: sysID %u compID %u" % (ARRC_mav_connection.target_system, ARRC_mav_connection.target_component))

# Send RPi heartbeat to confirm handshake
ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

# Wait for config message 
msg = ARRC_mav_connection.recv_match(type='ARRC_SENSOR_RAW', blocking=True)
if not msg:
    DFREQ = 3070
elif msg.get_type() == "BAD_DATA":
    DFREQ = 3070
else:
    DFREQ = msg.dfreq

sample_rate = 1e6 # Hz
center_freq = DFREQ*1000000
num_samps = 1024 # number of samples returned per call to rx()

sdr.gain_control_mode_chan0 = 'manual'
sdr.rx_hardwaregain_chan0 = 47.0 # dB
sdr.rx_lo = int(center_freq)
sdr.sample_rate = int(sample_rate)
sdr.rx_rf_bandwidth = int(sample_rate) # filter width, just set it to the same as sample rate for now
sdr.rx_buffer_size = num_samps

last_msg_sent = time.time()
last_beat = time.time()
while (True):
    samples = sdr.rx() # receive samples off Pluto

    IQ_dfreq = samples[511]
    print(IQ_dfreq)

    pwr_dB = 20*math.log10(math.sqrt(IQ_dfreq.real**2 + IQ_dfreq.imag**2))

    # Put code to save data to Pi's SD card here

    # Send Mavlink messege to Pixhawk
    if(time.time() - last_msg_sent > 0.1):
        # Pack ARRC's message and send it
        ARRC_mav_connection.mav.arrc_sensor_raw_send(10,0,center_freq/1000000,pwr_dB)

    # Send Heartbeat to Pixhawk every second
    if(time.time() - last_beat > 0.95):
        ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_beat = time.time()