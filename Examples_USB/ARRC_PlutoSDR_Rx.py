import time
import math
import cmath
import numpy as np
import adi
from pymavlink import mavutil

# Start ADI connection

time.sleep(10)

sdr = adi.Pluto('ip:192.168.2.1')

time.sleep(5)

# Start connection with Pixhawk through Mavlink
ARRC_mav_connection = mavutil.mavserial('/dev/serial0', baud=115200, source_system=1, source_component=191)

# Wait for hearbeat from Pixhawk
PX4_beat = ARRC_mav_connection.wait_heartbeat(timeout=10)
if (PX4_beat != None):
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
else:
    DFREQ = 3070

sample_rate = 1e6 # Hz
center_freq = DFREQ*1e6
num_samps = 1024 # number of samples returned per call to rx()

sdr.gain_control_mode_chan0 = 'manual'
sdr.rx_hardwaregain_chan0 = 47.0 # dB
sdr.rx_lo = int(center_freq)
sdr.sample_rate = int(sample_rate)
sdr.rx_rf_bandwidth = int(sample_rate) # filter width, just set it to the same as sample rate for now
sdr.rx_buffer_size = num_samps

last_msg_sent = time.time()
last_beat = time.time()
#last_loop = time.time()
while (True):   # This loop runs with a sampling period of 0.003sec ish
    samples = sdr.rx()  # receive samples off Pluto
    N = len(samples)    # The center freq is at N/2-1

    #IQ_dfreq = samples[511]
    #abss = abs(IQ_dfreq)
    #print(IQ_dfreq)

    samples = samples * np.hamming(N)           # apply a Hamming window
    PSD = (np.abs(np.fft.fft(samples))/N)**2
    #pwr_dB = 10.0*np.log10(PSD[511])
    pwr_dB = 10.0*np.log10(max(PSD[int(N/2)-2:int(N/2)]))     # Narrow range where to search for peak power

    print(pwr_dB)

    # Put code to save data to Pi's SD card here

    # Send Mavlink messege to Pixhawk
    if(time.time() - last_msg_sent > 0.05):
        # Pack ARRC's message and send it
        ARRC_mav_connection.mav.arrc_sensor_raw_send(10,0,center_freq/1e6,pwr_dB)
        last_msg_sent = time.time()

    # Send Heartbeat to Pixhawk every second
    if(time.time() - last_beat > 0.95):
        ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_beat = time.time()

    #print(time.time() - last_loop)
    #last_loop = time.time()