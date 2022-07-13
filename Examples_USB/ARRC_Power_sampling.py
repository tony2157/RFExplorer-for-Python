
#pylint: disable=trailing-whitespace, line-too-long, bad-whitespace, invalid-name, R0204, C0200
#pylint: disable=superfluous-parens, missing-docstring, broad-except, R0801
#pylint: disable=too-many-lines, too-many-instance-attributes, too-many-statements, too-many-nested-blocks
#pylint: disable=too-many-branches, too-many-public-methods, too-many-locals, too-many-arguments

#======================================================================================
#This is an example code for RFExplorer python functionality. 
#Display amplitude in dBm and frequency in MHz of the maximum value of frequency range.
#In order to avoid USB issues, connect only RF Explorer Spectrum Analyzer to run this example
#It is not suggested to connect RF Explorer Signal Generator at the same time
#======================================================================================

import time
import RFExplorer
from RFExplorer import RFE_Common 
import math
import numpy
import scipy.optimize as optimization
from pymavlink import mavutil

#---------------------------------------------------------
# Helper functions
#---------------------------------------------------------

def PrintPeak(objAnalazyer):
    """This function prints the amplitude and frequency peak of the latest received sweep
    """
    nIndex = objAnalazyer.SweepData.Count-1
    objSweepTemp = objAnalazyer.SweepData.GetData(nIndex)
    nStep = objSweepTemp.GetPeakDataPoint()      #Get index of the peak
    fAmplitudeDBM = objSweepTemp.GetAmplitude_DBM(nStep)    #Get amplitude of the peak
    fCenterFreq = objSweepTemp.GetFrequencyMHZ(nStep)   #Get frequency of the peak
    fCenterFreq = math.floor(fCenterFreq * 10 ** 3) / 10 ** 3   #truncate to 3 decimals

    # Interpolate bins adjacent to the peak
    if(nStep != 0 & nStep != 111):
        fAmplitudeDBM_bef = objSweepTemp.GetAmplitude_DBM(nStep-1)
        fAmplitudeDBM_aft = objSweepTemp.GetAmplitude_DBM(nStep+1)
        fCenterFreq_bef = objSweepTemp.GetFrequencyMHZ(nStep-1)   #Get frequency of the peak
        fCenterFreq_bef = math.floor(fCenterFreq_bef * 10 ** 3) / 10 ** 3   #truncate to 3 decimals
        fCenterFreq_aft = objSweepTemp.GetFrequencyMHZ(nStep+1)   #Get frequency of the peak
        fCenterFreq_aft = math.floor(fCenterFreq_aft * 10 ** 3) / 10 ** 3   #truncate to 3 decimals
        
        ydata = numpy.array([fAmplitudeDBM_bef, fAmplitudeDBM, fAmplitudeDBM_aft])
        xdata = numpy.array([fCenterFreq_bef, fCenterFreq, fCenterFreq_aft])
        fit = numpy.polyfit(xdata, ydata, 2)

        if(fit[0] < 0):
            k = -fit[1]/(2*fit[0])
            fAmplitudeDBM = fit[0]*k**2 + fit[1]*k + fit[2]

    # Pack ARRC's message and send it
    ARRC_mav_connection.mav.arrc_sensor_raw_send(10,0,fCenterFreq,fAmplitudeDBM)


def ControlSettings(objAnalazyer):
    """This functions check user settings 
    """
    SpanSizeTemp = None
    StartFreqTemp = None
    StopFreqTemp =  None

    #print user settings
    print("User settings:" + "Span: " + str(SPAN_SIZE_MHZ) +"MHz"+  " - " + "Start freq: " + str(START_SCAN_MHZ) +"MHz"+" - " + "Stop freq: " + str(STOP_SCAN_MHZ) + "MHz")

    #Control maximum Span size
    if(objAnalazyer.MaxSpanMHZ <= SPAN_SIZE_MHZ):
        print("Max Span size: " + str(objAnalazyer.MaxSpanMHZ)+"MHz")
    else:
        objAnalazyer.SpanMHZ = SPAN_SIZE_MHZ
        SpanSizeTemp = objAnalazyer.SpanMHZ
    if(SpanSizeTemp):
        #Control minimum start frequency
        if(objAnalazyer.MinFreqMHZ > START_SCAN_MHZ):
            print("Min Start freq: " + str(objAnalazyer.MinFreqMHZ)+"MHz")
        else:
            objAnalazyer.StartFrequencyMHZ = START_SCAN_MHZ
            StartFreqTemp = objAnalazyer.StartFrequencyMHZ    
        if(StartFreqTemp):
            #Control maximum stop frequency
            if(objAnalazyer.MaxFreqMHZ < STOP_SCAN_MHZ):
                print("Max Start freq: " + str(objAnalazyer.MaxFreqMHZ)+"MHz")
            else:
                if((StartFreqTemp + SpanSizeTemp) > STOP_SCAN_MHZ):
                    print("Max Stop freq (START_SCAN_MHZ + SPAN_SIZE_MHZ): " + str(STOP_SCAN_MHZ) +"MHz")
                else:
                    StopFreqTemp = (StartFreqTemp + SpanSizeTemp)
    
    return SpanSizeTemp, StartFreqTemp, StopFreqTemp

#---------------------------------------------------------
# global variables and initialization
#---------------------------------------------------------
SERIALPORT = None    #serial port identifier, use None to autodetect  
BAUDRATE = 500000

objRFE = RFExplorer.RFECommunicator()     #Initialize object and thread
objRFE.AutoConfigure = False

#These values can be limited by specific RF Explorer Spectrum Analyzer model. 
#Check RFE SA Comparation chart from www.rf-explorer.com\models to know what
#frequency setting are available for your model
#These freq settings will be updated later in SA condition.
SPAN_SIZE_MHZ = 20           #Initialize settings
START_SCAN_MHZ = 2990
STOP_SCAN_MHZ = 3010
FFT_Points = 512   # FFT points. Must be multiple of 2.
LNA_25dB = RFE_Common.eInputStage.LNA_25dB

#---------------------------------------------------------
# Main processing loop
#---------------------------------------------------------

try:
    #Find and show valid serial ports
    objRFE.GetConnectedPorts()

    #Connect to available port
    if (objRFE.ConnectPort(SERIALPORT, BAUDRATE)): 
        print("Reseting device...")   
        #Reset the unit to start fresh
        objRFE.SendCommand("#\x03r")    
        #Wait for unit to notify reset completed
        while(objRFE.IsResetEvent):
            pass
        #Wait for unit to stabilize
        time.sleep(8)

        #Request RF Explorer configuration
        objRFE.SendCommand_RequestConfigData()

        #Wait to receive configuration and model details
        while(objRFE.ActiveModel == RFExplorer.RFE_Common.eModel.MODEL_NONE):
            objRFE.ProcessReceivedString(True)    #Process the received configuration

        #If object is an analyzer, we can scan for received sweeps
        if(objRFE.IsAnalyzer()):
            print("---- Spectrum Analyzer Example ----")
            #update frequency setting. This was added to be compatible with all RFE SA models
            #START_SCAN_MHZ = objRFE.MinFreqMHZ
            #STOP_SCAN_MHZ = START_SCAN_MHZ + 200
            #SPAN_SIZE_MHZ = 50 is the minimum span available for RF Explorer SA models

            objRFE.SendCommand("#\x05C+\x00")    # Normal mode
            #objRFE.SendCommand("#\x05C+\x02")    # Average mode
            time.sleep(3)
            objRFE.SendCommand("#\x05Cp 2")       # DSP: fast
            time.sleep(3)
            objRFE.SendCommand("#\x06Cj" + chr(int((FFT_Points & 0xFF00) >> 8)) + chr(int(FFT_Points & 0xFF)))
            time.sleep(3)
            objRFE.SendCommand("#\x05a " + str(LNA_25dB.value))  # Enable LNA 25dB
            time.sleep(3)


            # Start connection with Pixhawk through Mavlink
            ARRC_mav_connection = mavutil.mavserial('/dev/serial0', baud=115200, source_system=1, source_component=191)
            
            # Wait for hearbeat from Pixhawk
            PX4_beat = ARRC_mav_connection.wait_heartbeat()
            print("Mavlink connection: "+ str(PX4_beat))
            print("Heartbeat system: sysID %u compID %u" % (ARRC_mav_connection.target_system, ARRC_mav_connection.target_component))
            
            # Send RPi heartbeat to confirm handshake
            ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

            # Wait for config message 
            msg = ARRC_mav_connection.recv_match(type='ARRC_SENSOR_RAW', blocking=True)
            if not msg:
                START_SCAN_MHZ = 2990
                STOP_SCAN_MHZ = 3010
            elif msg.get_type() == "BAD_DATA":
                START_SCAN_MHZ = 2990
                STOP_SCAN_MHZ = 3010
            else:
                START_SCAN_MHZ = msg.dfreq - 10
                STOP_SCAN_MHZ = msg.dfreq + 10

            #Control settings
            SpanSize, StartFreq, StopFreq = ControlSettings(objRFE)
            if(SpanSize and StartFreq and StopFreq):
                last_beat = time.time()
                last_RAM_reset = time.time()
                
                #Set new configuration into device
                objRFE.UpdateDeviceConfig(StartFreq, StopFreq)
                objSweep=None
                while (True): 

                    # Clear the RAM every once in a while
                    if(time.time() - last_RAM_reset > 10):
                        objRFE.CleanSweepData()
                        objRFE.ResetInternalBuffers()
                        last_RAM_reset = time.time()

                    # Send Heartbeat to Pixhawk every second
                    if(time.time() - last_beat > 0.95):
                        ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                        last_beat = time.time()

                    # Read the RFExplorer and send data over Mavlink
                    objRFE.ProcessReceivedString(True)
                    if (objRFE.SweepData.Count>0):
                        objSweep=objRFE.SweepData.GetData(objRFE.SweepData.Count-1)
                        PrintPeak(objRFE)
            
            else:
                print("Error: settings are wrong.\nPlease, change and try again")
    else:
        print("Not Connected")
except Exception as obEx:
    print("Error: " + str(obEx))

#---------------------------------------------------------
# Close object and release resources
#---------------------------------------------------------

objRFE.Close()    #Finish the thread and close port
objRFE = None 
