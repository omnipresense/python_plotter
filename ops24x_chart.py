#!/usr/bin/env python3
#####################################################
# Import time, decimal, serial, reg expr, sys
#
import sys
from time import *
import serial
import numpy as np
from platform import system
import pandas
import json

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

####################################################
#
# Description: OmniPreSense OPS24x RADAR Sensor generic signal processor
# 
#####################################################
# Modifiable parameters


# OPS24x module setting initialization constants
Fs = 10000
OPS24x_Sampling_Frequency = 'SX'  # 10Ksps
NFFT = 512  # we
OPS24x_Sampling_Size512 = 'S<'  # 10Ksps
OPS24x_Transmit_Power = 'PX'  # max power
OPS24x_Blanks_Send_Zeros = 'BZ'
OPS24x_Blanks_Send_Void = 'BV'
OPS24x_Module_Information = '??'
OPS24x_Power_Idle = 'PI'  # IDLE power
OPS24x_Power_Active = 'PA'  #  power ACTIVE
OPS24x_Power_Pulse = 'PP'  #  PULSE power

OPS24x_Output_NoSpeed = 'Os'  #  don't send speed values
OPS24x_Output_Raw = 'OR'  #  do send raw data

# Initialize the USB port to read from the OPS-24x module
serial_OPS24x = serial.Serial(
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1,
    writeTimeout=2
)
if len(sys.argv) > 1:
    serial_OPS24x.port = sys.argv[1]
else:
    if system == "Linux":
        serial_OPS24x.port = "/dev/ttyACM0"  # good for linux
    else:
        serial_OPS24x.port = "COM10 "  # maybe we'll luck out on windows

serial_OPS24x.open()
serial_OPS24x.flushInput()
serial_OPS24x.flushOutput()


# send_OPS24x_cmd: function for sending commands to the OPS-24x module
# console_msg_prefix is used only for printing out to console.
def send_OPS24x_cmd(console_msg_prefix, ops24x_command):
    data_for_send_str = ops24x_command + '\n'
    data_for_send_bytes = str.encode(data_for_send_str)
    print(console_msg_prefix, ops24x_command)
    serial_OPS24x.write(data_for_send_bytes)
    # Initialize message verify checking
    ser_message_start = '{'
    ser_write_verify = False
    # Print out module response to command string
    while not ser_write_verify:
        data_rx_bytes = serial_OPS24x.readline()
        data_rx_length = len(data_rx_bytes)
        if data_rx_length != 0:
            data_rx_str = str(data_rx_bytes)
            if data_rx_str.find(ser_message_start):
                print(data_rx_str)
                ser_write_verify = True
    return ser_write_verify


# Initialize and query Ops24x Module
print("\nInitializing Ops24x Module")
send_OPS24x_cmd("\nSet Power IDLE: ", OPS24x_Power_Idle)
send_OPS24x_cmd("\nSet Sampling Frequency: ", OPS24x_Sampling_Frequency)
send_OPS24x_cmd("\nSet Sampling Size to 512: ", OPS24x_Sampling_Size512)
send_OPS24x_cmd("\nSet Transmit Power: ", OPS24x_Transmit_Power)
send_OPS24x_cmd("\nSet: On Blank send nothing: ", OPS24x_Blanks_Send_Void)
send_OPS24x_cmd("\nSet No Speed data: ", OPS24x_Output_NoSpeed)
send_OPS24x_cmd("\nSet yes Raw data: ", OPS24x_Output_Raw)
send_OPS24x_cmd("\nDo Raw-Active: ", OPS24x_Power_Active)


try:
    # main loop to the program
    serial_OPS24x.flushInput()
    serial_OPS24x.flushOutput()
    while serial_OPS24x.is_open:
        data_rx_bytes = serial_OPS24x.readline()
        data_rx_length = len(data_rx_bytes)
        if data_rx_length != 0:
            try:
                data_rx_str = str.rstrip(str(data_rx_bytes.decode('utf-8', 'strict')))
                #print(data_rx_str)
                pobj = json.loads(data_rx_str)
                #if pobj.get('I'):
                #    np.I = pobj['I']
                if pobj.get('Q'):
                    q_signal = np.array(pobj['Q'])
                    t = np.linspace(0,NFFT-1,NFFT)
                    fig, ax = plt.subplots()
                    ax.plot(t, q_signal)
                    ax.set_xlabel('Sample Events')
                    ax.set_ylabel('Signal amplitude')
                    plt.show()
                    #send_OPS24x_cmd("\nDo Pulse: ", OPS24x_Power_Pulse)
            except UnicodeDecodeError:
                print("ERROR: Prior line failed to decode. Continuing.")
            except json.decoder.JSONDecodeError:
                print("ERROR: Prior line failed to parse. Continuing.")
                print(data_rx_str)
                print("ERROR-end: Resuming.")
##         do_dsp()
##         plot_dsp()

except KeyboardInterrupt:
    print("Keyboard interrupt received. Exiting.")
finally:
    # clean up
    serial_OPS24x.close()
