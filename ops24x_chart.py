#!/usr/bin/env python3
#####################################################
# Import time, decimal, serial, reg expr, sys
#
import sys
import serial
from optparse import OptionParser
import numpy as np
from platform import system
import serial.tools.list_ports
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
sample_count = 512  # we
NFFT = sample_count * 2
OPS24x_Sampling_Size512 = 'S<'  # 10Ksps
OPS24x_Blanks_Send_Zeros = 'BZ'
OPS24x_Blanks_Send_Void = 'BV'
OPS24x_Module_Information = '??'
OPS24x_Power_Idle = 'PI'  # IDLE power
OPS24x_Power_Min = 'PN'  # Min power
OPS24x_Power_Mid = 'PD'  # Medium power
OPS24x_Power_Max = 'PX'  # Max power
OPS24x_Power_Active = 'PA'  #  power ACTIVE
OPS24x_Power_Pulse = 'PP'  #  PULSE power

OPS24x_Wait_1kms = 'WM'  # Wait one ms between readings
OPS24x_Wait_500ms = 'WD'
OPS24x_Wait_200ms = 'W2'
OPS24x_Wait_100ms = 'WC'

OPS24x_Output_NoSpeed = 'Os'  #  don't send speed values
OPS24x_Output_NoDistance = 'Od'
OPS24x_Output_Raw = 'OR'  #  for raw data
OPS24x_Output_NoRaw = 'Or'  #  for raw data
OPS24x_Output_FFT = 'OF'  #  for fft data
OPS24x_Output_NoFFT = 'Of'  #  for no fft data
OPS24x_Output_TimeSignal = 'OT' # for timedomain signal
OPS24x_Output_NoTimeSignal = 'Ot' # for timedomain signal


# send_OPS24x_cmd: function for sending commands to the OPS-24x module
# console_msg_prefix is used only for printing out to console.
def send_OPS24x_cmd(serial_port, console_msg_prefix, ops24x_command):
    data_for_send_str = ops24x_command + '\n'
    data_for_send_bytes = str.encode(data_for_send_str)
    print(console_msg_prefix, ops24x_command)
    try:
        serial_port.write(data_for_send_bytes)
        # Initialize message verify checking
        ser_message_start = '{'
        ser_write_verify = False
        # Print out module response to command string
        while not ser_write_verify:
            data_rx_bytes = serial_port.readline()
            data_rx_length = len(data_rx_bytes)
            if data_rx_length != 0:
                data_rx_str = str(data_rx_bytes)
                if data_rx_str.find(ser_message_start):
                    print(data_rx_str)
                    ser_write_verify = True
        return ser_write_verify
    except serial.serialutil.SerialTimeoutException:
        print("Write timeout sending command:",ops24x_command)


hann_window = np.hanning(sample_count)
fft_bin_cutoff = 50

def read_plot_loop(serial_port, options):
    f, (ax1, ax2) = plt.subplots(2, 1)
    # fig = plt.figure()
    # ax1 = plt.axes()
    x_axis = np.linspace(0, sample_count - 1, sample_count)
    plt.ion()
    try:
        # main loop to the program
        serial_port.flushInput()
        serial_port.flushOutput()
        while serial_port.is_open:
            data_rx_bytes = serial_port.readline()
            data_rx_length = len(data_rx_bytes)
            if data_rx_length != 0:
                try:
                    data_rx_str = str.rstrip(str(data_rx_bytes.decode('utf-8', 'strict')))
                    #print(data_rx_str)
                    pobj = json.loads(data_rx_str)

                    signal = None
                    if options.plot_I:
                        if pobj.get('I'):
                            signal = pobj['I']
                    elif options.plot_Q:
                        if pobj.get('Q'):
                            signal = pobj['Q']
                    elif options.plot_T:
                        if pobj.get('T'):
                            signal = pobj['T']
                    elif options.plot_FFT:
                        if pobj.get('FFT'):
                            signal = pobj['FFT']

                    if signal is None:
                        print("Failed to get a signal input.")

                    ax1.clear()
                    ax1.grid()
                    if options.plot_FFT:
                        ax1.plot(x_axis[:fft_bin_cutoff], signal[:fft_bin_cutoff])
                    else:
                        ax1.plot(x_axis, signal)
                    ax1.set_xlabel('Samples')
                    ax1.set_ylabel('Signal amplitude')

                    np_values = np.array(signal)
                    if not options.plot_FFT:
                        if not options.plot_T:
                            mean = np.mean(np_values)
                            np_values = np_values - mean
                            np_values = np_values * hann_window
                        post_fft = np.fft.rfft(np_values, NFFT)
                        ax2.clear()
                        ax2.grid()
                        ax2.set_xlabel('BINS')
                        if not options.plot_FFT:
                            ax2.plot(x_axis[:fft_bin_cutoff], np.fabs(np.real(post_fft[:fft_bin_cutoff])))

                    plt.show(block=False)
                    plt.pause(0.001)

                except UnicodeDecodeError:
                    print("ERROR: Prior line failed to decode. Continuing.")
                except json.decoder.JSONDecodeError:
                    print("ERROR: Prior line failed to parse. Continuing.")
                    print(data_rx_str)
                    print("ERROR-end: Resuming.")
                # except:  # catch *all* exceptions
                #     e = sys.exc_info()[0]
                #     print(e)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting.")


def main():
    usage = "usage: %prog [options] arg"
    parser = OptionParser(usage)
    parser.add_option("-p", "--port", dest="port_name",
                      help="read data from PORTNAME")
    parser.add_option("-b", "--baud", dest="baudrate",
                      default="115200",
                      help="baud rate on serial port")
    parser.add_option("-I", "--plot_I",
                       action="store_true",
                       dest="plot_I")
    parser.add_option("-Q", "--plot_Q",
                       action="store_true",
                       dest="plot_Q")
    parser.add_option("-T", "--plot_T",
                       action="store_true",
                       dest="plot_T")
    parser.add_option("-F", "--plot_FFT",
                       action="store_true",
                       dest="plot_FFT")
    (options, args) = parser.parse_args()
    if options.plot_I is None and options.plot_Q is None and options.plot_T is None and options.plot_FFT is None:
        options.plot_I = True

    # Initialize the USB port to read from the OPS-24x module
    serial_OPS24x = serial.Serial(
        baudrate=options.baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1,
        writeTimeout=2
    )
    # print([comport.device for comport in serial.tools.list_ports.comports()])

    port_value = "";
    if options.port_name is None or len(options.port_name) < 1:
        if len(serial.tools.list_ports.comports()):
            serial_OPS24x.port = serial.tools.list_ports.comports()[0].device
        elif system() == "Linux":
            serial_OPS24x.port = "/dev/ttyACM0"  # good for linux
        else:
            serial_OPS24x.port = "COM4"  # maybe we'll luck out on windows
    else:
        serial_OPS24x.port = options.port_name

    serial_OPS24x.open()
    serial_OPS24x.flushInput()
    serial_OPS24x.flushOutput()

    # Initialize and query Ops24x Module
    print("\nInitializing Ops24x Module")
    send_OPS24x_cmd(serial_OPS24x, "\nSet Power: ", OPS24x_Power_Mid)
    send_OPS24x_cmd(serial_OPS24x, "\nSet no to Distance: ", OPS24x_Output_NoDistance)
    send_OPS24x_cmd(serial_OPS24x, "\nSet OPS24x_Wait_1kms: ",OPS24x_Wait_200ms)

    if options.plot_I or options.plot_Q:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes Raw data: ", OPS24x_Output_Raw)
    elif options.plot_T:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes Time Domain data: ", OPS24x_Output_TimeSignal)
    elif options.plot_FFT:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes FFT data: ", OPS24x_Output_FFT)

    read_plot_loop(serial_OPS24x, options)

    if options.plot_I or options.plot_Q:
        send_OPS24x_cmd(serial_OPS24x, "\nSet no Raw data: ", OPS24x_Output_NoRaw)
    elif options.plot_T:
        send_OPS24x_cmd(serial_OPS24x, "\nSet No Time data: ", OPS24x_Output_NoTimeSignal)
    elif options.plot_FFT:
        send_OPS24x_cmd(serial_OPS24x, "\nSet No Time data: ", OPS24x_Output_NoFFT)

    serial_OPS24x.close()
    exit()


if __name__ == "__main__":
    main()
