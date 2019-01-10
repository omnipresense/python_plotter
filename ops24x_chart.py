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
fft_bin_cutoff = 256

def read_plot_loop(serial_port, options):
    if options.plot_IQ_FFT:
        f, (plot1, plot2, plot3) = plt.subplots(3, 1)
    elif options.plot_FFT:
        f, (plot1) = plt.subplots(1, 1)
    else: # I,Q,IQ and T show the signal and then the FFT calculated in python
        f, (plot1, plot2) = plt.subplots(2, 1)

    # fig = plt.figure()
    # plot1 = plt.axes()
    x_axis = np.linspace(0, sample_count - 1, sample_count)
    plt.ion()
    signal_I = None
    signal_Q = None
    np_values = None
    np_values_I = None
    np_values_Q = None
    np_values_T = None
    np_values_FFT = None
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

                    # read off the wire
                    signal = None
                    if options.plot_I or options.plot_IQ or options.plot_IQ_FFT:
                        if pobj.get('I'):
                            signal_I = pobj['I']
                            signal = signal_I
                            np_values = np.array(signal_I)
                            mean = np.mean(np_values)
                            np_values = np_values - mean
                            np_values_I = np_values * hann_window
                            np_values_I = np_values_I * (3.3/4096)
                    if options.plot_Q or options.plot_IQ or options.plot_IQ_FFT:
                        if pobj.get('Q'):
                            signal_Q = pobj['Q']
                            signal = signal_Q
                            np_values = np.array(signal_Q)
                            mean = np.mean(np_values)
                            np_values = np_values - mean
                            np_values_Q = np_values * hann_window
                            np_values_Q = np_values_Q * (3.3/4096)

                    if options.plot_T:  # it's an array of [i,j] pairs
                        if pobj.get('T'):
                            signal = pobj['T']
                            np_values_T = np.array(signal)
                    elif options.plot_FFT or options.plot_IQ_FFT:
                        if pobj.get('FFT'):
                            signal = pobj['FFT']
                            np_values_FFT = np.array(signal)
                            print(signal[:5])

                    if signal is None:
                        print("Unexpected data received.")
                        #print(data_rx_str)
                        continue

                    plot1.clear()
                    plot1.grid()
                    # FFT is a special one-and-done
                    if options.plot_FFT:
                        plot1.plot(x_axis[:fft_bin_cutoff], np_values_FFT[:fft_bin_cutoff])
                        plot1.set_xlabel('BINS')
                        plot1.set_ylabel('magnitude')
                        plt.show(block=False)
                        plt.pause(0.001)
                        continue

                    legend_arr = []
                    if options.plot_I or options.plot_IQ or options.plot_IQ_FFT:
                        if signal_I is not None:
                            plot1.plot(x_axis, signal_I)
                            legend_arr.append("I")
                    # observe this is NOT an elif in order to maybe plot both I and Q....
                    if options.plot_Q or options.plot_IQ or options.plot_IQ_FFT:
                        if signal_Q is not None:
                            plot1.plot(x_axis, signal_Q)
                            legend_arr.append("Q")
                    if options.plot_T:
                        plot1.plot(x_axis, np_values_T)
                        legend_arr.append("signal")
                    plot1.set_xlabel('Samples')
                    plot1.set_ylabel('Signal amplitude')
                    plot1.legend(legend_arr)

                    # do the FFT (unless options.plot_FFT which means 'just show the sensor's output'
                    if (options.plot_IQ or options.plot_IQ_FFT) and np_values_I is not None and np_values_Q is not None:
                        # mingle the I and Q and do the FFT
                        complex_values = np_values_I + 1j * np_values_Q
                        post_fft = np.fft.fft(complex_values, NFFT)
                    elif np_values is not None and isinstance(np_values[0], np.float):  # handles I only or Q only (or anything else thats a list of floats)
                        post_fft = np.fft.rfft(np_values, NFFT)
                    elif options.plot_T and isinstance(np_values[0], (np.ndarray, np.generic)):  # handles OT (or maybe if options.plot_IQ_FFT returns this style
                        complex_values = [complex(x[0], x[1]) for x in np_values]
                        post_fft = np.fft.fft(complex_values, NFFT)
                    else:
                        post_fft = None

                    plot2.clear()
                    plot2.grid()
                    plot2.set_xlabel('BINS')
                    plot2.set_ylabel('Magnitude')
                    plot2.plot(x_axis[:fft_bin_cutoff], np.fabs(np.real(post_fft[:fft_bin_cutoff])))

                    if options.plot_IQ_FFT and np_values_FFT is not None:
                        plot3.clear()
                        plot3.grid()
                        plot3.set_xlabel('BINS')
                        plot3.set_ylabel('Magnitude')
                        plot3.plot(x_axis[:fft_bin_cutoff], np_values_FFT[:fft_bin_cutoff])
                        #print(np_values_FFT[:5])

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
    parser.add_option("-2", "--plot_IQ",
                       action="store_true",
                       dest="plot_IQ")
    parser.add_option("-T", "--plot_T",
                       action="store_true",
                       dest="plot_T")
    parser.add_option("-F", "--plot_FFT",
                       action="store_true",
                       dest="plot_FFT")
    parser.add_option("-3", "--plot_IQ_FFT",
                       action="store_true",
                       dest="plot_IQ_FFT")
    (options, args) = parser.parse_args()
    if options.plot_I is None and options.plot_Q is None and options.plot_T is None and options.plot_FFT is None:
        options.plot_I = True

    # Initialize the USB port to read from the OPS-24x module
    serial_OPS24x = serial.Serial(
        #baudrate=options.baudrate,
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
    send_OPS24x_cmd(serial_OPS24x, "\nSet Power: ", OPS24x_Power_Min)
#    send_OPS24x_cmd(serial_OPS24x, "\nSet no to Distance: ", OPS24x_Output_NoDistance)
    send_OPS24x_cmd(serial_OPS24x, "\nSet OPS24x_Wait_ms: ",OPS24x_Wait_1kms)

    if options.plot_I or options.plot_Q or options.plot_IQ or options.plot_IQ_FFT:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes Raw data: ", OPS24x_Output_Raw)
    else:
        send_OPS24x_cmd(serial_OPS24x, "\nSet no Raw data: ", OPS24x_Output_NoRaw)

    if options.plot_T:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes Time Domain data: ", OPS24x_Output_TimeSignal)
    else:
        send_OPS24x_cmd(serial_OPS24x, "\nSet No Time data: ", OPS24x_Output_NoTimeSignal)

    if options.plot_FFT or options.plot_IQ_FFT:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes FFT data: ", OPS24x_Output_FFT)
    else:
        send_OPS24x_cmd(serial_OPS24x, "\nSet No FFT data: ", OPS24x_Output_NoFFT)

    # do the work
    read_plot_loop(serial_OPS24x, options)

    # turn off all that we might have turned on
    send_OPS24x_cmd(serial_OPS24x, "\nSet no Raw data: ", OPS24x_Output_NoRaw)
    send_OPS24x_cmd(serial_OPS24x, "\nSet No Time data: ", OPS24x_Output_NoTimeSignal)
    send_OPS24x_cmd(serial_OPS24x, "\nSet No FFT data: ", OPS24x_Output_NoFFT)

    serial_OPS24x.close()
    exit()


if __name__ == "__main__":
    main()
