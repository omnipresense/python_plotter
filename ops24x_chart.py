#!/usr/bin/env python3
#####################################################
# Import time, decimal, serial, reg expr, sys
#
import sys
import serial
from optparse import OptionParser
import numpy as np
from matplotlib.widgets import Button, TextBox
from platform import system
import serial.tools.list_ports
import json
import pdb

import matplotlib
#matplotlib.use('TkAgg')
# if the fact that the chart window pops up over everything is too irritating, there are a few approaches according to
# https://stackoverflow.com/questions/45729092/make-interactive-matplotlib-window-not-pop-to-front-on-each-update-windows-7
# 1: Easiest: don't use TkAgg backend. Use Qt5Agg.  It must be installed.  So, as admin: pip install PyQt5
matplotlib.use("Qt5Agg")  # Can't find Qt5Agg? pip install it  OR use TkAgg
# 2. the plt.pause invokes plt.show, which forces on-top-ness.
# This mypause() avoids that show() call
# def mypause(interval):
#     backend = plt.rcParams['backend']
#     if backend in matplotlib.rcsetup.interactive_bk:
#         figManager = matplotlib._pylab_helpers.Gcf.get_active()
#         if figManager is not None:
#             canvas = figManager.canvas
#             if canvas.figure.stale:
#                 canvas.draw()
#             canvas.start_event_loop(interval)
#             return

import matplotlib.pyplot as plt

####################################################
#
# Description: OmniPreSense OPS24x RADAR Sensor generic signal processor
# 
#####################################################
# Modifiable parameters


# OPS24x module setting initialization constants
#Fs = 10000
sample_count = 512
NFFT = 1024

OPS24x_Sampling_Frequency = 'SX'     # 10Ksps
OPS24x_Sampling_Size512 = 'S<'       # 512 FFT

OPS24x_Blanks_Send_Zeros = 'BZ'
OPS24x_Blanks_Send_Void = 'BV'

OPS24x_Module_Information = '??'

OPS24x_Power_Idle = 'PI'             # IDLE power
OPS24x_Power_Min = 'PN'              # Min power
OPS24x_Power_Mid = 'PD'              # Medium power
OPS24x_Power_Max = 'PX'              # Max power
OPS24x_Power_Active = 'PA'           # power ACTIVE
OPS24x_Power_Pulse = 'PP'            # PULSE power

OPS24x_Wait_1kms = 'WM'              # Wait 1000 ms between readings
OPS24x_Wait_500ms = 'WD'             # Wait 500 ms between readings
OPS24x_Wait_200ms = 'W2'             # Wait 200 ms between readings
OPS24x_Wait_100ms = 'WC'             # Wait 100 ms between readings

OPS24x_Output_Speed = 'OS'           # send speed value
OPS24x_Output_No_Speed = 'Os'        # don't send speed value
OPS24x_Output_Distance = 'OD'        # send distance values
OPS24x_Output_No_Distance = 'Od'     # don't send distance values
OPS24x_Output_Raw = 'OR'             # send raw data for distance
OPS24x_Output_No_Raw = 'Or'          # don't send raw data for distance
OPS24x_Output_FFT = 'OF'             # send fft data for distance
OPS24x_Output_No_FFT = 'Of'          # don't send fft data for distance
OPS24x_Output_TimeSignal = 'OT'      # send timedomain signal
OPS24x_Output_No_TimeSignal = 'Ot'   # don't send timedomain signal
OPS24x_Output_JSONy_data = 'OJ'      # send JSON formatted data
OPS24x_Output_No_JSONy_data = 'Oj'   # don't send JSON formatted data


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
blackman_window = np.blackman(sample_count)
fft_bin_low_cutoff = 0
fft_bin_high_cutoff = 256

class UI:

    serial_port = None

    def read_plot_loop(self, serial_port, options):
        post_fft = None
        post_fft_I = None
        post_fft_Q = None
        values_I = None
        values_Q = None
        values_T = None
        np_values = None
        np_values_I = None
        np_values_Q = None
        np_values_T = None
        np_values_FFT = None
        complex_values = None
        complex_values_I = None
        complex_values_Q = None
        complex_values_T = None

        self.serial_port = serial_port

        if options.plot_IQ_FFT:
            f, (plot1, plot2, plot3, plot4) = plt.subplots(4, 1)
        elif options.plot_FFT:
            f, (plot1) = plt.subplots(1, 1)
            plot1.set_title("fft magnitudes")
        else: # I,Q,IQ and T show the signal and then the FFT calculated in python
            f, (plot1, plot2) = plt.subplots(2, 1)

        # fig = plt.figure()
        # plot1 = plt.axes()

        plt.figtext(0.50, 0.945, "TX Power")
        #ax_label = plt.axes([0.5, 0.93, 0.09, 0.05])
        #lbl = TextBox(ax_label, "TX Power")
        ax_min = plt.axes([0.61, 0.93, 0.09, 0.05])
        ax_mid = plt.axes([0.72, 0.93, 0.09, 0.05])
        ax_max = plt.axes([0.83, 0.93, 0.09, 0.05])
        b_min = Button(ax_min, 'Min')
        b_min.on_clicked(self.power_min)
        b_mid = Button(ax_mid, 'Mid')
        b_mid.on_clicked(self.power_mid)
        b_max = Button(ax_max, 'Max')
        b_max.on_clicked(self.power_max)
        plt.subplots_adjust(top=0.9)

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

                        # read off the wire
                        values = None
                        if options.plot_I or options.plot_IQ or options.plot_IQ_FFT:
                            if pobj.get('I'):
                                values_I = pobj['I']
                                values = values_I
                                np_values = np.array(values_I)
                                np_values_I = np_values
                                np_values_I = np_values_I * 16 * (3.3/4096)
                                mean_I = np.mean(np_values_I)
                                np_values_I = np_values_I - mean_I
                                np_values_I = np_values_I * hann_window * 2
                        if options.plot_Q or options.plot_IQ or options.plot_IQ_FFT:
                            if pobj.get('Q'):
                                values_Q = pobj['Q']
                                values = values_Q
                                np_values = np.array(values_Q)
                                np_values_Q = np_values * -1
                                np_values_Q = np_values_Q * 16 * (3.3/4096)
                                mean_Q = np.mean(np_values_Q)
                                np_values_Q = np_values_Q - mean_Q
                                np_values_Q = np_values_Q * hann_window * 2
                        if options.plot_T:  # it's an array of [i,j] pairs
                            if pobj.get('T'):
                                values_T = pobj['T']
                                values = values_T
                                np_values = np.array(values_T)
                                np_values_T = np_values / 1000000
                        elif options.plot_FFT or options.plot_IQ_FFT:
                            if pobj.get('FFT'):
                                values = pobj['FFT']
                                np_values_FFT = np.array(values)
                                #print(values[:5])

                        if values is None:
                            #print("Unexpected data received.")
                            print(data_rx_str)
                            continue

                        plot1.clear()
                        plot1.grid()
                        # FFT is a special one-and-done
                        if options.plot_FFT and np_values_FFT is not None:
                            plot1.plot(x_axis[fft_bin_low_cutoff:fft_bin_high_cutoff], np_values_FFT[fft_bin_low_cutoff:fft_bin_high_cutoff])
                            plot1.set_xlabel('Bins')
                            plot1.set_ylabel('magnitude')
                            plot1.set_title("fft magnitudes")
                            plt.show(block=False)
                            plt.pause(0.001)
                            continue

                        legend_arr = []
                        if options.plot_I or options.plot_IQ or options.plot_IQ_FFT:
                            if np_values_I is not None:
                                plot1.plot(x_axis, np_values_I)
                                legend_arr.append("I")
                        # observe this is NOT an elif in order to maybe plot both I and Q....
                        if options.plot_Q or options.plot_IQ or options.plot_IQ_FFT:
                            if np_values_Q is not None:
                                plot1.plot(x_axis, np_values_Q)
                                legend_arr.append("Q")
                        if options.plot_T:
                            plot1.plot(x_axis, np_values_T)
                            legend_arr.append("signal")

                        plot1.set_title("raw signal", loc='left')
                        plot1.set_xlabel('Samples')
                        plot1.set_ylabel('Signal amplitude')
                        plot1.legend(legend_arr, loc=1)

                        if options.plot_IQ_FFT and np_values_I is not None:
                            complex_values_I = np_values_I.astype(complex)
                            post_fft_I = np.fft.fft(complex_values_I, NFFT)
                            plot2.clear()
                            plot2.grid()
                            plot2.set_title("fft_I (local)", loc='left')
                            plot2.set_xlabel('Bins')
                            plot2.set_ylabel('Magnitude')
                            if post_fft_I is not None:
                                plot2.plot(x_axis[fft_bin_low_cutoff:fft_bin_high_cutoff], np.abs(post_fft_I[fft_bin_low_cutoff:fft_bin_high_cutoff]))

                        if options.plot_IQ_FFT and np_values_Q is not None:
                            complex_values_Q = 1j * np_values_Q.astype(complex)
                            post_fft_Q = np.fft.fft(complex_values_Q, NFFT)
                            plot3.clear()
                            plot3.grid()
                            plot3.set_title("fft_Q (local)", loc='left')
                            plot3.set_xlabel('Bins')
                            plot3.set_ylabel('Magnitude')
                            if post_fft_Q is not None:
                                plot3.plot(x_axis[fft_bin_low_cutoff:fft_bin_high_cutoff], np.abs(post_fft_Q[fft_bin_low_cutoff:fft_bin_high_cutoff]))

                        if options.plot_IQ_FFT and np_values_FFT is not None:
                            plot4.clear()
                            plot4.grid()
                            plot4.set_title("fft (sensor)", loc='left')
                            plot4.set_xlabel('Bins')
                            plot4.set_ylabel('Magnitude')
                            plot4.plot(x_axis[fft_bin_low_cutoff:fft_bin_high_cutoff], np_values_FFT[fft_bin_low_cutoff:fft_bin_high_cutoff])
                            plt.show(block=False)
                            plt.pause(0.001)
                            continue

                        # do the FFT (unless options.plot_FFT which means 'just show the sensor's output', but they left already
                        if options.plot_IQ:
                            if np_values_I is not None and np_values_Q is not None:
                                # mingle the I and Q and do the FFT
                                complex_values = np_values_I + 1j * np_values_Q
                                post_fft = np.fft.fft(complex_values, NFFT)
                        elif options.plot_I:
                            if np_values_I is not None:
                                post_fft = np.fft.rfft(np_values_I, NFFT)
                        elif options.plot_Q:
                            if np_values_Q is not None:
                                post_fft = np.fft.rfft(np_values_Q, NFFT)
                        elif options.plot_T: # handles OT (or maybe if options.plot_IQ_FFT returns this style
                            if np_values_T is not None:
                                if isinstance(np_values_T[0], (np.ndarray, np.generic)):
                                    complex_values_T = [complex(x[0], x[1]) for x in np_values_T]
                                    post_fft = np.fft.fft(complex_values_T, NFFT)
                        else:
                            if np_values is not None:
                                post_fft = np.fft.rfft(np_values, NFFT)

                        plot2.clear()
                        plot2.grid()
                        plot2.set_title("fft (local)", loc='left')
                        plot2.set_xlabel('Bins')
                        plot2.set_ylabel('Magnitude')
                        if post_fft is not None:
                            plot2.plot(x_axis[fft_bin_low_cutoff:fft_bin_high_cutoff], np.abs(post_fft[fft_bin_low_cutoff:fft_bin_high_cutoff]))

                        if options.plot_IQ_FFT and np_values_FFT is not None:
                            plot3.clear()
                            plot3.grid()
                            plot3.set_title("fft (sensor)", loc='left')
                            plot3.set_xlabel('Bins')
                            plot3.set_ylabel('Magnitude')
                            plot3.plot(x_axis[fft_bin_low_cutoff:fft_bin_high_cutoff], np_values_FFT[fft_bin_low_cutoff:fft_bin_high_cutoff])

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

    def power_max(self, event):
        send_OPS24x_cmd(self.serial_port, "\nSet Power: ", OPS24x_Power_Max)

    def power_mid(self, event):
        send_OPS24x_cmd(self.serial_port, "\nSet Power: ", OPS24x_Power_Mid)

    def power_min(self, event):
        send_OPS24x_cmd(self.serial_port, "\nSet Power: ", OPS24x_Power_Min)


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
    if options.plot_I is None and options.plot_Q is None and options.plot_T is None and options.plot_FFT is None and options.plot_IQ is None and options.plot_IQ_FFT is None:
        options.plot_FFT = True

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
#    send_OPS24x_cmd(serial_OPS24x, "\nSet Max Power: ", OPS24x_Power_Max)
#    send_OPS24x_cmd(serial_OPS24x, "\nSet no Distance: ", OPS24x_Output_No_Distance)
    send_OPS24x_cmd(serial_OPS24x, "\nSet OPS24x_Wait_1kms: ", OPS24x_Wait_1kms)

    send_OPS24x_cmd(serial_OPS24x, "\nSet yes JSONy data: ", OPS24x_Output_JSONy_data)

    if options.plot_I or options.plot_Q or options.plot_IQ or options.plot_IQ_FFT:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes Raw data: ", OPS24x_Output_Raw)
    else:
        send_OPS24x_cmd(serial_OPS24x, "\nSet no Raw data: ", OPS24x_Output_No_Raw)

    if options.plot_FFT or options.plot_IQ_FFT:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes FFT data: ", OPS24x_Output_FFT)
    else:
        send_OPS24x_cmd(serial_OPS24x, "\nSet no FFT data: ", OPS24x_Output_No_FFT)

    if options.plot_T:
        send_OPS24x_cmd(serial_OPS24x, "\nSet yes Time Domain data: ", OPS24x_Output_TimeSignal)
    else:
        send_OPS24x_cmd(serial_OPS24x, "\nSet no Time Domain data: ", OPS24x_Output_No_TimeSignal)

    # do the work
    ui = UI()
    ui.read_plot_loop(serial_OPS24x, options)

    # turn off all that we might have turned on
    send_OPS24x_cmd(serial_OPS24x, "\nSet no JSONy data: ", OPS24x_Output_No_JSONy_data)
    send_OPS24x_cmd(serial_OPS24x, "\nSet no Raw data: ", OPS24x_Output_No_Raw)
    send_OPS24x_cmd(serial_OPS24x, "\nSet no FFT data: ", OPS24x_Output_No_FFT)
    send_OPS24x_cmd(serial_OPS24x, "\nSet no Time Domain data: ", OPS24x_Output_No_TimeSignal)

    serial_OPS24x.close()
    exit()


if __name__ == "__main__":
    main()
