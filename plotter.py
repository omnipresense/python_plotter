#!/usr/bin/env python3
#####################################################
# Import time, decimal, serial, reg expr, sys
#
import sys
import serial
from optparse import OptionParser
import numpy as np
from matplotlib.widgets import Button, TextBox, CheckButtons
from platform import system
import serial.tools.list_ports
import json
import pdb
import time
import threading

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

OPS24x_CW_Sampling_Freq10 = 'S1'      # 10Ksps
OPS24x_CW_Sampling_Freq20 = 'S2'      # 20Ksps

OPS24x_CW_Sampling_Size128 = 'S('     # 128 data size
OPS24x_CW_Sampling_Size256 = 'S['     # 256 data size
OPS24x_CW_Sampling_Size512 = 'S<'     # 512 data size
OPS24x_CW_Sampling_Size1024 = 'S>'    # 1024 data size

OPS24x_FMCW_Sampling_Size128 = 's('   # 128 data size
OPS24x_FMCW_Sampling_Size256 = 's['   # 256 data size
OPS24x_FMCW_Sampling_Size512 = 's<'   # 512 data size
OPS24x_FMCW_Sampling_Size1024 = 's>'  # 1024 data size

OPS24x_CW_FFT_Scale1 = 'X1'           # FFT size = 1x data size
OPS24x_CW_FFT_Scale2 = 'X2'           # FFT size = 2x data size
OPS24x_CW_FFT_Scale4 = 'X4'           # FFT size = 4x data size
OPS24x_CW_FFT_Scale8 = 'X8'           # FFT size = 8x data size

OPS24x_FMCW_FFT_Scale1 = 'x1'         # FFT size = 1x data size
OPS24x_FMCW_FFT_Scale2 = 'x2'         # FFT size = 2x data size
OPS24x_FMCW_FFT_Scale4 = 'x4'         # FFT size = 4x data size
OPS24x_FMCW_FFT_Scale8 = 'x8'         # FFT size = 8x data size

OPS24x_Blanks_Send_Zeros = 'BZ'
OPS24x_Blanks_Send_Void = 'BV'

OPS24x_Query_Product = '?P'

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
OPS24x_Output_JSONy_data = 'OJ'      # send JSON formatted data
OPS24x_Output_No_JSONy_data = 'Oj'   # don't send JSON formatted data

OPS24x_Output_No_Binary_data = 'Ob'  # don't use binary mode

OPS24x_Output_CW_Raw = 'OR'          # send CW raw data
OPS24x_Output_No_CW_Raw = 'Or'       # don't send CW raw data
OPS24x_Output_CW_FFT = 'OF'          # send CW FFT data
OPS24x_Output_No_CW_FFT = 'Of'       # don't send CW FFT data
OPS24x_Output_FMCW_Raw = 'oR'        # send FMCW raw data
OPS24x_Output_No_FMCW_Raw = 'or'     # don't send FMCW raw data
OPS24x_Output_FMCW_FFT = 'oF'        # send FMCW FFT data
OPS24x_Output_No_FMCW_FFT = 'of'     # don't send FMCW FFT data

OPS24x_Output_CW_Mag = 'OM'          # send CW magnitudes
OPS24x_Output_No_CW_Mag = 'Om'       # send no CW magnitudes
OPS24x_Output_FMCW_Mag = 'oM'        # send FMCW magnitudes
OPS24x_Output_No_FMCW_Mag = 'om'     # send no FMCW magnitudes

# send_OPS24x_cmd: function for sending commands to the OPS-24x module
# console_msg_prefix is used only for printing out to console.
def send_OPS24x_cmd(serial_port, console_msg_prefix, ops24x_command, match_criteria = None):
    data_for_send_str = ops24x_command + '\n'
    data_for_send_bytes = str.encode(data_for_send_str)
    print(console_msg_prefix, ops24x_command)
    try:
        serial_port.write(data_for_send_bytes)
        # Initialize message verify checking
        if match_criteria is None:
            match_criteria = '{'
        ser_write_verify = False
        # Print out module response to command string
        while not ser_write_verify:
            data_rx_bytes = serial_port.readline()
            data_rx_length = len(data_rx_bytes)
            if data_rx_length != 0:
                data_rx_str = str(data_rx_bytes)
                if data_rx_str.find(match_criteria) >= 0:
                    ser_write_verify = True
                    #print("match:{0} while looking for {1}".format(data_rx_str,match_criteria))  #
                else:
                    ser_write_verify = False
                    #print("mismatch:{0} while looking for {1}".format(data_rx_str,match_criteria))  #
        return data_rx_str
    except serial.serialutil.SerialTimeoutException:
        print("Write timeout sending command:",ops24x_command)
    except serial.serialutil.SerialException:
        print("Connection Error (Write)")

fft_bin_low_cutoff = 0
fft_bin_high_cutoff = 400
graph_ylim = None
expect_data = True
data_available = threading.Event()

def read(serial_port):
    global data_rx_bytes
    global expect_data

    if expect_data:
        data_rx_bytes = serial_port.readline()
        data_available.set()


class UI:

    serial_port = None

    def init(self, serial_port):
        global is_OPS241_OPS242_A
        global is_OPS241_OPS242_B
        global is_OPS243_A
        global is_OPS243_C
        global is_doppler
        global invert_i_values
        global sample_count
        global NFFT

        self.serial_port = serial_port 

        # Initialize and query Ops24x Module

        print("\nInitializing Ops24x Module")

        send_OPS24x_cmd(serial_port, "\nSet Power Idle Mode: ", OPS24x_Power_Idle)

        is_OPS241_OPS242_A = False
        is_OPS241_OPS242_B = False
        is_OPS243_A = False
        is_OPS243_C = False
        rtn_val = send_OPS24x_cmd(serial_port, "\nQuery for Product: ", OPS24x_Query_Product, "Product")
        if rtn_val.find("243") >= 0:
            if rtn_val.find("oppler") >= 0:
                is_OPS243_A = True
            elif rtn_val.find("ombo") >= 0:
                is_OPS243_C = True
        else:
            if rtn_val.find("oppler") >= 0:
                is_OPS241_OPS242_A = True
            elif rtn_val.find("FMCW") >= 0:
                is_OPS241_OPS242_B = True

        is_doppler = False
        if options.is_doppler:
            is_doppler = True
        elif options.is_fmcw:
            is_doppler = False
        else:
            if is_OPS241_OPS242_A or is_OPS243_A:
                is_doppler = True
            else:
                is_doppler = False

        invert_i_values = False
        if is_OPS241_OPS242_A or is_OPS241_OPS242_B:
            invert_i_values = True

        if is_OPS241_OPS242_A or is_OPS243_A:
            send_OPS24x_cmd(serial_port, "\nSet no binary data: ", OPS24x_Output_No_Binary_data)

        if is_doppler:
            send_OPS24x_cmd(serial_port, "\nSet Send Speed report: ", OPS24x_Output_Speed)
        else:
            send_OPS24x_cmd(serial_port, "\nSet No Speed report: ", OPS24x_Output_No_Speed)

        if is_OPS243_C:
            if is_doppler:
                send_OPS24x_cmd(serial_port, "\nSet No Distance report: ", OPS24x_Output_No_Distance)
                send_OPS24x_cmd(serial_port, "\nSet yes CW Magnitudes: ", OPS24x_Output_CW_Mag)
            else:
                send_OPS24x_cmd(serial_port, "\nSet Send Distance report: ", OPS24x_Output_Distance)
                send_OPS24x_cmd(serial_port, "\nSet yes FMCW Magnitudes: ", OPS24x_Output_FMCW_Mag)
        else:
            send_OPS24x_cmd(serial_port, "\nSet yes Magnitudes: ", OPS24x_Output_CW_Mag)

        sample_count = 512
        NFFT = 1024
        if is_doppler:
            print("Sending CW sampling rate and size:", options.power_letter)
            send_OPS24x_cmd(serial_port, "\nSet OPS24x CW Frequency: ", OPS24x_CW_Sampling_Freq10)
            sample_count = 512
            NFFT = 512
            send_OPS24x_cmd(serial_port, "\nSet OPS24x CW Data Size: ", OPS24x_CW_Sampling_Size512)
            if is_OPS243_C:
                send_OPS24x_cmd(serial_port, "\nSet OPS24x CW FFT Scale: ", OPS24x_CW_FFT_Scale1)
        else:
            print("Sending FMCW sampling rate and size:", options.power_letter)
            send_OPS24x_cmd(serial_port, "\nSet OPS24x FMCW Frequency: ", 's=320')
            sample_count = 512
            NFFT = 1024
            send_OPS24x_cmd(serial_port, "\nSet OPS24x FMCW Data Size: ", OPS24x_FMCW_Sampling_Size512)
            if is_OPS243_C:
                send_OPS24x_cmd(serial_port, "\nSet OPS24x FMCW FFT Scale: ", OPS24x_FMCW_FFT_Scale2)

        global hann_window
        global blackman_window
        hann_window = np.hanning(sample_count)
        blackman_window = np.blackman(sample_count)

        if options.low_cutoff != ' ':
            print("Low cutoff:",options.low_cutoff)
            fft_bin_low_cutoff = int(options.low_cutoff)

        send_OPS24x_cmd(serial_port, "\nSet yes JSONy data: ", OPS24x_Output_JSONy_data, "utputFeature")

        if options.plot_I or options.plot_Q or options.plot_IQ_and_local_FFT or options.plot_IQ_only or options.plot_IQ_FFT:
            if is_doppler:
                send_OPS24x_cmd(serial_port, "\nSet yes CW Raw data: ", OPS24x_Output_CW_Raw)
                if is_OPS243_C:
                    send_OPS24x_cmd(serial_port, "\nSet no FMCW Raw data: ", OPS24x_Output_No_FMCW_Raw)
            else:
                if is_OPS243_C:
                    send_OPS24x_cmd(serial_port, "\nSet yes FMCW Raw data: ", OPS24x_Output_FMCW_Raw)
                    send_OPS24x_cmd(serial_port, "\nSet no CW Raw data: ", OPS24x_Output_No_CW_Raw)
                else:
                    send_OPS24x_cmd(serial_port, "\nSet yes FMCW Raw data: ", OPS24x_Output_CW_Raw)
        else:
            send_OPS24x_cmd(serial_port, "\nSet no CW Raw data: ", OPS24x_Output_No_CW_Raw)
            if is_OPS243_C:
                send_OPS24x_cmd(serial_port, "\nSet no FMCW Raw data: ", OPS24x_Output_No_FMCW_Raw)

        if options.plot_FFT or options.plot_IQ_FFT:
            if is_doppler:
                send_OPS24x_cmd(serial_port, "\nSet yes CW FFT data: ", OPS24x_Output_CW_FFT)
                if is_OPS243_C:
                    send_OPS24x_cmd(serial_port, "\nSet no FMCW FFT data: ", OPS24x_Output_No_FMCW_FFT)
            else:
                if is_OPS243_C:
                    send_OPS24x_cmd(serial_port, "\nSet yes FMCW FFT data: ", OPS24x_Output_FMCW_FFT)
                    send_OPS24x_cmd(serial_port, "\nSet no CW FFT data: ", OPS24x_Output_No_CW_FFT)
                else:
                    send_OPS24x_cmd(serial_port, "\nSet yes FMCW FFT data: ", OPS24x_Output_CW_FFT)
        else:
            send_OPS24x_cmd(serial_port, "\nSet no CW FFT data: ", OPS24x_Output_No_CW_FFT)
            if is_OPS243_C:
                send_OPS24x_cmd(serial_port, "\nSet no FMCW FFT data: ", OPS24x_Output_No_FMCW_FFT)

        if options.wait_letter != ' ':
            print("Sending wait argument:",options.wait_letter)
            send_OPS24x_cmd(serial_port, "\nSet OPS24x Wait ?: ", "W"+options.wait_letter)

        if options.power_letter != ' ':
            print("Sending power argument:",options.power_letter)
            send_OPS24x_cmd(serial_port, "\nSet OPS24x Power: ", "P"+options.power_letter)

        send_OPS24x_cmd(serial_port, "\nSet Power Active Mode: ", OPS24x_Power_Active, "PowerMode")

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
        title = "OPS241x Signal Plotter"
        min_I = 0
        min_Q = 0
        global graph_ylim
        global expect_data
        global plot1
        global plot2
        global plot3
        global power_level_lbl
        global fft_lbl
        global ax_max
        global ax_mid
        global ax_min
        global ax_ylim
        global ax_xlim_low
        global ax_xlim_high
        global ax_border
        global ax_pulse
        global ax_do_pulse
        global ax_charts
        global b_setting
        global b_chart
        global chk_pulse
        global chk_chart
        global chk_bin
        global do_voltage_convert
        global signal_multiply

        self.serial_port = serial_port 

        if is_doppler:
            title = "OPS24x Doppler Plotter"

        if options.plot_IQ_FFT:
            f, (plot1, plot2, plot3) = plt.subplots(3, 1)
        elif options.plot_IQ_only:
            f, (plot1) = plt.subplots(1, 1)
            plot2 = None
            plot3 = None
        elif options.plot_FFT:
            f, (plot1) = plt.subplots(1, 1)
            plot1.set_title("fft magnitudes")
            plot2 = None
            plot3 = None
        else: # I,Q,IQ and T show the signal and then the FFT calculated in python
            f, (plot1, plot2) = plt.subplots(2, 1)
            plot3 = None

        # fig = plt.figure()
        # plot1 = plt.axes()

        # top tabs
        ax_chart = plt.axes([0.15,.912,.15,.075])
        ax_setting = plt.axes([.005,.912,.15,.075])
        ax_quit = plt.axes([0.9, 0.95, 0.09, 0.05])
        b_setting = Button(ax_setting, 'Settings', color = 'darkgrey')  # , hovercolor = '')
        b_setting.on_clicked(self.open_settings)
        b_chart = Button(ax_chart, 'Chart', color = 'whitesmoke') # , hovercolor = 'whitesmoke')
        b_chart.on_clicked(self.close_settings)

        plt.figtext(0.50, 0.945, title)
        plt.axes([0,0.91,1,.0025], facecolor = 'k').get_xaxis().set_ticks([])
        #ax_label = plt.axes([0.5, 0.93, 0.09, 0.05])
        #lbl = TextBox(ax_label, "TX Power")
        power_level_lbl = plt.figtext(0.09, 0.86, "Power Level")
        power_level_lbl.set_visible(False)

        fft_base_y = 0.62
        fft_lbl = plt.figtext(0.09, fft_base_y+0.06, "FFT Chart Control")
        fft_lbl.set_visible(False)
        ax_border = plt.axes([0.1, fft_base_y-0.05, 0.80, 0.1], visible = False)
        ax_border.xaxis.set_visible(False)
        ax_border.yaxis.set_visible(False)
        ax_max = plt.axes([0.1, 0.75, 0.15, 0.1])
        ax_mid = plt.axes([0.25, 0.75, 0.15, 0.1])
        ax_min = plt.axes([0.4, 0.75, 0.15, 0.1])
        ax_ylim = plt.axes([0.25, fft_base_y-0.023, 0.075, 0.05])
        ax_xlim_low = plt.axes([0.50, fft_base_y-0.023, 0.075, 0.05])
        ax_xlim_high = plt.axes([0.75, fft_base_y-0.023, 0.075, 0.05])

        ax_charts = plt.axes([.1, .33, .2, .2])
        ax_pulse = plt.axes([0.1, 0.12, 0.2, 0.15])
        ax_do_pulse = plt.axes([.05,.025,.15,.05])

        # power buttons
        b_max = Button(ax_max, 'Max')
        b_max.on_clicked(self.power_max)
        b_mid = Button(ax_mid, 'Mid')
        b_mid.on_clicked(self.power_mid)
        b_min = Button(ax_min, 'Min')
        b_min.on_clicked(self.power_min)

        b_quit = Button (ax_quit, 'Quit')
        b_quit.on_clicked(self.do_quit)
        txt_ylim = TextBox(ax_ylim, 'Y axis limit ', initial = '')
        txt_ylim.on_submit(self.change_ylim)
        txt_xlim_low = TextBox(ax_xlim_low, 'Min Bin ', initial = '')
        txt_xlim_low.on_submit(self.change_xlim_low)
        txt_xlim_high = TextBox(ax_xlim_high, 'Max Bin ', initial = '')
        txt_xlim_high.on_submit(self.change_xlim_high)

        chk_pulse = CheckButtons(ax_pulse, ['Pulse', 'Continuous'], [False, True])
        chk_pulse.on_clicked(self.change_pulse)
        chk_chart = CheckButtons(ax_charts, ['Signal', 'Local FFT', 'Sensor FFT'], [True, True, True])

        b_do_pulse = Button(ax_do_pulse, 'Pulse')
        b_do_pulse.on_clicked(self.do_pulse)

        ax_min.set_visible(False)
        ax_mid.set_visible(False)
        ax_max.set_visible(False)
        ax_ylim.set_visible(False)
        ax_xlim_low.set_visible(False)
        ax_xlim_high.set_visible(False)
        ax_pulse.set_visible(False)
        ax_do_pulse.set_visible(False)
        ax_charts.set_visible(False)

        plot1.set_position([.15,.65,.8,.2])
        if plot2 is not None:
            plot2.set_position([.15,.35,.8,.2])
        if plot3 is not None:
            plot3.set_position([.15,.05,.8,.2])

        x_axis_raw = np.linspace(0, sample_count - 1, sample_count)
        x_axis_fft = np.linspace(0, NFFT - 1, NFFT)
        plt.ion()

        try:
            # main loop to the program

            serial_port.flushInput()
            serial_port.flushOutput()
            while serial_port.is_open:
                try:
                # fm.activateWindow()
                # fm.raise_()
                    data_rx_length = 0

                    thread =  threading.Thread(target = read(serial_port))
                    thread.start()
                    data_available.wait()
                    data_rx_length = len(data_rx_bytes)
                    if data_rx_length != 0  :
                        data_rx_str = str.rstrip(str(data_rx_bytes.decode('utf-8', 'strict')))
                        # print(data_rx_str)

                        pobj = json.loads(data_rx_str)

                        # read off the wire
                        values = None

                        if options.plot_I or options.plot_IQ_and_local_FFT or options.plot_IQ_only or options.plot_IQ_FFT:
                              if pobj.get('I'):
                                values_I = pobj['I']
                                values = values_I
                                np_values = np.array(values_I)
                                np_values_I = np_values
                                #pdb.set_trace()
                                min_I = np.min(np_values_I)
                                mean_I = np.mean(np_values_I)
                                np_values_I = np_values_I - mean_I
                                if invert_i_values:
                                    np_values_I = np_values_I * -1
                                if options.do_voltage_convert:
                                    np_values_I = np_values_I * (3.3/4096)
                                if options.signal_multiply > 1:
                                    np_values_I = np_values_I * options.signal_multiply
                                np_values_I = np_values_I * hann_window
                        if options.plot_Q or options.plot_IQ_and_local_FFT or options.plot_IQ_only  or options.plot_IQ_FFT:
                            if pobj.get('Q'):
                                values_Q = pobj['Q']
                                values = values_Q
                                np_values = np.array(values_Q)
                                np_values_Q = np_values
                                min_Q = np.min(np_values_Q)
                                mean_Q = np.mean(np_values_Q)
                                np_values_Q = np_values_Q - mean_Q
                                if options.do_voltage_convert:
                                    np_values_Q = np_values_Q * (3.3/4096)
                                if options.signal_multiply > 1:
                                    np_values_Q = np_values_Q * options.signal_multiply
                                np_values_Q = np_values_Q * hann_window

                        if options.plot_FFT or options.plot_IQ_FFT:
                            if pobj.get('FFT'):
                                values = pobj['FFT']
                                np_values_FFT = np.array(values)
                                #print(values[:5])
                        #  should it be elif or a new if.  should it be plot_foo, not show_ranges?
                        elif options.show_ranges:
                            if pobj.get('Range_Data'):
                                values = pobj['Range_Data']
                                ranges = values['Ranges']
                                print("next Range_Data.  Range (in", values['unit'], ") @ magnitude")
                                for idx, r in enumerate(ranges):
                                    print("r[", idx, "]=", r['d'], "@", r['mag'])

                        if values is None:
                            #print("Unexpected data received.")
                            print(data_rx_str)
                            continue
                        else:
                            if chk_pulse.get_status() == [True, False]:
                                expect_data = False

                        plot1.clear()
                        plot1.grid()

                        # FFT is a special one-and-done
                        if options.plot_FFT and np_values_FFT is not None:
                            plot1.plot(x_axis_fft[fft_bin_low_cutoff:fft_bin_high_cutoff], np_values_FFT[fft_bin_low_cutoff:fft_bin_high_cutoff])
                            plot1.set_xlabel('Bins')
                            plot1.set_ylabel('magnitude')
                            plot1.set_title("fft magnitudes")
                            if graph_ylim is not None:
                                plot1.set_ylim(0,graph_ylim)
                            plot1.set_xlim(fft_bin_low_cutoff,fft_bin_high_cutoff)
                            plt.show(block=False)
                            plt.pause(0.001)
                            # matplotlib._pylab_helpers.Gcf.get_active().canvas.draw_idle()
                            # matplotlib._pylab_helpers.Gcf.get_active().canvas.start_event_loop(.001)
                            continue

                        legend_arr = []
                        if options.plot_I or options.plot_IQ_and_local_FFT or options.plot_IQ_only or options.plot_IQ_FFT:
                            if np_values_I is not None:
                                plot1.plot(x_axis_raw, values_I)
                                legend_arr.append("I")
                        # observe this is NOT an elif in order to maybe plot both I and Q....
                        if options.plot_Q or options.plot_IQ_and_local_FFT or options.plot_IQ_only or options.plot_IQ_FFT:
                            if np_values_Q is not None:
                                plot1.plot(x_axis_raw, values_Q)
                                legend_arr.append("Q")

                        plot1.set_title("raw signal", loc='left')
                        plot1.set_xlabel('Samples')
                        plot1.set_ylabel('Signal amplitude')
                        if (min_I < 0) or (min_Q < 0):
                            plot1.set_ylim(-4095-10,4095+10) # the sample signal is from -4095->4095.  Never more.  Lock this one in (with margin)
                        else:
                            plot1.set_ylim(-0-10,4095+10) # the sample signal is from 0->4095.  Never more.  Lock this one in (with margin)
                        plot1.set_xlim(0,np_values_I.size)
                        plot1.legend(legend_arr, loc=1)

                        if options.plot_IQ_FFT and np_values_I is not None and np_values_Q is not None:
                            complex_values = np_values_I +1j * np_values_Q
                            post_fft_local = np.fft.fft(complex_values, NFFT)
                            plot2.clear()
                            plot2.grid()
                            plot2.set_title("fft (local)", loc='left')
                            plot2.set_xlabel('Bins')
                            plot2.set_ylabel('Magnitude')
                            if graph_ylim is not None:
                               plot2.set_ylim(0,graph_ylim)
                            plot2.set_xlim(fft_bin_low_cutoff,fft_bin_high_cutoff)
                            if post_fft_local is not None:
                                plot2.plot(x_axis_fft[fft_bin_low_cutoff:fft_bin_high_cutoff], np.abs(post_fft_local[fft_bin_low_cutoff:fft_bin_high_cutoff]))

                        if options.plot_IQ_FFT and np_values_FFT is not None:
                            plot3.clear()
                            plot3.grid()
                            plot3.set_title("fft (sensor)", loc='left')
                            plot3.set_xlabel('Bins')
                            plot3.set_ylabel('Magnitude')
                            plot3.plot(x_axis_fft[fft_bin_low_cutoff:fft_bin_high_cutoff], np_values_FFT[fft_bin_low_cutoff:fft_bin_high_cutoff])
                            if graph_ylim is not None:
                                plot3.set_ylim(0,graph_ylim)
                            plot3.set_xlim(fft_bin_low_cutoff,fft_bin_high_cutoff)
                            plt.show(block=False)
                            plt.pause(0.001)
                            continue

                        if not options.plot_IQ_only:
                            # do the FFT (unless options.plot_FFT which means 'just show the sensor's output', but they left already
                            if options.plot_IQ_and_local_FFT or options.plot_IQ_FFT:
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
                            else:
                                if np_values is not None:
                                    post_fft = np.fft.rfft(np_values, NFFT)

                            plot2.clear()
                            plot2.grid()
                            plot2.set_title("fft (local)", loc='left')
                            plot2.set_xlabel('Bins')
                            plot2.set_ylabel('Magnitude')
                            if graph_ylim is not None:
                                plot2.set_ylim(0,graph_ylim)
                            plot2.set_xlim(fft_bin_low_cutoff,fft_bin_high_cutoff)

                            if post_fft is not None:
                                plot2.plot(x_axis_fft[fft_bin_low_cutoff:fft_bin_high_cutoff], np.abs(post_fft[fft_bin_low_cutoff:fft_bin_high_cutoff]))

                            if options.plot_IQ_FFT and np_values_FFT is not None:
                                plot3.clear()
                                plot3.grid()
                                plot3.set_title("fft (sensor)", loc='left')
                                plot3.set_xlabel('Bins')
                                plot3.set_ylabel('Magnitude')
                                if graph_ylim is not None:
                                    plot3.set_ylim(0,graph_ylim)
                                plot3.set_xlim(fft_bin_low_cutoff,fft_bin_high_cutoff)
                                plot3.plot(x_axis_fft[fft_bin_low_cutoff:fft_bin_high_cutoff], np_values_FFT[fft_bin_low_cutoff:fft_bin_high_cutoff])

                        plt.show(block=False)
                        plt.pause(0.001)

                except UnicodeDecodeError:
                    print("ERROR: Prior line failed to decode. Continuing.")
                except json.decoder.JSONDecodeError:
                    print("ERROR: Prior line failed to parse. Continuing.")
                    #print(data_rx_str)  #
                    print("ERROR-end: Resuming.")
                except (ValueError, AttributeError) as err:
                    print("error in values/computation. toss and continue. details: {0}".format(err))
                    #print(data_rx_str)  #
                    values = None
                except serial.serialutil.SerialException:
                    print("Connection Error")
                    serial_port.close()
                    serial_port = serial.Serial(
                        #baudrate=options.baudrate,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        timeout=1,
                        writeTimeout=2
                    )
                    for x in range(10, 0, -1):
                        print(str(x) + ' seconds remaining to reconnect')
                        time.sleep(1)
                        try:
                            serial_port.open()
                            break
                        except:
                            pass

                    if serial_port.is_open:
                        self.init(serial_port)
                # except (portNotOpenError):
                #     print('Port has not reopened')
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

    def do_quit(self, event):
        quit(self.serial_port)

    def change_ylim(self, text):
        print('Changing Graph Y-limit')
        global graph_ylim
        if (len(text) > 0):
            graph_ylim = int(text)
        else:
            graph_ylim = None

    def change_xlim_low(self, text):
        print('Changing Low Bin Cutoff')
        global fft_bin_low_cutoff
        if (len(text) > 0):
            fft_bin_low_cutoff = int(text)
        else:
            fft_bin_low_cutoff = 0

    def change_xlim_high(self, text):
        print('Changing Bin Cutoff')
        global fft_bin_high_cutoff
        if (len(text) > 0):
            fft_bin_high_cutoff = int(text)
        else:
            fft_bin_high_cutoff = 256

    def change_pulse(self, index):
        print ('activating pulse')
        global expect_data
        i = 1
        if index == 'Pulse':
            i = 0
        # Validate State
        if chk_pulse.get_status() == [True, True]:
            chk_pulse.set_active((i+1)%2)
        elif chk_pulse.get_status() == [False, False]:
            chk_pulse.set_active(i)
        # Take Action
        if chk_pulse.get_status() == [False, True]:
            send_OPS24x_cmd(self.serial_port, "\nSet Continuous: ", OPS24x_Power_Active)
            expect_data = True
        else:
            for x in range(1,4):
                expect_data = True
                send_OPS24x_cmd(self.serial_port, "\nSet Pulse: ", OPS24x_Power_Pulse)

    def toggle_bin_start(self, index):
        global fft_bin_low_cutoff
        if chk_bin.get_status() == [True]:
            fft_bin_low_cutoff = 1
        else:
            fft_bin_low_cutoff = 0

    def do_pulse(self, event):
        # pdb.set_trace()
        global expect_data
        for x in range(1,4):
            expect_data = True
            send_OPS24x_cmd(self.serial_port, "\nSet Pulse: ", OPS24x_Power_Pulse)

    def open_settings(self, event):
        print("opening settings")
        b_setting.color = 'whitesmoke'
        b_setting.hovercolor = 'whitesmoke'
        b_chart.color = 'darkgray'
        b_chart.hovercolor = 'whitesmoke'
        power_level_lbl.set_visible(True)
        fft_lbl.set_visible(True)
        ax_min.set_visible(True)
        ax_mid.set_visible(True)
        ax_max.set_visible(True)
        ax_ylim.set_visible(True)
        ax_xlim_low.set_visible(True)
        ax_xlim_high.set_visible(True)
        ax_border.set_visible(True)
        ax_pulse.set_visible(True)
        ax_charts.set_visible(True)
        ax_do_pulse.set_visible(False)
        plot1.set_visible(False)
        try:
            plot2.set_visible(False)
            plot3.set_visible(False)
        except:
            pass

    def close_settings(self, event):
        global chk_chart
        print("closing settings")
        b_chart.color = 'whitesmoke'
        b_chart.hovercolor = 'whitesmoke'
        b_setting.color = 'darkgray'
        b_setting.hovercolor = 'darkgrey'
        power_level_lbl.set_visible(False)
        fft_lbl.set_visible(False)
        ax_min.set_visible(False)
        ax_mid.set_visible(False)
        ax_max.set_visible(False)
        ax_ylim.set_visible(False)
        ax_xlim_low.set_visible(False)
        ax_xlim_high.set_visible(False)
        ax_border.set_visible(False)
        ax_charts.set_visible(False)
        ax_pulse.set_visible(False)

        chart_space = .9
        start_height = .05

        if chk_pulse.get_status() == [True, False]:
            ax_do_pulse.set_visible(True)
            chart_space = .825
            start_height = .125

        chk_vals = chk_chart.get_status()
        count = 0
        for t in chk_vals:
            if t:
                count = 1 + count

        chart_height = (chart_space / count) - .1

        if chk_vals[2]:
            plot3.set_position([.15,start_height,.8,chart_height])
            start_height = start_height + chart_height + .1
        if chk_vals[1]:
            plot2.set_position([.15,start_height,.8,chart_height])
            start_height = start_height + chart_height + .1
        if chk_vals[0]:
            plot1.set_position([.15,start_height,.8,chart_height])

        plot1.set_visible(chk_vals[0])
        plot2.set_visible(chk_vals[1])
        plot3.set_visible(chk_vals[2])

        plt.show()

def main():
    global fft_bin_low_cutoff
    global options

    usage = "usage: %prog [options] arg"
    parser = OptionParser(usage)
    parser.add_option("-p", "--port", dest="port_name",
                      help="read data from PORTNAME")
    parser.add_option("-b", "--baud", dest="baudrate",
                      type="int", default="115200",
                      help="baud rate on serial port")
    parser.add_option("-W", "--wait_letter", dest="wait_letter",
                      default=" ",
                      help="wait value")
    parser.add_option("-P", "--power_letter", dest="power_letter",
                      default=" ",
                      help="power value")
    parser.add_option("-c", "--low_cutoff", dest="low_cutoff",
                      default=" ",
                      help="low_cutoff")
    parser.add_option("-z", "--fft_size", dest="fft_size",
                      default=" ",
                      help="fft_size")
    parser.add_option("-I", "--plot_I",
                       action="store_true",
                       dest="plot_I")
    parser.add_option("-Q", "--plot_Q",
                       action="store_true",
                       dest="plot_Q")
    parser.add_option("-1", "--plot_IQ_only",
                       action="store_true",
                       dest="plot_IQ_only")
    parser.add_option("-2", "--plot_IQ",
                       action="store_true",
                       dest="plot_IQ_and_local_FFT")
    parser.add_option("-F", "--plot_FFT",
                       action="store_true",
                       dest="plot_FFT")
    parser.add_option("-3", "--plot_IQ_FFT",
                       action="store_true",
                       dest="plot_IQ_FFT")
    parser.add_option("-r", "--ranges_noparse",
                       action="store_false",
                       dest="show_ranges")
    parser.add_option("-R", "--ranges_parse",
                       action="store_true",
                       dest="show_ranges",
                       default=True)
    parser.add_option("--CW",
                      action="store_true",
                      dest="is_doppler",
                      default=False)
    parser.add_option("--FMCW",
                      action="store_true",
                      dest="is_fmcw",
                      default=False)
    parser.add_option("--Sv", "--no-signal-voltage-convert",
                       action="store_false",
                       dest="do_voltage_convert")
    parser.add_option("--SV", "--signal-voltage-convert",
                       action="store_true",
                       dest="do_voltage_convert",
                       default=True)
    parser.add_option("--SX", dest="signal_multiply",
                      type="int", default="1",
                      help="multiplication factor to apply to signal")
    # parser.add_option("--bin_low", dest="bin_low",
    #                   type="int", default="0",
    #                   help="lowest bin to display")
    # parser.add_option("--bin_high", dest="bin_high",
    #                   type="int", default="400",
    #                   help="highest bin to display")

    (options, args) = parser.parse_args()
    if options.plot_I is None and options.plot_Q is None \
            and options.plot_FFT is None \
            and options.plot_IQ_and_local_FFT is None \
            and options.plot_IQ_only is None \
            and options.plot_IQ_FFT is None:
        options.plot_IQ_FFT = True

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
            serial_OPS24x.port = "COM3"  # maybe we'll luck out on windows
    else:
        serial_OPS24x.port = options.port_name

    serial_OPS24x.open()
    #serial_OPS24x.write(0x03)  # send a break code so that our commands might get through
    serial_OPS24x.flushOutput()
    serial_OPS24x.flushInput()

    # do the work
    ui = UI()
    ui.init(serial_OPS24x)
    ui.read_plot_loop(serial_OPS24x, options)

    # turn off all that we might have turned on
    quit(serial_OPS24x)

def quit(serial_port):
    if serial_port.isOpen() == True:
        serial_port.write(0x03)  # send a break code
        send_OPS24x_cmd(serial_port, "\nSet no CW Raw data: ", OPS24x_Output_No_CW_Raw)
        send_OPS24x_cmd(serial_port, "\nSet no CW FFT data: ", OPS24x_Output_No_CW_FFT)
        if is_OPS243_C:
            send_OPS24x_cmd(serial_port, "\nSet no FMCW Raw data: ", OPS24x_Output_No_FMCW_Raw)
            send_OPS24x_cmd(serial_port, "\nSet no FMCW FFT data: ", OPS24x_Output_No_FMCW_FFT)
        send_OPS24x_cmd(serial_port, "\nSet no JSONy data: ", OPS24x_Output_No_JSONy_data)
        send_OPS24x_cmd(serial_port, "\nSet no CW Magnitudes: ", OPS24x_Output_No_CW_Mag)
        if is_OPS243_C:
            send_OPS24x_cmd(serial_port, "\nSet no FMCW Magnitudes: ", OPS24x_Output_No_FMCW_Mag)
        serial_port.close()

    print('quiting')
    sys.exit(0)

if __name__ == "__main__":
    main()
