# plotter.py : a utility for displaying an OmniPreSense Radar's raw and FFT data

This is a simple rendering tool written in Python3, and uses pyserial, numpy and matplotlib.  

## Installation
The following example commands may be useful as part of installation: 
```sh
pip install serial  
pip install numpy matplotlib
pip install PyQt5 Qt5Agg
```
(Use pip3 if required for your system.)

## Execution
```
python plotter.py 
```
(Use python3 if python 3 is not your system default)

plotter.py attempts to use the first available serial port.  Use -p _port_ to 
specify a different port if your sensor is not on the first available port

The program renders data depending on the command flag passed.  For example:  
-1 = fetch only the raw (I and Q) data from the sensor and only render that data  
-2 = fetch raw data and render it.  Then do the fft in numpy locally and render it   
-3 = above, and also fetch the sensor's calculation of the fft on the signal and render that also  
--FMCW = force use of FMCW chirp behavior  
(The program may also change behaviour depending on the attached board)

Note: running plotter.py -help will list an extensive list of configuration parameters.  

Note that this program uses matplotlib's chart and widgets, so it must be run with a graphics display available.  
This is the default on Windows and Mac.  If using a Raspberry Pi shell terminal, you can use XWindows remote display abilities, 
or connect to the pi via VNC rather than ssh. 

Depending on your system configuration (such as Windowx 10's WSL), you may need to change matplotlib's graphical backend dependency.
