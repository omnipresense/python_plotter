# plotter.py : a utility for displaying an OmniPreSense Radar's raw and FFT data

This is a simple rendering tool written in Python3, and uses pyserial, numpy and matplotlib.  

## Installation
The following example commands may be useful as part of installation: 
```sh
pip install serial  
pip install numpy matplotlib
pip install PyQt5 Qt5Agg
```
(Use pip3 if required for your system. May need to ignore QtAgg to install)

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

## Termination

On linux platforms, ^C or ^\ will stop programs running in the shell.  
Alternatively, from a diffferent shell, you can find the process and then kill it.
(To find a process, ```ps -ef | grep python_plotter``` will find it.
```pkill python_plotter```  may be faster.
If you are not the same user, you might need sudo.  If it doesn't respond, you might need -9.  
So, presenting that the ps found the process 1234, then ```sudo kill -9 1234``` is the most aggressive you can get.   

On Windows, ^C might work.  If not, ^Break will if you have an old keyboard, or Ctrl - Fn - S  may work.  
You can also find it in Windows Task Manager (choose it from the Ctrl-Alt-Delete menu) then find the command terminal running pythonm then press the [End Task] button
