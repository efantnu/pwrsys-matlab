# NTNU Power Systems Library (pwrsys-matlab)
This is an open MATLAB and Simulink library for design and simulation of power systems with converter-interfaced equipment. It is developed by volunteer PhD fellows in the [Department of Electric Power Engineering](https://www.ntnu.edu/iel/) at NTNU.

This project started because some of us missed basic blocks and functionalitites in the Simscape Electrical Specialized Power Systems Toolbox to perform studies of power systems with high penetration of converter-interfaced equipment. 

This library is far from complete and is not yet thoroughly documented, tested nor optimized. Use it at your own risk. :-)

# Getting Started

## Matlab requirements
pwrsys-matlab Simulink Library is saved in MATLAB version R2018a, so any newer MATLAB release should be compatible with this library.

|**Required Software**|**Version**|
|---|---|
|MATLAB|Version 9.4 (R2018a)|
|Simulink|Version 9.1 (R2018a)|
|Simscape|Version 4.4 (R2018a)|
|Simscape Power Systems|Version 6.9 (R2018a)|
|Control System Toolbox|Version 10.4 (R2018a)|
 
Ensure that the correct version of MATLAB and the required toolboxes are installed by typing `ver` in the MATLAB Command Window.

There is a branch open to reach compatibility with MATLAB version R2016a. The main challenge has been implementing a substitute to the "Discrete Varying Notch" block from the Control System Toolbox. Fell free to download this branch if you do not intend to use any block that depends on it (e.g. frequency-adaptive current controllers).


## Installing the library

Follow these steps:
1. Clone the repository from GitHub: (https://github.com/efantnu/pwrsys-matlab)
2. Open your MATLAB startup file typing 'open startup.m' in MATLAB Command Window.
3. Add the following lines to the end of this file:
```matlab
    pwrsysPath = '<pwrsys-matlab path>';
    addpath(genpath(pwrsysPath));
```
4. Restart your MATLAB
5. Open the Simulink Library Browser by typing `slLibraryBrowser` in the MATLAB Command Window
6. Once the Simulink Library Browser opens, [refresh the Simulink Library](https://se.mathworks.com/help/simulink/slref/librarybrowser.librarybrowser2.refresh.html).

The NTNU Power Systems library should now be visible. :-)


## Library organization

The library is currently organized as following:
|**Sublibrary**|**File**|**Description**|
|---|---|---|
|Basic|pwrsysBasic.slx|Contains basic blocks that most other sublibraries depends on, such as PLLs, PI controllers with non-linearities and additional functionalities, filters and power transducers.|
|Power Theories|pwrsysPwrTheories.slx|Blocks to calculate power components based on the p-q and conservative power theories. It also contains adaptative moving average blocks required to apply the CPT in systems with variable frequency. For details, refer to [D. dos S. Mota and E. Tedeschi, “On Adaptive Moving Average Algorithms for the Application of the Conservative Power Theory in Systems with Variable Frequency,” Energies, vol. 14, no. 4, p. 1201, Feb. 2021](http://dx.doi.org/10.3390/en14041201).|
|Transformations|pwrsysTransf.slx|Diverse reference frame transformations with or without filtering.|
|Under Development|pwrsysAlpha.slx|Blocks under development. This is where you shall put your blocks if you want to contribute to this library.|
|Voltage and Current Control|pwrsysVICtrl.slx|Many types of voltage and current controllers for VSCs, such as ac or dc voltage, single and dual controllers in the rotating reference frame. It also includes 3-phase PWM generators.|
|Voltage Sources|pwrsysVSrc.slx|Many types of 3-phase voltage sources such as ideal, synchronous generators with turbine governor and exciter, 2-level VSC with LCL filter switched or average.|






