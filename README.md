# NTNU Power Systems Library (pwrsys-matlab)
This is an open MATLAB and Simulink library for design and simulation of power systems with converter-interfaced equipment. It is developed by volunteer PhD fellows in the [Department of Electric Power Engineering](https://www.ntnu.edu/iel/) at NTNU.

This project started because some of us missed basic blocks and functionalitites in the Simscape Electrical Specialized Power Systems Toolbox to perform studies of power systems with high penetration of converter-interfaced equipment. 

This library is far from complete and is not yet thoroughly documented, tested nor optimized. Use it at your own risk. :-)

# Getting Started

## Matlab requirements
pwrsys-matlab Simulink Library is saved in MATLAB version 9.0.0.960322 (R2016a) Update 7, so any newer MATLAB release should be compatible with this library.

|**Required Software**|**Version**|
|---|---|
|MATLAB|Version 9.0 (R2016a)|
|Simulink|Version 8.7 (R2016a)|
|Simscape|Version 4.0 (R2016a)|
|Simscape Power Systems|Version 6.5 (R2016a)|
|Control System Toolbox|Version 10.0 (R2016a)|
 
Ensure that the correct version of MATLAB and the required toolboxes are installed by typing `ver` in the MATLAB Command Window.

Some blocks of the library are implemented in C/C++ to allow execution in real-time systems and to avoid algebraic loops. Therefore, a C/C++ compiler shall be installed. In Windows, install the [MinGW-w64 Compiler](https://se.mathworks.com/help/matlab/matlab_external/install-mingw-support-package.html). In Linux, install [gcc](https://se.mathworks.com/matlabcentral/answers/377997-how-do-i-install-gcc-compiler-on-linux).  


## Installing and testing the library

Follow these steps:
1. Clone the repository from GitHub: (https://github.com/efantnu/pwrsys-matlab)
2. Open your MATLAB startup file typing 'open startup.m' in MATLAB Command Window.
3. Add the following lines to the end of this file:
```matlab
    pwrsysPath = '<pwrsys-matlab path in your machine>';
    addpath(genpath(pwrsysPath));
```
4. Restart your MATLAB
5. Open the Simulink Library Browser by typing `slLibraryBrowser` in the MATLAB Command Window
6. Once the Simulink Library Browser opens, [refresh the Simulink Library](https://se.mathworks.com/help/simulink/slref/librarybrowser.librarybrowser2.refresh.html). The NTNU Power Systems library should now be visible. :-)
7. Generate mex files of the C/C++ based blocks:
```
mex enabled*.c
mex notch*.c
mex pi_std_antiwindup*.c
mex variable_delay_floor_ceil*.c
``` 
8. Open 'examples/oog_example.slx' and run the simulation. If it runs without erros, then the library is properly installed and working.   


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






