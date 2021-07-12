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
|Simscape Power Systems|Version 6.9(R2018a)|
 
Ensure that the correct version of MATLAB and the required toolboxes are installed by typing `ver` in the MATLAB Command Window.

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









