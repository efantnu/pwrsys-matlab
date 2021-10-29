%% GC_LCLdesign.m
% Authors: Erick Fernando Alves, Daniel dos Santos Mota
% Date: 2021-10-29
%
% This function designs the LCL filter of a grid-connected thre-phase
% voltage source converter (VSC).
%
% Naming convention:
%   GC_    : grid converter, i.e., the converter connected to AC grid
%   LCL    : inductor - capacitor - inductor
%   design : self explanatory



%%
function [designOk, output] = GC_LCLdesign(param)
%% Interface
% Input Struct
% param
%   .Fn         : [Hz] rated frequency
%   .Sn         : [VA] rated apparent power
%   .Un         : [Vrms] rated line voltage LV side of trafo
%   .Udc        : [Vdc] rated dc voltage
%   .LCL.l2     : [pu] transformer short circuit reactance
%   .LCL.r2     : [pu] transformer short circuit resistance
%   .LCL.r1     : [pu] main reactor resistance 
%   .Fsw        : [Hz] PWM switching frequency
%
%% Output Struct
% output
%   .L1         : [H] LCL converter side inductance
%   .l1         : [pu] LCL converter side inductance
%   .R1         : [Ohm] LCL converter side resistance
%   .r1         : [pu] LCL converter side resistance
%   .Cf         : [F] shunt capacitance
%   .cf         : [pu] shunt capacitance
%   .Rf         : [Ohm] damping resistor
%   .rf         : [pu] damping resistor
%   .L2         : [H] trafo short circuit inductance seen from LV side
%   .l2         : [pu] trafo short circuit inductance
%   .R2         : [Ohm] trafo short circuit resistance seen from LV side
%   .r2         : [pu] trafo short circuit resistance
%   .fres       : [Hz] filter resonance frequency
%
%% DesignOk
%   1           : Ok
%   -1          : Resonance frequency of filter below 10*Fn
%   -2          : Resonance frequency of filter above 0.5*fsw
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('% GRID CONVERTER - LCL design');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

%% Base values
%AC
wbase = 2 * pi * param.Fn;
Zbase = param.Un^2 / param.Sn;
Ibase = param.Sn / (sqrt(3) * param.Un);
Lbase = Zbase / wbase;
Cbase = 1 / (wbase * Zbase);


%% Brantsaeter 2015
% Passive Filter Design and Offshore Wind Turbine Modelling for System Level Harmonic Studies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (1)
% Main reactor inductance
disp('Main reactance - converter side');
output.L1 = param.Udc / (0.8 * sqrt(2) * Ibase * param.Fsw);
disp(['    L1 = ',num2str(output.L1),' H']);
output.l1 = output.L1 / Lbase;
disp(['    l1 = ',num2str(output.l1),' pu']);

output.r1 = param.LCL.r1;
output.R1 = param.LCL.r1 * Zbase;
disp(['    R1 = ',num2str(output.R1),' Ohm']);
disp(['    r1 = ',num2str(param.LCL.r1),' pu']);

%% Brantsaeter 2015
% Passive Filter Design and Offshore Wind Turbine Modelling for System Level Harmonic Studies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (2)
% Shunt capacitance
output.cf = 0.05;
output.Cf = output.cf * Cbase;

% Transformer short circuit inductance seen from the LV side
output.l2 = param.LCL.l2;
output.L2 = param.LCL.l2 * Lbase;
disp('Line side reactance (transformer) seen from LV towards HV');
disp(['    L2 = ',num2str(output.L2),' H']);
disp(['    l2 = ',num2str(output.l2),' pu']);

output.r2 = param.LCL.r2;
output.R2 = output.r2 * Zbase;
disp(['    R2 = ',num2str(output.R2),' Ohm']);
disp(['    r2 = ',num2str(output.r2),' pu']);


%% Brantsaeter 2015
% Passive Filter Design and Offshore Wind Turbine Modelling for System Level Harmonic Studies
%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (5)
% Resonance frequency
output.fres = 1 / (2 * pi) * sqrt((output.L1 + output.L2)/(output.L1 * output.L2 * output.Cf));

if output.fres < 10 * param.Fn 
    disp('NOT OK! Resonance frequency of filter below 10*Fn');
    disp(['fres = ',num2str(output.fres),' Hz']);
    disp('Script stopped!');
    designOk = -1;
    return
elseif output.fres > 0.5 * param.Fsw
    disp('NOT OK! Resonance frequency of filter above 0.5*fsw');
    disp(['fres = ',num2str(output.fres),' Hz']);
    disp('Script stopped!');
    designOk = -2;
    return
end

%% Brantsaeter 2015
% Passive Filter Design and Offshore Wind Turbine Modelling for System Level Harmonic Studies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (6)
% Damping resistor
output.Rf = 1 / ( 3 * 2 * pi * output.fres * output.Cf);
output.rf = output.Rf / Zbase;

disp('Shunt branch of the filter');
disp(['    Cf = ',num2str(output.Cf),' F']);
disp(['    Rf = ',num2str(output.Rf),' Ohm']);
disp(['    fres = ',num2str(output.fres),' Hz']);
disp('    OK! Resonance frequency: 10*Fn < fres < 0.5*fsw');
designOk = 1;