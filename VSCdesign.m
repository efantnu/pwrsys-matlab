%% VSCdesign.m
% Autors: Erick Fernando Alves, Daniel dos Santos Mota
% Date: 2021-06-07
%
% This function designs the AC-side LCL filter and DC-side RC filter
% of a grid-connected thre-phase voltage source converter (VSC). Based on 
% this data, it also defines optimal controller parameters for a current 
% controller on the dq0 reference frame.
%
%% Interface
% Input Struct
% param
%   .Fn         : [Hz] rated frequency
%   .Sn         : [VA] rated apparent power
%   .Un         : [Vrms] rated line voltage LV side of trafo
%   .Udc        : [Vdc] rated dc voltage
%   .l2         : [pu] transformer short circuit reactance
%   .r2         : [pu] transformer short circuit resistance
%   .r1         : [pu] main reactor resistance 
%   .fsw        : [Hz] PWM switching frequency
%   .Ts_control : [s] sampling time of control loop
%
%% Output Struct
% filter
%   .L1         : [H] LCL converter side inductance
%   .l1         : [pu] LCL converter side inductance
%   .R1         : [Ohm] LCL converter side resistance
%   .Cf         : [F] shunt capacitance
%   .Rf         : [Ohm] damping resistor
%   .L2         : [H] trafo short circuit inductance seen from LV side
%   .fres       : [Hz] filter resonance frequency
%
%% Output Struct
% controller    : kp (1 + 1/sTi)
%   .Ti         : [s] Integrator time
%   .kp         : [pu/pu] proportional gain
%
%% DesignOk
%   1           : Ok
%   -1          : Resonance frequency of filter below 10*Fn
%   -2          : Resonance frequency of filter above 0.5*fsw

%%
function [designOk, filter, controller] = VSCdesign_01(param)
filter = {};
controller = {};

%disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('% VSCdesign');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

%% Base values
%AC
wbase = 2 * pi * param.Fn;
Zbase = param.Un^2 / param.Sn;
Ibase = param.Sn / (sqrt(3) * param.Un);
Lbase = Zbase / wbase;
Cbase = 1 / (wbase * Zbase);

%DC
Zdcbase = param.Udc^2 / param.Sn;
Cdcbase = 1 / (wbase * Zdcbase);

%

%% Brantsaeter 2015
% Passive Filter Design and Offshore Wind Turbine Modelling for System Level Harmonic Studies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (1)
% Main reactor inductance
disp('Main reactance - converter side');
filter.L1 = param.Udc / (0.8 * 2^0.5 * Ibase * param.fsw);
disp(['    L1 = ',num2str(filter.L1),' H']);
filter.l1 = filter.L1 / Lbase;
disp(['    l1 = ',num2str(filter.l1),' pu']);

filter.R1 = param.r1 * Zbase;
disp(['    R1 = ',num2str(filter.R1),' Ohm']);
filter.r1 = param.r1;
disp(['    r1 = ',num2str(filter.r1),' pu']);

%% Brantsaeter 2015
% Passive Filter Design and Offshore Wind Turbine Modelling for System Level Harmonic Studies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (2)
% Shunt capacitance
filter.Cf = 0.05 * Cbase;

% Transformer short circuit inductance seen from the LV side
filter.l2 = param.l2;
filter.r2 = param.r2;
filter.L2 = param.l2 * Lbase;
disp('Line side reactance (transformer) seen from LV towards HV');
disp(['    L2 = ',num2str(filter.L2),' H']);
disp(['    l2 = ',num2str(filter.l2),' pu']);
disp(['    r2 = ',num2str(filter.r2),' pu']);


%% Brantsaeter 2015
% Passive Filter Design and Offshore Wind Turbine Modelling for System Level Harmonic Studies
%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (5)
% Resonance frequency
filter.fres = 1 / (2 * pi) * sqrt((filter.L1 + filter.L2)/(filter.L1 * filter.L2 * filter.Cf));

if filter.fres < 10 * param.Fn 
    disp('NOT OK! Resonance frequency of filter below 10*Fn');
    disp(['fres = ',num2str(filter.fres),' Hz']);
    disp('Script stopped!');
    designOk = -1;
    return
elseif filter.fres > 0.5 * param.fsw
    disp('NOT OK! Resonance frequency of filter above 0.5*fsw');
    disp(['fres = ',num2str(filter.fres),' Hz']);
    disp('Script stopped!');
    designOk = -2;
    return
end

%% Brantsaeter 2015
% Passive Filter Design and Offshore Wind Turbine Modelling for System Level Harmonic Studies
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (6)
% Damping resistor
filter.Rf = 1 / ( 3 * 2 * pi * filter.fres * filter.Cf);

disp('Shunt branch of the filter');
disp(['    Cf = ',num2str(filter.Cf),' F']);
disp(['    Rf = ',num2str(filter.Rf),' Ohm']);
disp(['    fres = ',num2str(filter.fres),' Hz']);
disp('    OK! Resonance frequency: 10*Fn < fres < 0.5*fsw');
designOk = 1;

%% Suul 2008
% Tuning of Control Loops for Grid Connected Voltage Source Converters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equation (8)
% Sum of small time constants
% assumed filter time constant at half fres
Tsum = 2 / (2 * pi * filter.fres) + 0.5 / param.fsw;

% Equation (13)
% PI controller parameters
halfTs = 0.5*param.Ts_control;
controller.Ti = (filter.L1/filter.R1) + halfTs;
disp('Current controller PI transfer function = kp (1 + 1/(sTi))');
disp(['    Sum of small time constants Tsum = ',num2str(Tsum),' s']);
disp(['    Ti = ',num2str(controller.Ti),' s']);
controller.kp = 0.5* filter.r1 * (controller.Ti - halfTs)/(Tsum + halfTs);
disp(['    kp = ',num2str(controller.kp),' pu/pu']);
disp('    Current feedback from LV side of trafo!');

    
    









