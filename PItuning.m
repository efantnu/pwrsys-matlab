%% PItuning.m
% Autors: Erick Fernando Alves, Daniel dos Santos Mota
% Date: 2021-09-07
%
% This function tunes the PI controllers for current and DC-link voltage
% regulation on the dq0 reference frame of a grid-connected three-phase
% voltage source converter (VSC). 
%
%%

function [Icont, Ucont] = PItuning(param, dual)
%% Interface
% Input Struct
% param
%   .Fn         : [Hz] rated frequency
%   .Sn         : [VA] rated apparent power
%   .Udc        : [Vdc] rated dc voltage
%   .Cdc        : [F] dc link capacitor
%   .dampDC     : [-] damping of dc link voltage regulator
%   .fsw        : [Hz] PWM switching frequency
%   .Ts_control : [s] sampling time of control loop
%   .L1         : [H] LCL converter side inductance
%   .l1         : [pu] LCL converter side inductance
%   .R1         : [Ohm] LCL converter side resistance
%   .Cf         : [F] shunt capacitance
%   .Rf         : [Ohm] damping resistor
%   .L2         : [H] trafo short circuit inductance seen from LV side
%   .fres       : [Hz] filter resonance frequency
% dual          : [0/1] 0 = single, 1 = dual controller

%% Output Struct
% Icont         : kp (1 + 1/sTi)
%   .Ti         : [s] Integrator time
%   .kp         : [pu/pu] proportional gain
% Ucont         : kp (1 + 1/sTi)
%   .Ti         : [s] Integrator time
%   .kp         : [pu/pu] proportional gain
%
Icont = {};
Ucont = {};

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('% PItuning');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');


%% Suul 2008
% Tuning of Control Loops for Grid Connected Voltage Source Converters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equations (8) and (10)
% Sum of small time constants
% assumed filter time constant at half fres
TsumI = 2 / (2 * pi * param.LCL.fres) + 0.5 / param.Fsw;
TsumU = TsumI + 2 / (2 * pi * param.LCL.fres);

if dual == 1
    TsumI = TsumI + 1 / (2 * pi * param.Fn);    
    TsumU = TsumU + 1 / (2 * pi * param.Fn);
end

% Equation (13)
% Current controller parameters
halfTs = 0.5*param.Ts_control;
Icont.Ti = (param.LCL.L1/param.LCL.R1) + halfTs;
disp('Current controller PI transfer function = kp (1 + 1/(sTi))');
disp(['    Sum of small time constants TsumI = ',num2str(TsumI),' s']);
disp(['    Ti = ',num2str(Icont.Ti),' s']);
Icont.kp = 0.5* param.LCL.r1 * (Icont.Ti - halfTs)/(TsumI + halfTs);
disp(['    kp = ',num2str(Icont.kp),' pu/pu']);
disp('    Current feedback from LV side of trafo!');

Idc = param.Sn / param.Udc;              
Zdc = param.Udc / Idc;
Wb = 2*pi*param.Fn;
Cdcb = 1/(Wb*Zdc);

% Equation (18)
a = 2*param.dampDC + 1;

% Equations (17)
Ucont.Ti = a^2 * TsumU;
Ucont.kp = (1/(Wb*param.Cdc/Cdcb))/(a*TsumU);
disp('DC voltage controller PI transfer function = kpdc (1 + 1/(s Tidc))');
disp(['    Sum of small time constants TsumU = ',num2str(TsumU),' s']);
disp(['    Ti = ',num2str(Ucont.Ti),' s']);
disp(['    kp = ',num2str(Ucont.kp),' pu/pu']);


    
    

















