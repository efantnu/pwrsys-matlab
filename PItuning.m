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
%   .Fsw        : [Hz] PWM switching frequency
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
if dual == 1
    disp('% PI tuning - Dual Controller');
else
    disp('% PI tuning - Single Controller');
end
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

%% Temporary comments by Mota 2021-09-09
% There was something odd for me in Eq.(17), so I redid this part
% I decided to use Eq.(20) instead of (17)
% Furthermore, Suul does not explain how to calculat Tc in (20), so I
% calculated it myself.

%Idc = param.Sn / param.Udc;              
%Zdc = param.Udc / Idc;
%Wb = 2*pi*param.Fn;
%Cdcb = 1/(Wb*Zdc);

% Equation (18)
%a = 2*param.dampDC + 1;

% Equations (17)
%Ucont.Ti = a^2 * TsumU;
%Ucont.kp = 1/(a*(param.Cdc/Cdcb)*TsumU);
%disp('DC voltage controller PI transfer function = kpdc (1 + 1/(s Tidc))');
%disp(['    Sum of small time constants TsumU = ',num2str(TsumU),' s']);
%disp(['    Ti = ',num2str(Ucont.Ti),' s']);
%disp(['    kp = ',num2str(Ucont.kp),' pu/pu']);

% Calculation of Tc in (18)
% Vdc[V] = 1/(Cdc[F]) * integral(Idc[A] dt)
% Vdc[V] = 1/(s Cdc[F]) Idc[A] : s is the Laplace variable
% vdc[pu] * Udc[V] = 1/(s Cdc[F]) * idc[pu] Idc[A] : Udc Idc are the bases
% vdc[pu] = 1/(s Tc) * idc[pu] : where Tc = Cdc[F] * Udc[V] / Idc[A] 
Idc = param.Sn / param.Udc;
Tc = param.Cdc *  param.Udc / Idc;

% Equation (18)
a = 2*param.dampDC + 1;
% Equations (20)
Ucont.Ti = a^2 * (TsumU + halfTs);
Ucont.kp = Tc / (a*(TsumU + halfTs));
disp('DC voltage controller PI transfer function = kpdc (1 + 1/(s Tidc))');
disp(['    Sum of small time constants TsumU = ',num2str(TsumU),' s']);
disp(['    Ti = ',num2str(Ucont.Ti),' s']);
disp(['    kp = ',num2str(Ucont.kp),' pu/pu']);




    
    

















