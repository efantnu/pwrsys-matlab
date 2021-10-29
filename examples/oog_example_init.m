%% REMOVE THAT
Ts_sim = 100e-6; %inv(30*60*60); %100E-6;  % simulation time step [s] for physical subsystem
% Ts = 70E-6; % simulation time step [s]
%Ts_sim = 10e-6; % Simulation time step [s] for simulated subsystem and grid emulator.
Ts_phy = 1*Ts_sim; %2*Ts_sim; % Simulation time step [s] for simulated subsystem and grid emulator.

k_Ts = 1; % ratio between simulation time step and control time step
Ts_c = k_Ts * Ts_phy; % control time step [s]
Ts = Ts_c;

Ttot = 10;

%% System
systemBase = {};

% System baseValues
%systemBase.Sn = 60e3; % VA
systemBase.Sn = 2*44e6; % VA
%systemBase.Vn = 400;  % V rms phase to phase
systemBase.Vn = 11000;  % V rms phase to phase
%systemBase.fn = 50; % Hz
systemBase.fn = 60; % Hz

systemBase.Vn_phase = systemBase.Vn/sqrt(3); % V
systemBase.In   = systemBase.Sn/(systemBase.Vn*sqrt(3)) ; % A
systemBase.Zn   = systemBase.Vn*systemBase.Vn/systemBase.Sn ; % Ohm
systemBase.Rn   = systemBase.Zn; % Ohm
systemBase.Wn   = 2*pi()*systemBase.fn ; % rad/s
systemBase.Ln   = systemBase.Zn/systemBase.Wn ; % H
systemBase.Cn   = 1/(systemBase.Zn*systemBase.Wn) ; % F


%%%% SINTEF SM - BEGIN %%%%
%% SM

% Change to pu- input 
sm.fn=systemBase.fn; % [Hz] Rated electric frequency 
sm.Vn=systemBase.Vn; % [Vrms] Rated line voltage 
sm.Sn=systemBase.Sn; % [VA] Rated VA
sm.Vn_phase = sm.Vn/sqrt(3); % V
sm.In   = sm.Sn/(sm.Vn*sqrt(3)) ; % A
sm.Zn   = sm.Vn*sm.Vn/sm.Sn ; % Ohm
sm.Rn   = sm.Zn; % Ohm
sm.Wn   = 2*pi()*sm.fn ; % rad/s
sm.Ln   = sm.Zn/sm.Wn ; % H
sm.Cn   = 1/(sm.Zn*sm.Wn) ; % F

sm.r_pu=0.0242; % [pu] SM stator resistance
sm.x_pu=0.3834; %0.2; % [pu] SM stator reactance

sm.R=sm.r_pu*sm.Rn; % 5*0.0064; %Ohm
sm.L=sm.x_pu*sm.Ln; %3*2.0368E-4; %H

sm.phaseInitial=0; %[rad] Initial phase angle of phase 1 (electric)

sm.inertiaSeconds= 5;      % [s] Mechanical inertia (=wJ/T)
sm.mechanicalDamping=0.01; % [pu] Mechanical damping 
sm.initialSpeed=1 ;        % [pu] Initial turbine speed

sm.minSpeedForTorqueCalc=0.1; % [pu] Miniumum speed to use when calculating torque based on power and speed (prevent divison by zero)
sm.speedBlockingLimit=1.3 ; % [pu] Override limit on speed calculation. Speed will be forced below limit

%% Measurements
sm.V_measure_T=0.005; % [s] LP filter time constant for SM volatge measurement
sm.I_measure_T=0.005; % [s] LP filter time constant for SM current measurement
sm.P_measure=0.04; % LP filter time constant for SM power measurement
sm.Q_measure=0.04; % LP filter time constant for SM reactive power measurement

%% SM Governor
governor.kP = 20;% [pu] Proportonal gain  u=(Kp+kI/s+ s kd)e
governor.kI = governor.kP/10; % [pu] Integral gain  u=(Kp+kI/s + s kd)e
governor.kd = 0 ; % [pu] Derivative gain
governor.derivativeLP = 0.001 ; % [s] Low pass filter before derivative in PID

governor.speedErrMax=1 ; % [pu] Maximum speed error limit
governor.speedErrMin=-1 ; % [pu] Minimum speed error limit

governor.init = 0; %[pu] Initial output command at time zero
governor.saturationUpper=1; %[pu] Upper saturation limit for controller (anti-wind-up and output limit)
governor.saturationLower=-1; %[pu] Lower saturation limit for controller (anti-wind-up and output limit)

governor.powerMeasSaturationUpper=1; %[pu] Upper saturation limit for for power measurement
governor.powerMeasSaturationLower=-1; %[pu] Lower saturation limit for power measurement
governor.LPfilterPowerMeasurementDroop.T=0.4; % [s] LP filter time constant for power measurement used for droop

governor.LPfilterFrequencyMeasurement.T=0.002; % [s] LP filter time constant for frequency measurement

governor.powerDroop.K=0;%0.06; % [pu]
governor.droopOutputMax=0.2 ; % Maximum speed ref modification from droop
governor.droopOutputMin=-0.2 ;  % Minimum speed ref modification from droop

governor.powerControl.errMax=0.2; % [pu] Limiter on power error signal
governor.powerControl.errMin=-0.2; % [pu] Limiter on power error signal
governor.powerControl.kP=5 ;% []  Proportional gain power controller
governor.powerControl.kI=0.1; % [] Integral gain power controller
governor.powerControl.saturationUpper=0.1 ; % [] Maximum speed ref modification from power controller
governor.powerControl.saturationLower=-0.1; % [] Minimum speed ref modification from power controller
governor.LPfilterPowerMeasurementPowerControl.T=0.001 ; % [s] LP filter time constant for power measurement used for power control


%% SM Automatic Voltage Regulator

avr.fieldWinding.T=0.1; % [s] Time constant representing the field winding time constant

avr.LPfilterVoltageMeasurement.T= 0.05 ; % [s] Time constant for LP filter of voltage measurement

avr.ErrMax=0.5; % Maximum error to controller input
avr.ErrMin=-0.5; % Minimum error to controller input


avr.kP=5;     % [pu] Proportonal gain  u=(Kp+kI/s+ s kd)e
avr.kI=5;     % [pu] Integral gain  u=(Kp+kI/s + s kd)e
avr.kd= 0;    % [pu] Derivative gain
avr.derivativeLP=0.001;    % [s] Low pass filter before derivative in PID
avr.init=1 ; % [pu] Initial value for integrator 
avr.saturationUpper=0.5;     %[pu] Maximum controller output  (note: This is relative to reference since reference is feed-forwarded)
avr.saturationLower=-0.5;     %[pu] Minimum controller output  (note: This is relative to reference since reference is feed-forwarded)

avr.LPfilterPowerMeasurementDroop.T=0.001; % [s] LP filter time constant active power measurement 
avr.activePowerDroop.K=0;   % [pu/pu] Droop based on active power

avr.LPfilterReactivePowerMeasurementDroop.T=0.001;  % [s] LP filter time constant reactive power measurement
avr.reactivePowerDroop.K=0; % [pu/pu] Droop based on reactive power

avr.droopSaturationUpper=0.2; % [pu] Maximum contribution from droop
avr.droopSaturationLower=-0.2; % [pu] Minimum contribution from droop


%%%% SINTEF SM - END %%%%


%%%% NTNU SM - BEGIN %%%%

% %% SM
% sm = {};
% 
% % Rated values
% sm.fn=systemBase.fn;            % [Hz] electric frequency 
% sm.Vn=systemBase.Vn;            % [Vrms] Line voltage 
% sm.Sn=systemBase.Sn;            % [VA] Apparent power
% sm.Vn_phase = sm.Vn/sqrt(3);    % [Vrms] Phase voltage
% sm.In = sm.Sn/(sm.Vn*sqrt(3));  % [A] Line current
% sm.Zn = sm.Vn*sm.Vn/sm.Sn ;     % [Ohm] Base impedance
% sm.Rn = sm.Zn;                  % [Ohm] Base resistance
% sm.Wn = 2*pi()*sm.fn ;          % [rad/s] Base frequency
% sm.Ln = sm.Zn/sm.Wn ;           % [H] Base inductance
% sm.Cn = 1/(sm.Zn*sm.Wn) ;       % [F] Base capacitance
% sm.cosphi = 0.8;                % [-] Power factor
% sm.sat = [0.6404,0.7127,0.8441,0.9214,0.9956,1.082,1.19,1.316,1.457;0.7,0.7698,0.8872,0.9466,0.9969,1.046,1.1,1.151,1.201]; % No-load saturation curve
% 
% % Impedances and time constants
% sm.Xd = 2.12;               % d-axis steady-state impendance [pu]
% sm.Xdp = 0.299;             % d-axis transient impendance [pu]
% sm.Xdpp = 0.188;            % d-axis subtransient impendance [pu]
% sm.Xq = 0.982;              % q-axis steady-state impendance [pu]
% sm.Xqpp = 0.24;             % q-axis subtransient impendance [pu]
% sm.Xl = 0.131;              % Leakage impendance [pu]
% sm.TdpSC = 0.92;            % d-axis transient time - short-circuit [s]
% sm.TdppSC = 0.022;          % d-axis subtransient time - short-circuit [s]
% sm.TqppSC = 0.0334;         % q-axis subtransient time - short-circuit [s]
% sm.Rs = 0.0242;             % Stator (armature) resistance [pu]
% sm.M = 2.03;                % Inertia constant [s]
% sm.p = 2;                   % Pairs of poles
% 
% % Measurements
% sm.V_measure_T=0.005; % [s] LP filter time constant for SM volatge measurement
% sm.I_measure_T=0.005; % [s] LP filter time constant for SM current measurement
% sm.P_measure=0.04; % LP filter time constant for SM power measurement
% sm.Q_measure=0.04; % LP filter time constant for SM reactive power measurement
% 
% %% SM Exciter
% exc = {};
% exc.Tr = 1/(2*sm.fn);       % Input filter time constant [s]
% exc.Ka = 100;               % Proportional gain
% exc.VRmax = 5;              % Maximum output [pu]
% exc.VRmin = exc.VRmax*cos(deg2rad(150)); % Minimum output [pu]
% exc.Kf = 1e-5;              % Rate feedback (derivative) gain 
% exc.Tf = 0.01;              % Rate feedback (reset) time [s] 
% exc.Tb = 0.01;              % 1st lag time constant [s]
% exc.Tc = exc.Tb/10;     % 1st lead time constant [s]
% exc.Tb1 = 0;                % 2nd lag time constant [s]
% exc.Tc1 = 0;                % 2nd lead time constant [s]
% 
% %% Turbine
% turb = {};
% turb.T = 2.25;              % [s] Equivalent first-order delay
% 
% %% Turbine Governor
% gov = {};
% gov.Kp = 12e6/(sm.Sn*sm.cosphi*sys.rss); % [pu/pu] Permanent droop (proportional gain)
% gov.DB = 0.00025;           % [pu] Deadband
% gov.Kd = 0;                 % [pu/pu] Transient droop (derivative gain)
% gov.Tr = 0.1;               % [s] Reset time for transient droop [s]
% gov.flp = 3*2*pi();         % [rad/s] Low-pass frequency


%%%% NTNU SM - END %%%%

%% Load 1 (Parallel RL)
i=1;
load.Rpu(i) = 2*44/37 ; % [pu] Resistance
load.Xpu(i) =2*44/(37*tan(acos(0.95))) ; % [pu] Reactance
load.R(i) = load.Rpu(i)* systemBase.Rn ; % [ohm]
load.L(i) = load.Xpu(i) * systemBase.Ln ; % [H]

%% Load 2 (Series RL)
i=2;
load.Rpu(i) = 0.707; % [pu] Resistance
load.Xpu(i) = 0.707 ; % [pu] Reactance
load.R(i) = load.Rpu(i)* systemBase.Rn ; % [ohm]
load.L(i) = load.Xpu(i) * systemBase.Ln ; % [H]

%% Load 3 (Series RL)
i=3;
load.Rpu(i) = 5 ; % [pu] Resistance
load.Xpu(i) = 2 ; % [pu] Reactance
load.R(i) = load.Rpu(i)* systemBase.Rn ; % [ohm]
load.L(i) = load.Xpu(i) * systemBase.Ln ; % [H]

%% Interface to real HW lab

interface.Rs=0.1; % [Ohm] Serial interface resistance (at output to lab)
interface.Rp=100; % [Ohm] Paralell interface resistance (at  output to lab)


%% Wind farm - Collector system
col = {};
col.Un = 33000;                     % [Vrms] Line voltage
col.Uanp = col.Un*sqrt(2/3);        % [Vpeak] Phase to ground voltage 
col.cosphi = 0.9;                   % [-] Power factor
col.Sn = 12e6/col.cosphi;           % [VA] Wind farm power: 13.33MVA for cosphi=0.9
col.Iap = col.Sn/col.Un*sqrt(2/3);  % [Vpeak] phase to ground voltage 
col.Trafo.xsc = 0.12;               % [pu] Transformer short circuit reactance 
col.Trafo.rsc = 0.005;              % [pu] Transformer short circuit resistance

%95 mm2
%17MVA at 33kV
% DIGISILENT POWER FACTORY - 30kV Paper	NEKBA 3x95rm 18/30kV
% VERY SIMILAR TO OTHER CABLES 20kV and 30kV cables
% but not the most capacitive of them
col.Cable.Rpos = 194.800E-3;        % [Ohm/km] Resistance positive sequence
col.Cable.Lpos = 379.998E-6;        % [H/km] Inductance positive sequence
col.Cable.Cpos = 250.000E-9;        % [F/km] Capacitance positive sequence
col.Cable.Rzero = 779.200E-3;       % [Ohm/km] Resistance zero sequence
col.Cable.Lzero = 1.520E-3;         % [H/km] Inductance zero sequence
col.Cable.Czero = 246.900E-9;       % [F/km] Capacitance positive sequence

col.Cable.Limport = 4;              % [km] Import lenght
col.Cable.LinterWT = 2;             % [km] Inter WT lenght

%% Wind turbine 1 - Specification
wt1 = {};

% Sampling time, system frequency, switching frequency
wt1.Fn = systemBase.fn;
wt1.Fs_control = 1/Ts_phy;          % [Hz] Sampling frequency controller
wt1.Fsw = 5e3;                      % [Hz] Sampling frequency PWM
wt1.Ts_control = 1/wt1.Fs_control;  % [s] Sampling time controller A/D converter

% Grid-side converter
wt1.Pn = 4e6;                       % [W] Active power
wt1.cosphi = 0.9;                   % [-] Power factor
wt1.Sn = wt1.Pn/wt1.cosphi;         % [VA] Apparent power
wt1.Un = 690;                       % [Vrms] Line voltage (at LV side of trafo)
wt1.Uanp = wt1.Un*sqrt(2/3);        % [Vpeak] Phase to ground voltage (at LV side of trafo)
wt1.Uabp = wt1.Un*sqrt(2);          % [Vpeak] Line voltage (at LV side of trafo)
wt1.Iap = wt1.Sn/wt1.Un*sqrt(2/3);  % [Apeak] Line current (at LV side of trafo)
wt1.Udc = 1500;                     % [Vdc] Rated dc voltage
wt1.Idc = wt1.Sn/wt1.Udc;           % [Adc] Rated dc current
wt1.Cdc = wt1.Idc/(2*wt1.Fn*wt1.Udc); % [F] DC link capacitor
wt1.LCL.l2 = 0.06;                  % [pu] Short circuit reactance of trafo
wt1.LCL.r2 = 0.005;                 % [pu] Short circuit resistance of trafo
wt1.LCL.r1 = 0.01;                  % [pu] LCL main reactance resistance
wt1.dampDC = 0.8; %sqrt(2)/2;             % [-] damping Udc controller 

%% Wind turbine 1 - Grid-converter design
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('% WIND TURBINES');
[designOk, wt1.LCL] = LCLdesign(wt1);
if designOk == 1 
    [wt1.CurrContSingle, wt1.DCVContSingle] = PItuning(wt1,0);
    [wt1.CurrContDual, wt1.DCVContDual] = PItuning(wt1,1);
end

%% Wind turbine 1 - Controllers
% Current 
wt1.CurrCont.PImax = 1.5;           % [pu] max value of PI output
wt1.CurrCont.PImin = -1.5;          % [pu] min value of PI output
wt1.CurrCont.x = wt1.LCL.l1;        % [pu] reactance for dq decoupling
wt1.CurrCont.f_LPF_noise = wt1.LCL.fres;    % [Hz] low pass cutout filter
wt1.CurrCont.zeta = sqrt(2)/2;      % [pu] damping coefficient for notch filters
wt1.filtertype = 1;                 % Notch = 0; DSC = 1

%DC link voltage
wt1.DCVCont.PImax = 1.5;            % [pu] Max value of PI output
wt1.DCVCont.PImin = -1.5;           % [pu] Min value of PI output

wt1.DCVCont.Start = 1.1;

%AC voltage controller
wt1.ACVCont.PImax = 1.5;            % [pu] Max value of PI output
wt1.ACVCont.PImin = -1.5;           % [pu] Min value of PI output
wt1.ACVCont.kg = 1;                 % [pu/pu] Loop gain kg(kp + 1/sTi)
wt1.ACVCont.kp = 0.00;              % [pu/pu] Proportional gain kg(kp + 1/sTi)
wt1.ACVCont.Ti = 0.15;               % [s] Integral time
wt1.ACVCont.kiq_droop = 0.00;

%% Wind turbine 2 - Specification
wt2 = wt1;
wt2.DCVCont.Start = 1.2;

%% Wind turbine 3 - Specification
wt3 = wt1;
wt3.DCVCont.Start = 1.3;

%% Flex load - Specification
flex = {};

% Sampling time, system frequency, switching frequency
flex.Fn = systemBase.fn;
flex.Fs_control = 1/Ts_phy;          % [Hz] Sampling frequency controller
flex.Fsw = 2.5e3;                      % [Hz] Sampling frequency PWM
flex.Ts_control = 1/flex.Fs_control;  % [s] Sampling time controller A/D converter

% Grid-side converter
flex.Pn = 7.6e6;                     % [W] Active power
flex.cosphi = 0.9;                   % [-] Power factor
flex.Sn = flex.Pn/flex.cosphi;       % [VA] Apparent power
flex.Un = 690;                       % [Vrms] Line voltage (at LV side of trafo)
flex.Uanp = flex.Un*sqrt(2/3);        % [Vpeak] Phase to ground voltage (at LV side of trafo)
flex.Uabp = flex.Un*sqrt(2);          % [Vpeak] Line voltage (at LV side of trafo)
flex.Iap = flex.Sn/flex.Un*sqrt(2/3);  % [Apeak] Line current (at LV side of trafo)
flex.Udc = 1500;                     % [Vdc] Rated dc voltage
flex.Idc = flex.Sn/flex.Udc;           % [Adc] Rated dc current
flex.Cdc = flex.Idc/(2*flex.Fn*flex.Udc); % [F] DC link capacitor
flex.LCL.l2 = 0.06;                  % [pu] Short circuit reactance of trafo
flex.LCL.r2 = 0.005;                 % [pu] Short circuit resistance of trafo
flex.LCL.r1 = 0.01;                  % [pu] LCL main reactance resistance
flex.dampDC = 0.8; %sqrt(2)/2;             % [-] damping Udc controller 

%% Flex load - Grid-converter design
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('% FLEX LOAD');
%[designOk, flex.LCL] = LCLdesign(wt1);
[designOk, flex.LCL] = LCLdesign(flex);
if designOk == 1 
    [flex.CurrContDual, flex.DCVContDual] = PItuning(flex,1);
    [flex.CurrContSingle, flex.DCVContSingle] = PItuning(flex,0);
end

%% Wind turbine 1 - Controllers
% Current 
flex.CurrCont.PImax = 1.5;           % [pu] max value of PI output
flex.CurrCont.PImin = -1.5;          % [pu] min value of PI output
flex.CurrCont.x = flex.LCL.l1;        % [pu] reactance for dq decoupling
flex.CurrCont.f_LPF_noise = flex.LCL.fres;    % [Hz] low pass cutout filter
flex.CurrCont.zeta = sqrt(2)/2;      % [pu] damping coefficient for notch filters
flex.filtertype = 0;                 % Notch = 0; DSC = 1

%DC link voltage
flex.DCVCont.PImax = 1.5;            % [pu] Max value of PI output
flex.DCVCont.PImin = -1.5;           % [pu] Min value of PI output

flex.DCVCont.Start = 1;

%AC voltage controller
flex.ACVCont.PImax = 1.5;            % [pu] Max value of PI output
flex.ACVCont.PImin = -1.5;           % [pu] Min value of PI output
flex.ACVCont.kg = 1;                 % [pu/pu] Loop gain kg(kp + 1/sTi)
flex.ACVCont.kp = 0.00;              % [pu/pu] Proportional gain kg(kp + 1/sTi)
flex.ACVCont.Ti = 0.15;               % [s] Integral time
flex.ACVCont.kiq_droop = 0.00;




%% Initial conditions
turb.P0 = 0.027;         % [pu] Turbine power output

sm.Vt0 = 1;             % [pu] SM stator voltage
sm.Efd0 = 1.00182;            % [pu] SM field voltage
sm.dw0 = 0;             % [%] SM dw 
sm.th0 = -0.1;          % [deg] SM theta
sm.phi0 = -0.1;         % [deg] SM phi

ess.Vac0 = sm.Vt0;      % [pu] ESS AC voltage
ess.P0 = 0;             % [pu] ESS active power

wt1.Vac0 = sm.Vt0;      % [pu] WT1 AC voltage
wt2.Vac0 = sm.Vt0;      % [pu] WT2 AC voltage
wt3.Vac0 = sm.Vt0;      % [pu] WT3 AC voltage

% Original initial conditions from SINTEF template
avr.init=sm.Vt0;
