clearvars;
%% REMOVE THAT
%Ts_sim = 100e-6; %inv(30*60*60); %100E-6;  % simulation time step [s] for physical subsystem
Ts_sim = 0.1e-6; 
Ts_phy = 1*Ts_sim; %2*Ts_sim; % Simulation time step [s] for simulated subsystem and grid emulator.

k_Ts = 1; % ratio between simulation time step and control time step
Ts_c = k_Ts * Ts_phy; % control time step [s]
Ts = Ts_c;

Ttot = 2;

%% System
systemBase = {};

% System baseValues
systemBase.Sn = 2*44e6; % VA
systemBase.Vn = 11000;  % V rms phase to phase
systemBase.fn = 60; % Hz

systemBase.Vn_phase = systemBase.Vn/sqrt(3); % V
systemBase.In   = systemBase.Sn/(systemBase.Vn*sqrt(3)) ; % A
systemBase.Zn   = systemBase.Vn*systemBase.Vn/systemBase.Sn ; % Ohm
systemBase.Rn   = systemBase.Zn; % Ohm
systemBase.Wn   = 2*pi()*systemBase.fn ; % rad/s
systemBase.Ln   = systemBase.Zn/systemBase.Wn ; % H
systemBase.Cn   = 1/(systemBase.Zn*systemBase.Wn) ; % F



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ESS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ess = {};

% Sampling time, system frequency, switching frequency
ess.Fn = systemBase.fn;
ess.Fs_control = 1/Ts_phy;          % [Hz] Sampling frequency controller
ess.Fsw = 5e3;                      % [Hz] Sampling frequency PWM
ess.Ts_control = 1/ess.Fs_control;  % [s] Sampling time controller A/D converter

% Grid-side converter
ess.Pn = 7.72e6;                       % [W] Active power
ess.cosphi = 0.8;                   % [-] Power factor
ess.Sn = ess.Pn/ess.cosphi;         % [VA] Apparent power
ess.Un = 690;                       % [Vrms] Line voltage (at LV side of trafo)
ess.Uanp = ess.Un*sqrt(2/3);        % [Vpeak] Phase to ground voltage (at LV side of trafo)
ess.Uabp = ess.Un*sqrt(2);          % [Vpeak] Line voltage (at LV side of trafo)
ess.Iap = ess.Sn/ess.Un*sqrt(2/3);  % [Apeak] Line current (at LV side of trafo)
ess.Udc = 1500;                     % [Vdc] Rated dc voltage
ess.Idc = ess.Sn/ess.Udc;           % [Adc] Rated dc current
ess.Cdc = ess.Idc/(2*ess.Fn*ess.Udc); % [F] DC link capacitor
ess.LCL.l2 = 0.06;                  % [pu] Short circuit reactance of trafo
ess.LCL.r2 = 0.005;                 % [pu] Short circuit resistance of trafo
ess.LCL.r1 = 0.01;                  % [pu] LCL main reactance resistance
ess.dampDC = 0.8; %sqrt(2)/2;             % [-] damping Udc controller 

%% ESS - Grid-converter design
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('% ESS - Grid Converter');
[designOk, ess.LCL] = GC_LCLdesign(ess);
if designOk == 1 
    [ess.CurrCont, ess.DCVCont] = GC_PItuning(ess,0);
%    [ess.CurrContDual, ess.DCVContDual] = PItuning(ess,1);
end

%% ESS - Controllers
ess.filtertype = 1;                 % None = 0; Notch = 1; DSC = 2

% Current 
ess.CurrCont.PImax = 1.5;           % [pu] max value of PI output
ess.CurrCont.PImin = -1.5;          % [pu] min value of PI output
ess.CurrCont.x = ess.LCL.l1;        % [pu] reactance for dq decoupling
ess.CurrCont.f_LPF_noise = ess.LCL.fres;    % [Hz] low pass cutout filter
ess.CurrCont.zeta = sqrt(2)/2;      % [pu] damping coefficient for notch filters



%% BC - Grid-converter design
bc1 = {};
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('% BC1 - Battery Converter');
bc1.Pn = 1.5e6 ;
bc1.Udc = ess.Udc ;
bc1.Fsw = 5000;
bc1.eta = 0.99;
bc1.deltaimax = 0.1;

[bcc.reactor] = BC_Ldesign(bc1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% State Space Model
%inputs
mb = 1;
vb = ess.Udc;


% initial conditinons
ibini = 0;
vdcini = ess.Udc;
igaaini = 0;
igabini = 0;
igacini = 0;

%Plant parameters
Rb = bcc.reactor.L; %0.001; % H - wild guess
Lb = bcc.reactor.R; %0.001; % H - wild guess
Rdc = 100e3;
Cdc = ess.Cdc;
L1 = ess.LCL.L1;
R1 = ess.LCL.R1;

