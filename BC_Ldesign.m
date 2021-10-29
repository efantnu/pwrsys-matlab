%% BC_Ldesign.m
% Authors: Erick Fernando Alves, Daniel dos Santos Mota
% Date: 2021-10-29
%
% This function designs the L filter of a battery (DC-DC) converter.
%
% Naming convention:
%   BC_    : battery converter, i.e., the converter connected to energy storage device
%   L      : inductor (and parasitic series resistance)
%   design : self explanatory



%%
function [output] = BC_Ldesign(param)
%% Interface
% Input Struct
% param
%   .Pn         : [W] rated power power
%   .Udc        : [Vdc] rated dc voltage (battery side, not DC link side)
%   .Fsw        : [Hz] converter switching frequency
%   .eta        : [pu] converter efficiency (for series R of inductor)
%   .deltaimax  : [pu] maximum current ripple in pu of rated
%
%% Output Struct
% output
%   .L          : [H] L on the battery side
%   .R          : [Ohm] R on the battery side
%
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('% BATTERY CONVERTER - L design');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');

%% Base values
Ibase = param.Pn / param.Udc;
Rbase = (param.Udc^2) / param.Pn;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assumed a Duty Cycle of 50%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% v = L di / dt
%       v: rated dc voltage
%       L: the inductance we want to size
%       di: delta i, the maximum ripple in A 
%       dt: the time, half the switching period (Duty cycle 50%)

di = param.deltaimax * Ibase;
output.L = param.Udc / ( di * 2 * param.Fsw);
output.R = (1 - param.eta) * Rbase; 

disp('Inductanced - battery side');
disp(['    L = ',num2str(output.L),' H']);
disp(['    R = ',num2str(output.R),' Ohm']);

