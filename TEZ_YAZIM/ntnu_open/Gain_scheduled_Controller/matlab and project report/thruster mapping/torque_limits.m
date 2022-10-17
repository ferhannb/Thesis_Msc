clear all
close all
clc

%% Moment arm for on thruster
a  = 0.395;        %[m]

%% Total measured force for both thrusters FORWARD
Fp = [0 0.4 1.3 3.0 5.4 8.8 12.4 17.3 22.7 24.4 24.0]'*9.81;    %[N]

%% Force from 1 thruster FORWARD
Fp_half = Fp/2;                                                 %[N]

%% Total measured force for both thrusters REVERSE
Bp = -[0 0 0.7 1.9 3.5 5.4 7.4 10.8 13.4 13.5 13.6]'*9.81;      %[N]

%% Force from 1 thruster REVERSE
Bp_half = Bp/2;                                                 %[N]

%% Maximum force

F1 = max(Fp_half)                                              %[N]
F2 = min(Bp_half)                                              %[N]

Tp = (F1-F2)*a                                                 %[Nm]
 

