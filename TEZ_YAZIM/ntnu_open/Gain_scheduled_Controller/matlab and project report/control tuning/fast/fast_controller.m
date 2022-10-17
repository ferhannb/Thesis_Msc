clear all
clc

%% Conversion
rad2deg = 180/pi;
deg2rad = pi/180;

%% Model parameters
K = 0.0008;             
T = 0.5;               
%%
% System matrices
A = [0 1 0;
     0 0 1;
     0 0 -1/T];
B = [0; 0; K/T];
C = [1 0 0;0 1 0;0 0 1];


%% LQR

Q = [230 0 0;
     0  1670 0;
     0  0 330]*300;

R = 0.1;
N = 0;
[K_, P, e] = lqr(A,B,Q,R,N); 

K_aw = 0.4;

%% Scale referance
A2 = [0 1; 0 -1/T];
B2 = [0;K/T];
C2 = [1 0];
D2 = 0;

s = size(A2,1);
Z = [zeros([1,s]) 1];
N = inv([A2,B2;C2,D2])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar = Nu + K_(2:3)*Nx;


%% Sim model
sim fast_sim

