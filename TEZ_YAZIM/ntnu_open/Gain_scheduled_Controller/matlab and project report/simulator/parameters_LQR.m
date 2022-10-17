clear all
clc

%% Conversion
rad2deg = 180/pi;
deg2rad = pi/180;

%% Model parameters
k = 0.0002373;
a = 0.395; 

K_fast = 0.0008;             %Fast: 0.0008               Slow: 0.0017
T_fast = 0.5;                %Fast: 0.5                  Slow: 0.7



A(:,:,1) = [0 1 0; 0 0 1; 0 0 -1.429];
A(:,:,2) = [0 1 0; 0 0 1; 0 0 -1.579];
A(:,:,3) = [0 1 0; 0 0 1; 0 0 -1.765];
A(:,:,4) = [0 1 0; 0 0 1; 0 0 -2];

B (:,:,1) = [0;0;0.002429];
B (:,:,2) = [0;0;0.002211];
B (:,:,3) = [0;0;0.001941];
B (:,:,4) = [0;0;0.0016];

C = [1 0 0;0 1 0; 0 0 1];

%% LQR

LQR_slow = [624.5,	2682.3,	1569.3]; 
LQR_fast = [830.7,	3032.2,	1268.5];

%% Referance scale factor
% Specify the scheduling variable's range.
u = 0:2; % Sog in m/s

% Specify K parameter
K = -0.0003*u + 0.0017;

% Specify T parameter
T = -0.0667*u + 0.7;

% Compute linear system at a given K and T value.
for i = 1:length(u)
    A1 = [0 1 0;
         0 0 1;
         0 0 -1/T(i)];
     
    B1 = [0; 0; K(i)/T(i)];
    
    C1 = [1 0 0;0 1 0;0 0 1];
    
   sys(:,:,i) = ss(A1,B1,C1,0); 
end
A2 = [0 1; 0 -1/T(2)];
B2 = [0;K(2)/T(2)];
C2 = [1 0];
D2 = 0;

s = size(A2,1);
Z = [zeros([1,s]) 1];
N = inv([A2,B2;C2,D2])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar = Nu + [(LQR_slow(2)+LQR_fast(2))/2 (LQR_slow(3)+LQR_fast(3))/2]*Nx;
  
%% Sim model
sim simulator_LQR




