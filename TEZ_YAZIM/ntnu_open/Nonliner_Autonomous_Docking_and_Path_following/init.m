function  init(rg,rp,mp,T_yaw,k_pos,k_neg)

global Xudot Nrdot Iz Xu Nr n_max n_min v_stw alpha ...
    tau_1_int tau_6_int pos_noise vel_noise pitchroll_noise heading_noise;

m=55;
g = 9.81;

L = 2.0;            % length (m)
B = 1.08;           % beam (m)
rg = (m*rg + mp*rp)/(m+mp);

R44 = 0.4 * B;      % radii of gyration (m)
R55 = 0.25 * L;
R66 = 0.25 * L;

Ig_CG = m * diag([R44^2, R55^2, R66^2]);    % only hull in CG
Ig = Ig_CG - m * Smtrx(rg)^2 - mp * Smtrx(rp)^2;  % hull + payload in CO

Iz = 45.126;
Xu = 24.4*(9.81/(6*0.5144));

Nr = Iz/T_yaw;
Xudot = -0.1 * m;
Nrdot = -1.7 * Ig(3,3);

n_max =  sqrt((0.5*24.4 * g)/k_pos);    % maximum propeller rev. (rad/s)
n_min = -sqrt((0.5*13.6 * g)/k_neg);    % minimum propeller rev. (rad/s)

v_stw=0;
alpha=0;

tau_1_int=0;
tau_6_int=0;

pos_noise=zeros(2,1);
vel_noise=0;
pitchroll_noise=zeros(2,1);
heading_noise=0;
end