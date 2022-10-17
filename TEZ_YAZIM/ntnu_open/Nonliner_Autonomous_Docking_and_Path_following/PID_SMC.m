function [u_smc,s,s_dot]=PID_SMC(k_s,lambda,e_smc,chi_e,chi_e_dot,chi_d_dot,chi_d_ddot,chi_e_int)

global Nrdot Iz Nr;

T= (Iz - Nrdot)/Nr;
K=1/Nr;

v = chi_d_dot - 2*lambda*chi_e - lambda^2 * chi_e_int;
v_dot = chi_d_ddot - 2*lambda*chi_e_dot - lambda^2 * chi_e;

%Defining sliding surface
s = chi_e_dot + 2*lambda*chi_e + lambda^2 * chi_e_int;
s_dot = 2* lambda*chi_e_dot+ lambda^2 * chi_e;

rho= (abs((v_dot) *T/K) + 1/K * abs(v));

%10% uncertainty for all measurements
eta=rho*1.1;

%Saturation
sign_s=s/(abs(s) + e_smc);

%Control-law
u_smc = (T/K)*v_dot + v/K - (k_s+Nr*0)*s - eta*sign_s;

end