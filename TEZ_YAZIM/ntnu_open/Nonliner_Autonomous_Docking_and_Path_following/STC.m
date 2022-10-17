function [u_st,v_stw_dot, alpha_dot]=STC(lambda2,alpha_m,omega,gamma, epsilon,lambda,chi_e,chi_e_dot,e_stc)

global alpha v_stw;

%Sliding surface
s =  lambda*chi_e + chi_e_dot;

%Boundary layer
if abs(s) > alpha_m
    alpha_dot = omega * sqrt (gamma /2);
else
    alpha_dot=0;
end

beta = (2 * epsilon*alpha) + lambda2 + (2*(epsilon^2));

%Saturate s
sign_s=s/(abs(s) + e_stc);

v_stw_dot= -beta * sign_s;

%Control-law
u_st= - alpha * sqrt(abs(s)) * sign_s + v_stw;

end