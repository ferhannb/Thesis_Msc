function [n,Thrust] = calc_thrust(tau_1,tau_6,k_pos,k_neg)
global n_min n_max
l1 = -0.395;
l2 = 0.395;

Thrust(2)=(tau_6 + l1*tau_1)/(l1 - l2);
Thrust(1)=tau_1 - Thrust(2);

%Preallocating array for k
k= [0 0];

%Multiplying with correct coefficient depending on rotational direction of thrusters
for j=1:2
    if Thrust(j) >0
        k(j)=k_pos;
    else
        k(j)=k_neg;
    end
end

B=[1 1; -l1 -l2]*[k(1) 0; 0 k(2)];
input = B\ [tau_1; tau_6];

n(1)=sat2(sign(input(1))*sqrt(abs(input(1))),n_min,n_max);
n(2)=sat2(sign(input(2))*sqrt(abs(input(2))),n_min,n_max);

end