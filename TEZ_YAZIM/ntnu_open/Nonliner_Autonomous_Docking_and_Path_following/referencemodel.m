function [chi_d_dot, r_d_dot, a_d_dot] = referencemodel(chi_r,r_d,r_dmax,a_d,a_dmax,omega_n,damp,chi_d)

chi_d_dot = sat(r_d,r_dmax);
r_d_dot   = sat(a_d,a_dmax);
a_d_dot   = -(2*damp + 1)*omega_n * sat(a_d,a_dmax)-(2*damp +1)*omega_n^2 * sat(r_d,r_dmax) + omega_n^3 * (chi_r-chi_d);

end