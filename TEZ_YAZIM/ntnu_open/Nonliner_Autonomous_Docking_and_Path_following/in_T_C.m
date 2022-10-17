function [a] = in_T_C(sigma,sigma_dot,sigma_min,sigma_max)

if sigma_min<sigma && sigma<sigma_max
    a=true;
elseif sigma<=sigma_min && sigma_dot>=0
    a=true;
elseif sigma<=sigma_min && sigma_dot<0
    a=false;
elseif sigma>= sigma_max && sigma_dot<=0
    a=true;
else
    a=false;
end

end