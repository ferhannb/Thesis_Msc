function [y] = sat(x,xmax)

if abs(x)>= xmax
    y = sign(x)*xmax;
else
    y = x;
end

end