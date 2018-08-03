function [dx] = vdp(t, x, u, l)
%VDP Summary of this function goes here
%   Detailed explanation goes here
    x = x(:);
    dx = zeros(size(x));
    dx(1) = x(2);
    dx(2) = l*(1 - x(1)^2)*x(2) - x(1) + u + 5*((t>3) & (t<4));
end

