function [dz, xhat] = red(t, z, y, lambda)
%RED Robust Exact Differentiator implementation.
%
%   [dz, xhat] = RED(t, z, y, lambda) returns the time derivative of the
%   obverver's internal state, along with the plant's state estimates 
%   at a time instant t, driven by the plant's output y. The observer's 
%   gains are given by the N-element vector lambda.
%
%   See more:
%   Levant, Arie. "Higher-order sliding modes, differentiation and output
%   feedback control." International journal of Control 76.9-10 (2003):
%   924-941.

%   Ioannis Dimanidis (2017)

    z = z(:);
    n = length(z);
   
    dz = zeros(n, 1);
    v = zeros(n-1, 1);
    
    sign = @(x) sat(x*1e9); % Local replacement for the signum function, as
                            % such abrupt switching makes the problem stiff
    
    v(1) = -lambda(1)*abs(z(1) - y)^((n-1)/n)*sign(z(1) - y) + z(2);
    dz(1) = v(1);
    for i = 2:n-1
        v(i) = -lambda(i)*abs(z(i) - v(i-1))^((n-i)/(n-i+1))...
                *sign(z(i) - v(i-1)) + z(i+1);
        dz(i) = v(i);
    end
    dz(n) = -lambda(n)*sign(z(n) - v(n-1));
    
    dz = dz(:);
    xhat = [z(1); v(:)];
end

