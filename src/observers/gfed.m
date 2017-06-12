function [dz, xhat] = gfed(t, z, y, lambda, L, mu)
%GFED Globally convergent Fast Exact Differentiator implementation.
%
%   [dz, xhat] = GFED(t, z, y, lambda) returns the time derivative of the
%   obverver's internal state, along with the plant's state estimates 
%   at a time instant t, driven by the plant's output y. The observer's 
%   gains are given by the N-element vector lambda, the Lipschitz constant 
%   L, and the N linear gains mu. We assume that L, the Lipschitz
%   constant, is not time variant.
%
%   See more:
%   Levant, Arie. "Globally convergent fast exact differentiator with 
%   variable gains." Control Conference (ECC), 2014 European. IEEE, 2014.

%   Ioannis Dimanidis (2017)

    z = z(:);
    n = length(z) - 1;
   
    dz = zeros(n+1, 1);
    phi = zeros(n+1, 1);
   
    phi(1) = lambda(n+1)*L^(1/(n+1))*abs(z(1) - y)^(n/(n+1))*sign(z(1) - y)...
             + mu(n+1)*(z(1) - y);
    dz(1) = -phi(1) + z(2);
    for i = 1:n-1
        phi(i+1) = lambda(n-i+1)*L^(1/(n-i+1))...
                   *abs(z(i+1) - dz(i))^((n-i)/(n-i+1))*sign(z(i+1) - dz(i))...
                   + mu(n-i+1)*(z(i+1) - dz(i));
        dz(i+1) = -phi(i+1) + z(i+2);
    end
    phi(n+1) = lambda(1)*L*sign(z(n+1) - dz(n)) + mu(1)*(z(n+1) - dz(n));
    dz(n+1) = -phi(n+1);
    
    dz = dz(:);
    xhat = [z(1); dz(1:n, :)];
end

function y = sign(x)
% Local replacement for the signum function,
% as abrupt switching makes the problem stiff
    y = sat(1e9*x);
end
