function [dz, xhat] = hgo(t, z, y, alpha, mu)
%HGO High-Gain Observer implementation.
%
%   [dz, xhat] = HGO(t, z, y, alpha, mu) returns the time derivative of the 
%   state estimates, at a time instant t, driven by the plant's output y.
%   The observer's parameters are given by alpha and mu. 
%
%   See more:
%   Khalil, Hassan K. Noninear Systems. Prentice-Hall, New Jersey, 1996.
   
%   Ioannis Dimanidis (2017)

    z = z(:);
    xhat = z;
    n = length(z);
    A = [zeros(n-1, 1), eye(n-1); 0, zeros(1, n-1)];
    H = alpha./mu.^(1:n);
    H = H(:);
    
    dz = A*z + H*(y - z(1));
    dz = dz(:);
end

