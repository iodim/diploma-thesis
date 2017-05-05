function [dz, xhat] = chgo(t, z, y, alpha, mu, M)
%CHGO Cascade High-Gain Observer implementation.
%
%   [dz, xhat] = CHGO(t, z, y, alpha, mu, M) returns the time derivative of 
%   the obverver's internal state, along with the plant's state estimates, 
%   at a time instant t, driven by the plant's output. The observer's 
%   parameters are given by alpha and mu, and M is a vector with the 
%   saturation limits for each state (except the first).
%
%   See more:
%   Khalil, Hassan K. "Cascade high-gain observers in output feedback 
%   control." Automatica 80 (2017): 110-118.
   
%   Ioannis Dimanidis (2017)

    z = z(:);
    n = length(z);
   
    dz(1) = 1/mu*(z(2) + alpha(1)*(y - z(1)));
    dz(2) = alpha(2)/mu*(y - z(1));
    
    xhat(1) = z(1);
    xhat(2) = min(M(1), max(-M(1), z(2)/mu));
    
    for i = 3:n
       dz(i) = -alpha(i)/mu*(z(i) + xhat(i-1)); 
       xhat(i) = min(M(i-1), max(-M(i-1), -dz(i)));
    end
    dz = dz(:);
    xhat = xhat(:);
end

