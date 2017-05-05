function [dq] = ppc_observer(t, q, plant, observer, Lambda, rho, k, sat)
%PPC Prescribed Perfomance Controller implementation (output feedback).
%
%   [dq] = PPC_OBSERVER(t, q, plant, observer, Lambda, rho, k, sat) 
%   controls given plant according to the perfomance function rho, using an
%   observer to estimate the plant's states. The controller gain is k, and 
%   the sliding surface parameters are given by Lambda. Furthermore, the
%   control input produced can be saturated in a desired interval (set
%   sat = inf if no saturation is desired). Assumes that the observer has a 
%   function signature of the form:
%
%       [dz, xhat] = observer(t, z, y)
%
%   where dz, is the observer's internal state time derivative, and xhat is
%   the plant's state estimations. The input arguments t, z, y, represent 
%   the time instant, the observer's previous state, and the plant's output
%   respectively. Also, the plant has a similar function signature:
%
%       [dx] = plant(t, x, u)
%
%   where dx is the time derivative of the plant's state, x, t is the time
%   instant and u is the control input.
%
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form" Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)

    q = q(:);
    n = length(q)/2;
    x = q(1:n);
    z = q(n+1:end);
    [dz, xhat] = observer(t, z, x(1));
    dz = dz(:);
    xhat = xhat(:);
    
    s = Lambda'*xhat;
    u = min(sat, max(-sat, -k*log((1 + s/rho(t))/(1 - s/rho(t)))));
    dx = plant(t, x, u);
   
    dq = [dx(:); dz(:)];
end
