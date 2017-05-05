function [dq] = ppc(t, q, plant, Lambda, rho, k)
%PPC Prescribed Perfomance Controller implementation (state feedback).
%
%   [dq] = PPC_OBSERVER(t, q, plant, Lambda, rho, k) controls the plant
%   according to the perfomance function rho, assuming full state feedback.
%   The controller gain is k, and the sliding surface parameters are given 
%   by Lambda. It is assumed that the plant obeys the following signature:
%
%       [dx] = plant(t, x, u)
%
%   where dx is the time derivative of the plant's state, x, t is the time
%   instant and u is the control input.
%
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form." Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)

    q = q(:);
    s = Lambda'*q;
    u = -k*log((1 + s/rho(t))/(1 - s/rho(t)));
    dq = plant(t, q, u);
    dq = dq(:);
end
