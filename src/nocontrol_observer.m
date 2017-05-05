function [dq] = nocontrol_observer(t, q, plant, observer)
%NOCONTROL_OBSERVER Applies an observer to given plant (no control).
%
%   Assumes that the plant adheres to the following signature:
%
%       [dx] = plant(t, x, u)
%
%   where dx is the time derivative of the plant's state, x, t is the time
%   instant and u is the control input, which is in this case is
%   permanently zero. Also, it is assumed that the observer has a function
%   signature of the following form:
%
%       [dz, xhat] = observer(t, z, y)
%
%   where dz, is the observer's internal state time derivative, and xhat is
%   the plant's state estimations. The input arguments t, z, y, represent 
%   the time instant, the observer's previous state, and the plant's output
%   respectively. 


%   Ioannis Dimanidis (2017)
    q = q(:);
    n = length(q)/2;
    x = q(1:n);
    z = q(n+1:end);
    o = observer(t, z, x(1));
    o = o(:);
    dz = o(1:n);
    dx = plant(t, x, 0);
    dx = dx(:);
    dq = [dx; dz];
end

