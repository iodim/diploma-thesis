function [u, dw] = dwell_time_controller(t, q, controller, td, w)
%DWELL_TIME_CONTROLLER Implementation of a dwell time control scheme. 
%   
%   [u, dw] = DWELL_TIME_CONTROLLER(t, q, controller, td, w)
%   Accepts a controller function handle as an input argument (as described
%   in control_loop.m) and plugs into it, disabling the control input while
%   t < td. After td has passed the control scheme functions as normally.

%   Ioannis Dimanidis (2017)

    if nargin < 5
        [u, dw] = controller(t, q);
        u(t < td) = 0;
    else
        [u, dw] = controller(t, q, w);
        u(t < td) = 0;
    end
end
