function [u, dw] = sat_control(t, x, controller, satlvl, yd, w)
%SAT_CONTROL Wrapper function that saturates the control input.
    
    dw = [];
    if nargin < 5, u = controller(t, x);
    elseif nargin < 6, u = controller(t, x, yd);
    else, [u, dw] = controller(t, x, yd, w);
    end
    u = sat(u, satlvl);
end

