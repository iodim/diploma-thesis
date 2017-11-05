function [u, dw] = sat_control(t, x, controller, satlvl, w)
%SAT_CONTROL Wrapper function that saturates the control input.
    
    if nargin < 5, [u, dw] = controller(t, x);
    else, [u, dw] = controller(t, x, w);
    end
    u = sat(u, satlvl);
end

