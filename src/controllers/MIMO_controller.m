function [u, dw] = MIMO_controller(t, x, A, controller, yd, w)
%MIMO_CONTROLLER Applies a specified controller to a MIMO system.
%
%       [u, dw] = MIMO_controller(t, x, A, controller, yd, w)
%
%   Given the whole state vector of an m-input m-output system of n-th
%   order, and the matrix A which associates inputs to outputs, this
%   function produces a m x 1 vector u along with the optional dw vector as
%   output. Assumes that the input parameter controller is a function 
%   handle (or a cell array of them) of the following form:
%       
%       [u, dw] = controller(t, x, y, w)
%   
%   and that yd is a vector function which produces the desired reference
%   trajectories for a given time instant t. The matrix A, describing input
%   output relations, should be of size m x n. When the controller is not
%   adaptive, the argument w is of course optional.

    [m, n] = size(A);
    adaptive = ~isempty(w);

    u = zeros(m, 1);
    if adaptive, dw = zeros(size(w));
    else, dw = [];
    end
    
    y = yd(t);
    
    for i = 1:m
        xm = x(A(i, :));
        if iscell(controller), controller_i = controller{i};
        else, controller_i = controller;
        end
        if adaptive, [u(i), dw(i)] = controller_i(t, xm, @(t) y(i, :)', w);
        else, u(i) = controller_i(t, xm, @(t) y(i, :)');
        end
    end
end