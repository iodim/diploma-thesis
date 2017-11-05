function [dq] = control_loop(t, q, plant, n, controller, observer, C)
%CONTROL_LOOP Applies a controller to a plant with an observer in the loop.
%
%   [dq] = CONTROL_LOOP(t, q, plant, n, controller, observer)
%
%   Assumes that the plant adheres to the following signature:
%
%       [dx] = plant(t, x, u)
%
%   where dx is the time derivative of the plant's state, x, t is the time
%   instant and u is the control input. The controller has the following 
%   signature:
%
%       [u, dw] = controller(t, xhat, w)
%   
%   where w is the controller's internal state - in case of an adaptive
%   control law - and dw is its the time derivative. The controller does 
%   not assume full state feedback, but operates according to an estimation
%   of the plant's state, given by an observer. It is assumed that the 
%   observer has a function signature of the following form:
%
%       [dz, xhat] = observer(t, z, y)
%
%   where dz, is the observer's internal state time derivative, and xhat is
%   the plant's state estimations. The input arguments z and y, represent 
%   the observer's internal state and the plant's output respectively.
%   The argument n is a vector containing the plant's, the controller's and
%   the observer's rank. If the observer's rank is zero, then full state 
%   feedback is assumed, and then the observer can be omitted. Finally, C
%   is a vector indicating the available states for measurement - optional.

%   Ioannis Dimanidis (2017)
    q = q(:);
    x = q(1:n(1));
    w = q(n(1)+1:n(1)+n(2));
    z = q(n(1)+n(2)+1:n(1)+n(2)+n(3));
    if n(3) ~= 0 
        if ~exist('C', 'var') || isempty(C)
            C = [1 zeros(1, n(1)-1)];
        end
        [dz, xhat] = observer(t, z, C*x);
        [u, dw] = controller(t, xhat, w);
    else
        dz = [];
        [u, dw] = controller(t, x, w);
    end
    dx = plant(t, x, u);
    dq = [dx(:); dw(:); dz(:)];
end
