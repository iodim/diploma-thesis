function [dx] = manipulator_2dof(t, x, u, manipulator_parameters, disturb)
%MANIPULATOR_2DOF Parametric plant decription for a 2 DOF manipulator system.
%
%   INPUT:
%     t  = Time instance       (scalar) 
%     x  = State vector        (4x1 real vector)
%     u  = 2 dimensional input (2x1 real vector )
%     manipulator_parameters = parameters vector (5x1 real)
%     in the form : [m1,m2,l1,l2,Iz1,Iz2,g].
%
%   OUTPUT: 
%     dx = The time derivative for each state. (4x1 vector)  
%
%   USAGE : 
%     params = [m1,m2,l1,l2,Iz1,Iz2,g];
%     sim_plant = @(t,x,u) manipulator_2dof(t,x,u,params);

    if nargin < 5, disturb = 1; end

    %% Parameters
    % Masses
    m1 = manipulator_parameters(1);
    m2 = manipulator_parameters(2);
    
    % Lengths
    l1 = manipulator_parameters(3);
    l2 = manipulator_parameters(4);
    
    % Moments of inertia
    Iz1 = manipulator_parameters(5);
    Iz2 = manipulator_parameters(6);

    % Gravity
    g = manipulator_parameters(7);
    
    % Notation
    c1 = cos(x(1));
    c2 = cos(x(3));
    c12 = cos(x(1) + x(3));
    s2 = sin(x(3));
    
    % Friction
    k1 = 1;
    k2 = 1;
    
    %% System Dynamics
    % Inertia Matrix
    M11 = Iz1 + Iz2 + m1*l1^2/4 + m2*(l1^2 + l2^2/4 + l1*l2*c2);
    M12 = Iz2 + m2*(l2^2/4 + 1/2*l1*l2*c2); 
    M21 = M12;
    M22 = Iz2 + m2*l2^2/4;
    
    M = [M11, M12;
         M21, M22];
    
    % Coriollis vector
    c = 1/2*m2*l1*l2*s2;
    C = [-c*x(4)+k1, -c*(x(2)+x(4));
         c*x(2), k2];
    Cq = C*x([2,4]);
    
    % Gravity vector
    G = [1/2*m1*g*l1*c1 + m2*g*(l1*c1+1/2*l2*c12); ...
        1/2*m2*g*l2*c12];
   
    % Nonlinear Friction
    T = 0.2*[-x(2);-x(4)];
    
    % Disturbances
    d = [3*sin(t) + 4*(t > 3 && t < 5);
         2*cos(t) + 3*(t > 4 && t < 6)];
    
    %% Plant
    dx = zeros(4,1);
    dx([1,3]) = x([2,4]);
    dx([2,4]) = -(M\(Cq + G)) + M\u + disturb*(M\d);

end

