function [dx] = manipulator_2dof(t, x, u, manipulator_parameters)
%MANIPULATOR_2DOF Parametric plant decription for a 2 DOF manipulator system.
%
%   INPUT:
%     t  = Time instance       (scalar) 
%     x  = State vector        (2x1 real vector)
%     u  = 2 dimensional input (2x1 real vector )
%     manipulator_parameters = parameters vector (5x1 real)
%     in the form : [m1,m2,l1,l2,g].
%
%   OUTPUT: 
%     dx = The time derivative for each state. (4x1 vector)  
%
%   USAGE : 
%     params = [m1,m2,l1,l2,g];
%     sim_plant = @(t,x,u) manipulator_2dof(t,x,u,params);

    %% Parameters
    % Masses
    m1 = manipulator_parameters(1);
    m2 = manipulator_parameters(2);
    
    % Lengths
    l1 = manipulator_parameters(3);
    l2 = manipulator_parameters(4);
    
    % Gravity
    g = manipulator_parameters(5);
    
    %% System Dynamics
    % Inetria Matrix
    B = [(m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(x(3)),... % M(1,1)
          m2*l2^2 + m2*l1*l2*cos(x(3));...  % M(1,2)
          m2*l2^2 + m2*l1*l2*cos(x(3)),...  % M(2,1)
          m2*l2^2];                         % M(2,2)
    
    % Coriollis vector
    C = [-m2*l1*l2*sin(x(3))*(2*x(2)*x(4) + x(4)^2);...
         -m2*l1*l2*sin(x(3))*x(2)*x(4)];
     
    % Gravity vector
    G = [-(m1 + m2)*g*l1*sin(x(1)) - m2*g*l2*sin(x(1) + x(3));...
         -m2*g*l2*sin(x(1) + x(3))];
    
    % Nonlinear Friction
    T = 0.2*[-x(2);-x(4)];
    T = zeros(2,1);
    
    %% Plant
    dx = zeros(4,1);
    dx([1,3]) = eye(2)*x([2,4]);
    dx([2,4]) = -B\(C + G) + B\u + T ;

end

