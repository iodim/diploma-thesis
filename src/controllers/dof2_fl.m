function [u, dw] = dof2_fl(t, x, ref, dref, zeta, wn, manipulator_parameters)

    %% Linear feedback
    k2 = 2*zeta*wn;
    k1 = wn^2;
%     K = [k1 0 k2 0;
%          0 k1 0 k2];
    K = [k1 k2 0 0;
         0 0 k1 k2];

    b = [0 1 0 0;
         0 0 0 1];
    
    v = -K*(x - ref(t)) + b*dref(t);   

    %% Feedback Linearization
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
    
    % Friction
    kappa1 = 1;
    kappa2 = 1;
    
    % Notation
    c1 = cos(x(1));
    c2 = cos(x(3));
    c12 = cos(x(1) + x(3));
    s2 = sin(x(3));
    
    % Inertia Matrix
    M11 = Iz1 + Iz2 + m1*l1^2/4 + m2*(l1^2 + l2^2/4 + l1*l2*c2);
    M12 = Iz2 + m2*(l2^2/4 + 1/2*l1*l2*c2); 
    M21 = M12;
    M22 = Iz2 + m2*l2^2/4;
    
    M = [M11, M12;
         M21, M22];
    
    % Coriollis vector
    c = 1/2*m2*l1*l2*s2;
    C = [-c*x(4)+kappa1, -c*(x(2)+x(4));
         c*x(2), kappa2];
    Cq = C*x([2,4]);
    
    % Gravity vector
    G = [1/2*m1*g*l1*c1 + m2*g*(l1*c1+1/2*l2*c12); ...
        1/2*m2*g*l2*c12];

    %% Controller
%     u = +Cq + G + M*v;
    u = v;
end

