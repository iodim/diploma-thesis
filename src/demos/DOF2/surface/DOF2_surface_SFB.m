DOF2_surface_common

%% State feedback
A = [1 2; 3 4];
controller = @(t, e, yd, w) ppc_surf_robust(t, e, Lambda, rho, k, yd);
MIMO_control = @(t, e, w) MIMO_controller(t, e, A, controller, @(t) 0, w);
sys1 = @(t, q) control_loop(t, q, plant_e, [n 0 0], MIMO_control);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

% Reconstruct state estimates
e1 = q(:, A(1, :));
e2 = q(:, A(2, :));

% Plots
plotter('t', t, 'x', e1 + psi1(t')');
suptitle('Link 1 states');

plotter('t', t, 'x', e2 + psi2(t')');
suptitle('Link 2 states');
