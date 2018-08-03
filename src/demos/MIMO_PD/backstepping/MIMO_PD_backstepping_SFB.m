%% Output feedback
demo_MIMO_PD_common

A = [1 2 3; 4 5 6];
C = [1 0 0 0 0 0
     0 0 0 1 0 0];
 
controller = @(t, x, yd, w) ppc_backstepping_robust_2(t, x, yd, rho, k);
MIMO_control = @(t, x, w) MIMO_controller(t, x, A, controller, yd, w);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 0], MIMO_control);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

% Reconstruct state estimates
q1 = q(:, A(1, :));
q2 = q(:, A(2, :));

y = yd(t')';

u1 = zeros(size(t));
u2 = zeros(size(t));

for i = 1:numel(t)
    qi = q(i, :)';
    u = MIMO_control(t(i), qi, []);
    u1(i) = u(1);
    u2(i) = u(2);
end

%% Plots
plotter('t', t, 'x', q1);
suptitle('Link 1 states');

plotter('t', t, 's', q1(:, 1) - y(:, 1), 'u', u1, 'rho', rho);
suptitle('Link 1 tracking error');

plotter('t', t, 'x', q2);
suptitle('Link 2 states');

plotter('t', t, 's', q2(:, 1) - y(:, 2), 'u', u2, 'rho', rho);
suptitle('Link 2 tracking error');
