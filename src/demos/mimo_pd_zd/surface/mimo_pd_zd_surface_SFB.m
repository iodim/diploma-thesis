%% Output feedback
mimo_pd_zd_surface_common

A = [1 2 3; 4 5 6];
C = [0 0 1 0 0 0 0 0
     0 0 0 0 0 1 0 0];
 
controller = @(t, e, yd) ppc_surf_robust(t, e, Lambda, rho, k, yd);
MIMO_control = @(t, e) MIMO_controller(t, e(3:end), A, controller, @(t) 0, []);
sys1 = @(t, q) control_loop(t, q, plant_e, [n 0 0], MIMO_control);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

%% Reconstruct state estimates
x = q(:, 3:end);
x1 = x(:, A(1, :));
x2 = x(:, A(2, :));

y = yd(t')';

u1 = zeros(size(t));
u2 = zeros(size(t));
s1 = x1*Lambda;
s2 = x2*Lambda;

for i = 1:numel(t)
    qi = q(i, :)';
    u = MIMO_control(t(i), qi);
    u1(i) = u(1);
    u2(i) = u(2);
end

%% Plots
plotter('t', t, 'x', x1);
suptitle('Link 1 states');

plotter('t', t, 's', s1, 'u', u1, 'rho', rho);
suptitle('Link 1 tracking error');

plotter('t', t, 'x', x2);
suptitle('Link 2 states');

plotter('t', t, 's', s2, 'u', u2, 'rho', rho);
suptitle('Link 2 tracking error');
