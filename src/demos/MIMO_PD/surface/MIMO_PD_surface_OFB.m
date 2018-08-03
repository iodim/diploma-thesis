%% Output feedback
MIMO_PD_surface_common

A = [1 2 3; 4 5 6];
C = [1 0 0 0 0 0
     0 0 0 1 0 0];
 
observer = @(t, z, y) hgo(t, z, y, alpha, mu);
MIMO_observer = @(t, z, y) MIMO_observer(t, z, y, observer, A);

controller = @(t, e, yd, w) ppc_surf_robust(t, e, Lambda, rho, k);
sat_controller = @(t, e, yd, w) sat_control(t, e, controller, satlvl, yd);
MIMO_control = @(t, e, w) MIMO_controller(t, e, A, sat_controller, @(t) 0, w);

sys1 = @(t, q) control_loop(t, q, plant_e, [n 0 n], MIMO_control, MIMO_observer, C);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct state estimates
q1 = q(:, A(1, :));
q2 = q(:, A(2, :));
z1 = q(:, n + A(1, :));
z2 = q(:, n + A(2, :));

y = yd(t')';

u1 = zeros(size(t));
u2 = zeros(size(t));

s1 = q1*Lambda;
s2 = q2*Lambda;
s1hat = z1*Lambda;
s2hat = z2*Lambda;

for i = 1:numel(t)
    qi = [z1(i, :)'; z2(i, :)'];
    u = MIMO_control(t(i),  qi, []);
    u1(i) = u(1);
    u2(i) = u(2);
end

%% Plots

plotter('t', t, 'x', q1, 'xhat', z1);
save2tikz ../tex/reports/'surface sfb+ofb (MIMO)'/figures/ex2_link1_states.tex
% suptitle('Link 1 states');

plotter('t', t, 's', s1, 'shat', s1hat, 'u', u1, 'rho', rho, 'peak', 0.005);
% suptitle('Link 1 tracking error');
save2tikz ../tex/reports/'surface sfb+ofb (MIMO)'/figures/ex2_link1_error.tex

plotter('t', t, 'x', q2, 'xhat', z2);
% suptitle('Link 2 states');
save2tikz ../tex/reports/'surface sfb+ofb (MIMO)'/figures/ex2_link2_states.tex

plotter('t', t, 's', s2, 'shat', s2hat, 'u', u2, 'rho', rho, 'peak', 0.005);
% suptitle('Link 2 tracking error');
save2tikz ../tex/reports/'surface sfb+ofb (MIMO)'/figures/ex2_link2_error.tex
