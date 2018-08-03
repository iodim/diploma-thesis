%% Output feedback
mimo_pd_zd_surface_common

A = [1 2 3; 4 5 6];
C = [0 0 1 0 0 0 0 0
     0 0 0 0 0 1 0 0];
 
observer = @(t, z, y) hgo(t, z, y, alpha, mu);
MIMO_observer = @(t, z, y) MIMO_observer(t, z, y, observer, A);

controller = @(t, e, yd) ppc_surf_robust(t, e, Lambda, rho, k);
sat_controller = @(t, e, yd) sat_control(t, e, controller, satlvl, yd);
MIMO_control = @(t, e) MIMO_controller(t, e, A, sat_controller, @(t) 0, []);

sys1 = @(t, q) control_loop(t, q, plant_e, [n 0 6], MIMO_control, MIMO_observer, C);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

%% Reconstruct state estimates
e = q(:, 3:n);
e_hat = q(:, n+1:end);
e1 = e(:, A(1, :));
e2 = e(:, A(2, :));
e1_hat = e_hat(:, A(1, :));
e2_hat = e_hat(:, A(2, :));

y = yd(t')';

u1 = zeros(size(t));
u2 = zeros(size(t));

s1 = e1*Lambda;
s2 = e2*Lambda;
s1hat = e1_hat*Lambda;
s2hat = e2_hat*Lambda;

for i = 1:numel(t)
    qi = [e1_hat(i, :)'; e2_hat(i, :)'];
    u = MIMO_control(t(i),  qi);
    u1(i) = u(1);
    u2(i) = u(2);
end

save('lastrun')

%% Plots

plotter('t', t, 'x', e1 + psi1(t')', 'xhat', e1_hat + psi1(t')');
% save2tikz ../tex/reports/'surface sfb+ofb (MIMO)'/figures/ex2_link1_states.tex
% suptitle('Link 1 states');

plotter('t', t, 's', s1, 'shat', s1hat, 'u', u1, 'rho', rho, 'peak', 0.005);
% suptitle('Link 1 tracking error');
% save2tikz ../tex/reports/'surface sfb+ofb (MIMO)'/figures/ex2_link1_error.tex

plotter('t', t, 'x', e2 + psi2(t')', 'xhat', e2_hat + psi2(t')');
% suptitle('Link 2 states');
% save2tikz ../tex/reports/'surface sfb+ofb (MIMO)'/figures/ex2_link2_states.tex

plotter('t', t, 's', s2, 'shat', s2hat, 'u', u2, 'rho', rho, 'peak', 0.005);
% suptitle('Link 2 tracking error');
% save2tikz ../tex/reports/'surface sfb+ofb (MIMO)'/figures/ex2_link2_error.tex
