vdp_surface_common

%% Output feedback
C = [1 0];
observer = @(t, z, y) hgo(t, z, y, alpha, mu);
controller = @(t, e) ppc_surf_robust(t, e, Lambda, rho, k);
sat_controller = @(t, e) sat_control(t, e, controller, satlvl);
sys1 = @(t, q) control_loop(t, q, plant_e, [n 0 n], sat_controller, observer, C);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

e = q(:, 1:n);
z = q(:, n+1:end);

u = zeros(size(t));
s = e*Lambda;

for i = 1:numel(t)
    u(i) = sat_controller(t(i), z(i,:)');
end


%% Plots
plotter('t', t, 'x', e + psi(t')', 'xhat', z + psi(t')', 'u', u, 's', s, 'rho', rho)