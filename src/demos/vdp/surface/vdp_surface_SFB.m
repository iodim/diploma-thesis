vdp_surface_common

%% State feedback
controller = @(t, e, yd) ppc_surf_robust(t, e, Lambda, rho, k);
sys1 = @(t, q) control_loop(t, q, plant_e, [n 0 0], controller);

[t, e] = ode15s(sys1, [0 tmax], e0, ode_options);

u = zeros(size(t));
s = e*Lambda;

for i = 1:numel(t)
    u(i) = controller(t(i), e(i,:)', yd(t(i)));
end


%% Plots
plotter('t', t, 'x', e + psi(t')', 'u', u, 's', s, 'rho', rho)