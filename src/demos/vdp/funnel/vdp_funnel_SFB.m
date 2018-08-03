vdp_funnel_common

%% State feedback
controller = @(t, x) funnel(t, x, mu1, k, @(x) 1, @(t) 0, phi);
sys1 = @(t, q) control_loop(t, q, plant_e, [n 0 0], controller);

[t, e] = ode15s(sys1, [0 tmax], e0, ode_options);

u = zeros(size(t));
M = (mu1.^(0:n-1))';
sigma = M.*e'; % scaled parameters
s = k*sigma; % virtual output

for i = 1:numel(t)
    u(i) = controller(t(i), e(i,:)');
end


%% Plots
plotter('t', t, 'x', e, 'u', u, 's', s, 'rho', rho)