function [u, dw] = ppc_backstepping_robust_2(t, x, yd, rho, k)
    n = size(x, 1);
    e = x(1) - yd(t);
    xi = e./rho(t);
    epsilon = transform(xi);
    for i = 2:n
        e = x(i) + k(i-1)*epsilon;
        xi = e./rho(t);
        epsilon = transform(xi);
    end
    u = -k(n)*epsilon./(1-xi^2);
    dw = [];
end

function [epsilon] = transform(xi)
    epsilon = log((1+xi)./(1-xi));
end