function [u, dw] = ppc_backstepping(t, x, yd, rho, k)
    n = size(x, 1);
    e = x(1) - yd(t);
    xi = e./rho(t);
    epsilon = transform(xi)./(1-xi^2);
    for i = 2:n
        e = x(i) + k(i-1)*epsilon;
        xi = e./rho(t);
        epsilon = transform(xi)./(1-xi^2);
    end
    u = -k(n)*epsilon;
    dw = [];
end

function [epsilon] = transform(xi)
    epsilon = log((1+xi)./(1-xi));
end