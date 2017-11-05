function [u, dw] = ppc_backstepping(t, x, yd, rho, k)
    n = size(x, 1);
    
    e = x(1) - yd;
    xi = e./rho{1}(t);
    epsilon = transform(xi);
    for i = 2:n
        e = x(2) - k(1)*epsilon;
        xi = e./rho{1}(t);
        epsilon = transform(xi);
    end
    u = -k(n)*epsilon;
    dw = [];
end

function [epsilon] = transform(xi)
    epsilon = log((1+xi)./(1-xi));
end