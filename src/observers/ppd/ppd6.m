function [dz, xhat] = ppd6(t, z, y, rho, k, t_p)
%     b = rho(inf);
%     a = rho(0) - b;
%     lambda = -log((rho(t_p) - b)/a)/t_p;
    
    n =  size(z, 1);
    dz = zeros(n, 1);
    epsilon = zeros(n, 1);
    
    e = y - z(1);
    ksi = e./rho(t);
    epsilon = log((1 + ksi)./(1 - ksi));
    [A, B] = brunovsky(n);
    dz = A*z + k.*B.*epsilon;
    
    xhat = z;
    if any(imag(z) ~= 0)
        error(['Escaped performance bounds at t = ' num2str(t)]);
    end
end
