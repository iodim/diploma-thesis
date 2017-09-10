function [dz, xhat] = ppd4a(t, z, y, rho, k, t_p)
    b = rho(inf);
    a = rho(0) - b;
    lambda = -log((rho(t_p) - b)/a)/t_p;
    
    n =  size(z, 1);
    e = zeros(n, 1);
    ksi = zeros(n, 1);
    dz = zeros(n, 1);
    epsilon = zeros(n, 1);
    
    e(1) = y - z(1);
    ksi(1) = e(1)./rho(t);
    epsilon(1) = log((1 + ksi(1))./(1 - ksi(1)));
    dz(1) = z(2) + lambda*e(1);
    
    for i = 2:n-1
        e(i) = k*epsilon(i-1) - z(i);
        ksi(i) = e(i)./rho(t);
        epsilon(i) = log((1 + ksi(i))./(1 - ksi(i)));
        dz(i) = z(i+1) + lambda*e(i);
    end
    
    e(n) = k*epsilon(n-1) - z(n);
    ksi(n) = e(n)./rho(t);
    epsilon(n) = log((1 + ksi(n))./(1 - ksi(n)));
    dz(n) = k*epsilon(n) + lambda*e(n);    

    xhat = z;
    if any(imag(z) ~= 0)
        error(['Escaped performance bounds at t = ' num2str(t)]);
    end
end
