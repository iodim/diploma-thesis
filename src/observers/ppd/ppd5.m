function [dz, xhat] = ppd5(t, z, y, rho, k, t_p)
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
    epsilon(1) = -log((1 + ksi(1))./(1 - ksi(1)));
    dz(1) =  k*epsilon(1) + lambda*e(1);
    
    for i = 2:n-1
        if t > (i-1)*t_p
            e(i) = k*epsilon(i-1) - z(i);
            ksi(i) = e(i)./rho(t - (i-1)*t_p);
            epsilon(i) = -log((1 + ksi(i))./(1 - ksi(i)));
            dz(i) =  k*epsilon(i) + lambda*e(i);
       end
    end
    
    if t > (n-1)*t_p
        e(n) = k*epsilon(n-1) - z(n);
        ksi(n) = e(n)./rho(t - (n-1)*t_p);
        epsilon(n) = -log((1 + ksi(n))./(1 - ksi(n)));
        dz(n) = k*epsilon(n) + lambda*e(n);    
    end
    
    xhat = z;
    if any(imag(z) ~= 0)
        error(['Escaped performance bounds at t = ' num2str(t)]);
    end
end
