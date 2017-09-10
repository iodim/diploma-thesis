function [dz, xhat] = ppd3b(t, z, y, rho, k, t_p)
    n =  size(z, 1);
    dz = zeros(n, 1);
    
    ksi = (y - z(1))./rho(t);
    epsilon = log((1 + ksi)./(1 - ksi));
    
    dz(1) = z(2) + k*epsilon;
    
    for i = 2:n-1
       if t > (i-1)*t_p
           dz(i) = z(i+1) + k^i*epsilon;
       end
    end
    if t > (n-1)*t_p
       dz(n) = k^n*epsilon;
    end

    xhat = z;
    if any(imag(z) ~= 0)
        error(['Escaped performance bounds at t = ' num2str(t)]);
    end
end
