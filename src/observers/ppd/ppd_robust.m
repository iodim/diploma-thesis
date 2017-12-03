function [dz, xhat] = ppd_robust(t, z, y, rho, k, t_p)
    n =  size(z, 1);
    ksi = zeros(n, 1);
    dz = zeros(n, 1);
       
    ksi(1) = (y - z(1))./rho(t);
    dz(1) = k*log((1 + ksi(1))./(1 - ksi(1)))./(1-ksi(1).^2);
    
    for i = 2:n
       if t > (i-1)*t_p
           ksi(i) = (dz(i-1) - z(i))./rho(t - (i-1)*t_p);
           dz(i) = k*log((1 + ksi(i))./(1 - ksi(i)))./(1-ksi(i).^2);
       end
    end
%     xhat = [z(1); dz(1:n-1)];
    xhat = z(:);
    if any(imag(z) ~= 0)
        warning(['Escaped performance bounds at t = ' num2str(t)]);
        close(gcf)
    end
end
