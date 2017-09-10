function [dz, xhat] = ppd2b(t, z, y, rho, k, t_p)
    n =  size(z, 1);
    ksi = zeros(n, 1);
    dz = zeros(n, 1);
    
%     ksi = (y - z(1))./rho(t);
%     epsilon = log((1 + ksi)./(1 - ksi));
%     
%     A = brunovksy(n);
%     K = k*ones(n, 1);
%     
%     dz = A*z + K*epsilon;
    ksi(1) = (y - z(1))./rho(t);
    dz(1) = z(2) + k*log((1 + ksi(1))./(1 - ksi(1)));
    
    for i = 2:n-1
       if t > (i-1)*t_p
           ksi(i) = (dz(i-1) - z(i))./rho(t - (i-1)*t_p);
           dz(i) = z(i+1) + k^i*log((1 + ksi(i))./(1 - ksi(i)));
       end
    end
    if t > (n-1)*t_p
       ksi(n) = (dz(n-1) - z(n))./rho(t - (n-1)*t_p);
       dz(n) = k^n*log((1 + ksi(n))./(1 - ksi(n)));
    end

    xhat = z;
    if any(imag(z) ~= 0)
        close(gcf)
%         error(['Escaped performance bounds at t = ' num2str(t)]);
    end
end
