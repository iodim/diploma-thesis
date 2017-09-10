function [dz, xhat] = ppd3a(t, z, y, rho, k, t_p)
    n =  size(z, 1);   
    ksi = (y - z(1))./rho(t);
    epsilon = log((1 + ksi)./(1 - ksi));
    K = (k.^(1:n))';
    A = brunovsky(n);
    
    dz = A*z + K.*epsilon;
    
%     dz(1) = z(2) + k^n*epsilon;
%     
%     for i = 2:n-1
%        if t > (i-1)*t_p
%            dz(i) = z(i+1) + k^(n-i)*epsilon;
%        end
%     end
%     if t > (n-1)*t_p
%        dz(n) = k*epsilon;
%     end

    xhat = z;
    if any(imag(z) ~= 0)
        error(['Escaped performance bounds at t = ' num2str(t)]);
    end
end
