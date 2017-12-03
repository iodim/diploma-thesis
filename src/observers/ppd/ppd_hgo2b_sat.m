function [dz, xhat] = ppd_hgo2b_sat(t, z, y, rho, k, satlvl)
    n = size(z, 1);
    k = k(:);
    K = diag(k);
    
    e1 = y - z(1);
    xi = e1./rho(t);

    eps1 = log((1 + xi)./(1 - xi))./(1-xi.^2);
    
    if e1 == 0, Eps = ((2./rho(t)).^(1:n))'; 
    else, Eps = ((eps1./e1).^(1:n))';
    end
    
    z(2:end) = sat(z(2:end), satlvl);
    
    A = brunovsky(n);
    dz = A*z + K*Eps.*e1;
    xhat = z(:);
    xhat = sat(z(:), satlvl);
    if any(imag(z) ~= 0)
        warning(['Escaped performance bounds at t = ' num2str(t)]);
        close(gcf)
    end
end
