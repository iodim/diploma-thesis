function [dz, xhat] = ppd_additive2(t, z, y, rho, r, q)
    q = q(:);
    n =  size(z, 1);
    
    e1 = y - z(1);
    xi = e1./rho(t);
    eps1 = log((1 + xi)./(1 - xi))./(1 - xi.^2);
    
    A0 = brunovsky(n);
    k = eps1/e1;
    K = (k.^(1:n))';
    R = diag(r);

    dz = A0*z + q.*e1 + R*K.*e1;
    
    xhat = z(:);
    if abs(xi) > 1
        warning(['Escaped performance bounds at t = ' num2str(t)]);
        close(gcf)
    end
end
