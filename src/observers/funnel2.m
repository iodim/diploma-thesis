function [dz, xhat] = funnel2(t, z, y, p, q, Gamma, phi)
    z = z(:);
    n = length(z) - 1;
   
    dz = zeros(n+1, 1);
    e = y - z(1);
    k = 1/(1 - phi(t).^2*norm(e).^2);
    
    dz(1) = z(2) + (q(1)  + p(1)*k)*e;
    dz(2) = (q(2) + p(2)*k)*e;
    dz = dz(:);
    xhat = z;
end

