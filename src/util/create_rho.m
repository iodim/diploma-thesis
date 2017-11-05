function [rho_wrapper] = create_rho(rho0, t1, ss)
    rho_wrapper = @(t) rho_exp(rho0, t1, ss, t);
end

function [y] = rho_exp(rho0, t1, ss, t)
    lambda = -(lambertw(-ss/(rho0*exp(1))) + 1)/t1;
    rho1 = rho0.*exp(-lambda.*t) + lambda.*rho0.*exp(-lambda.*t1).*t;
    y = (t <= t1).*rho1 + (t > t1).*ss;
end

function [y] = rho_inv(t1, ss, t)
    c = ss*t1/2;
    rho1 = c./t + c./(t1.^2).*t;
    y = (t <= t1).*rho1 + (t > t1).*ss;
end