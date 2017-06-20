function dx = plant4a(t, x, u)
    x = x(:);
    dx(1) = x(2);
    dx(2) = x(3);
    dx(3) = x(4);
    dx(4) = log(norm(x) + 1)/(3*x(1)^2 + 0.21*x(2)^2 + 2.47*abs(x(3)) ...
            + 0.37*x(4)^2 + 1) ...
            + 1.33 + (2 + cos(1.71*norm(x)))^3*u;
    dx = dx(:);
end
