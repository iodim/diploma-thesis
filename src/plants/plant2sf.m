function dx = plant2sf(t, x, u)
    dx(1) = 0.2*x(2)^3;
    dx(2) = x(1)*x(2) + u + u^3/7;
    dx = dx(:);
end

