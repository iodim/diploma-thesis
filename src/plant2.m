function dx = plant2(t, x, u)
    x = x(:);
    dx(1) = x(2);
    dx(2) = -2*(x(1)^2 - 1)*x(2) - x(1) + 1 + (2 + sin(x(1)*x(2)))*u;
    dx = dx(:);
end
