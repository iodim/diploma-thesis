function dx = plant4b(t, x, u)
    x = x(:);
    dx(1) = x(2);
    dx(2) = x(3);
    dx(3) = x(4);
    dx(4) = -(x(1) + 2*x(2) + 2*x(3) + 2*x(4)) ...
            + (1 - (x(1) + 2*x(2) + x(3))^2)*(x(2) + 2*x(3) + x(4)) + u;
    dx = dx(:);
end
