function dx = plant2b(t, x, u)
    x = x(:);
    dx(1) = x(2);
    dx(2) = cos(t);
    dx = dx(:);
end
