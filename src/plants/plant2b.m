function dx = plant2b(t, x, u)
    x = x(:);
    dx(1) = x(2);
    dx(2) = cos(t) + 5e1*cos(1e2*t);
    dx = dx(:);
end
