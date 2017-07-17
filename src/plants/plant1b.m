function dx = plant1b(t, x, u)
    dx = 1.3*cos(14*t)*sin(17*t) - 0.7*x^3;
end
