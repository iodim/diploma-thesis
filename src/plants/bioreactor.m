function dx = bioreactor(t, x, u)
    epsilon = 0.001;
    if t <= 30 -epsilon
        u = 0.08;
    elseif t <= 50 - epsilon
        u = 0.02;
    else
        u = 0.08;
    end
    a1 = 1;
    a2 = 1;
    a3 = 1;
    a4 = 0.1;
    m = x(1);
    s = x(2);
    dm = a1*m*s/(a2*m + s) - m*u;
    ds = -a1*a3*m*s/(a2*m +s) + (a4 - s)*u;
    dx = [dm; ds];
end
