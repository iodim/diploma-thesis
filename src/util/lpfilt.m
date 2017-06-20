function [y, dx] = lpfilt(t, x, u, omega0, n)
    [A, B, C, D] = tf2ss(omega0^n, poly(repmat(-omega0, [1 n])));
%     [A, B, C, D] = butter(n, omega0/5e10, 'low');
    dx = A*x + B*u;
    y = C*x + D;
end

