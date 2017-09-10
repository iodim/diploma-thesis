function [A, B] = brunovksy(n)
    if n >= 2
        A = [zeros(n-1, 1) eye(n-1); 0 zeros(1, n-1)];
        B = [zeros(n-1, 1); 1];
    else
        A = 1;
        B = 1;
    end
end
