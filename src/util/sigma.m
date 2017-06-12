function y = sigma(x)
    A = -1.005;
    B = 15.34;
    C = 1.057;
    D = 0.1712;
    K = 1.12;
    M = -0.09753;
    Q = 1.366;
%     y = A + (K-A)./(C+Q*exp(-B*(x-M)).^D);
%     y = (1-1e-16)*(exp(x) - exp(-x))./(exp(x) + exp(-x));
    y = (1-1e-16)*(2./(1 + exp(-2.677.*x)) - 1);
end
