function y = sigma(x)
    y = (1 - 1e-16)*(2./(1 + exp(-8*x)) - 1);
end
