function y = sigma(x, satlvl)
    y = satlvl*(2./(1 + exp(-16*x)) - 1);
end
