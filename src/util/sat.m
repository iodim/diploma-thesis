function y = sat(x, lvl)
    if nargin < 2; lvl = 1; end
    x(abs(imag(x))>0) = sign(real(x(abs(imag(x))>0)))*lvl;
    y = min(lvl, max(-lvl, x));
end
