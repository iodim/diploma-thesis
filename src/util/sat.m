function y = sat(x, lvl)
    if nargin < 2; lvl = 1; end
    y = min(lvl, max(-lvl, x));
end
