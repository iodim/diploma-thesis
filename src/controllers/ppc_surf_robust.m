function [u, dw] = ppc_surf_robust(t, x, Lambda, rho, k, yd)
%PPC_SURF Prescribed Perfomance Controller implementation.
%
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form." Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)
    
    s = (Lambda'*x)';
    xi = s./rho(t);
    eps = transform(xi);
    u = -k.*eps./(rho(t).*(1 - xi.^2));
%     if imag(u)~=0, u = -u; end
    dw = [];
end

function [epsilon] = transform(xi)
    epsilon = log((1+xi)./(1-xi));
end