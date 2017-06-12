function [u, dw] = ppc_sat(t, x, Lambda, rho, k, satlvl)
%PPC Prescribed Perfomance Controller implementation with saturation.
%
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form" Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)

    s = (Lambda'*x)';
    u = -k*log((1 + s./rho(t))./(1 - s./rho(t)));
    u(imag(u) ~= 0) = sign(real(u(imag(u) ~= 0)))*satlvl;
    u = sat(u, satlvl);
    dw = [];
end
