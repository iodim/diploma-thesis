function [u, dw] = ppc_sat2(t, x, Lambda, rho, k, satlvl)
%PPC Prescribed Perfomance Controller implementation with saturation.
%
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form" Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)

    s = (Lambda'*x)';
    ksi = sat(s./rho(t), satlvl);
    u = -k*log((1 + ksi)./(1 - ksi));
    dw = [];
end