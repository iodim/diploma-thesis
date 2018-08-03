function [dz, xhat] = MIMO_observer(t, z, y, observer, A)
%MIMO_OBSERVER Applies a specified observer to a MIMO system.
%
%       [dz, xhat] = MIMO_observer(t, z, y)
%
%   Given the whole state vector of an m-output system of n-th
%   order, and the matrix A which associates inputs to outputs, this
%   function applies observer to the m x 1 vector y in order to calculate
%   its derivatives.
    
    [m, n] = size(A);
    dz = zeros(size(z));
    xhat = zeros(size(z));
    
    for i = 1:m
        zm = z(A(i, :));
        [dz(A(i, :)), xhat(A(i, :))] = observer(t, zm, y(i));
    end
end