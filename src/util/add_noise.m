function [z] = add_noise(t, tmax, x, noise_vec)
%ADD_NOISE Adds measurement noise to the given states.
%   Splits the noise vector into n parts, one for each state, and then
%   interps according to t/tmax, in order to have consistent results.
    x = x(:);
    noise_vec = noise_vec(:);
    n = size(x, 1);
    N = size(noise_vec, 1);

    z = zeros(n, 1);
    x = linspace(0, 1, N/n);
    for i = 1:n
        idx = ((i-1)*N/n + 1):(i*N/n);
        y = noise_vec(idx);
        v = interp1(x, y, t/tmax, 'linear');
        z(i) = x(i) + v;
    end
end

