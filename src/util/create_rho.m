function [rho_wrapper, lambda] = create_rho(rho0, t1, ss, b, type)
%CREATE_RHO Returns a function handle to a performance function with the
%specified characteristics. Accepts 'flat' and 'lamb' as type. Default is 
%'flat' and can be omitted.
%   [rho_wrapper, lambda] = create_rho(rho0, t1, ss, type)
    if nargin < 4
        b = 1;
        type = 'flat';
    elseif nargin < 5
        type = 'flat';
    end
    
    if strcmp(type, 'flat')
        rho_wrapper = @(t) rho_flat(rho0, t1, ss, t, b);
    elseif strcmp(type, 'lamb')
        % By using the primary branch (0) of the Lambert function, lambda 
        % should be negative as long as -1/e < -ss/(rho0*exp(1)). If we 
        % want lambda to be positive, we should switch to the secondary 
        % branch (-1).
        lambda = -(lambertw(0, -ss/(rho0*exp(1))) + 1)/t1;
        rho_wrapper = @(t) rho_lamb(rho0, t1, ss, t, lambda);
    else
        error('Not a valid type.');
    end
end

function [y] = rho_flat2(rho0, t1, ss, t, b)
    if nargin < 5, b = 1; end
    % b can adjust the curve of rho
    rho1 = (rho0 - ss)*exp(b*t1./(t - t1) + b) + ss;
    y = (t <= t1).*rho1 + (t > t1).*ss;
end

function [y] = rho_flat(rho0, t1, ss, t, b)
    if nargin < 5, b = 1; end
    % b can adjust the curve of rho
    rho1 = (rho0 - ss)*exp(-b*t1^2./(t - t1).^2 + b) + ss;
    y = (t <= t1).*rho1 + (t > t1).*ss;
end

function [y] = rho_lamb(rho0, t1, ss, t, lambda)
    rho1 = rho0.*exp(-lambda.*t) + lambda.*rho0.*exp(-lambda.*t1).*t;
    y = (t <= t1).*rho1 + (t > t1).*ss;
end