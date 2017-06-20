function [] = plotter4(t, x, s, rho, u, xhat, shat, t_p)
%PLOTTER4 Plots given signals for a dynamical system of rank 4
%
%   PLOTTER4(t, x, s, rho, u, xhat, shat, t_p)
%   Assumes PPC structure but all arguments beyond t and x are optional and
%   are plotted only if they are supplied. Can also create zoom plots for
%   the peaking period for the sliding surface and the control input.

    plot_surface = exist('s', 'var');
    plot_control = exist('u', 'var');
    plot_perfom = exist('rho', 'var');
    plot_xhat = exist('xhat', 'var');
    plot_shat = exist('shat', 'var');
    plot_peak = exist('t_p', 'var');

    % Plot state trajectories and their estimates
    figure();
    for i = 1:4
        subplot(2, 2, i)
            box on; 
            hold on; 
            axis([0, max(t), 1.1*min(x(:, i)), 1.1*max(x(:, i))]);
            plot(t, x(:, i), 'k');
            if plot_xhat
                plot(t, xhat(:, i), '--k');
                ylabel(['$x_' num2str(i) '(t), \hat{x}_' num2str(i) '(t)$'], ...
                        'Interpreter', 'Latex');
            else
                ylabel(['$x_' num2str(i) '(t)$'], 'Interpreter', 'Latex');
            end
            if i >= 3; xlabel('$t$', 'Interpreter', 'Latex'); end
    end

    % Plot sliding surface, its estimate and a zoom plot of the peaking period
    if plot_surface
        figure();
        if plot_control; subplot(2, 1, 1); end
        box on;
        hold on;
        plot(t, s, 'k');
        if plot_perfom
            axis([0, max(t), -1.1*rho(0), 1.1*rho(0)]);
            plot([t, t], [rho(t), -rho(t)], ':k'); 
        else
            axis([0, max(t), 1.1*min(s), 1.1*max(s)]);
        end
        if plot_shat
            plot(t, shat, '--k');
            ylabel('$s(x(t)), s(\hat{x}(t))$', 'Interpreter', 'Latex');
        else
            ylabel('$s(x(t))$', 'Interpreter', 'Latex');
        end
        
        % If no control plot is requested, set the x-axis label as there
        % won't be a plot below the current one
        if ~plot_control; xlabel('$t$', 'Interpreter', 'Latex'); end
        
        % Peaking plot
        if plot_peak
            axes('position', [.675 .675 .2 .2]); 
            box on;
            hold on;
            axis tight;
            plot(t(t_p), s(t_p), 'k');
            plot(t(t_p), shat(t_p), '--k');
            plot([t(t_p), t(t_p)], [rho(t(t_p)), -rho(t(t_p))], ':k');
        end

        % Plot control input and a zoom plot of the peaking period    
        if plot_control
            subplot(2, 1, 2)
            box on;
            hold on;
            plot(t, u, 'k')
            ylabel('$u(t)$', 'Interpreter', 'Latex');
            xlabel('$t$', 'Interpreter', 'Latex');
        
            % Peaking plot
            if plot_peak
                axes('position', [.675 .175 .2 .2]); 
                box on;
                hold on;
                axis([0, max(t(t_p)), 1.1*min(u), 1.1*max(u)]);
                plot(t(t_p), u(t_p), 'k')
            end
        end
    end
end
