function [] = plotter(varargin)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if mod(nargin, 2) ~= 0
        error('Inputs must come in description-value pairs');
    end

    plot_x = 0;
    plot_s = 0;
    plot_u = 0;
    plot_rho = 0;
    plot_xhat = 0;
    plot_shat = 0;
    plot_peak = 0;

    for i=1:2:nargin-1
        desc = varargin{i};
        val = varargin{i+1};
        switch desc
            case {'Time', 'time', 't'}
                t = val;
            case {'State', 'state', 'x'}
                x = val;
                rank = size(x, 2);
                plot_x = 1;
            case {'Surface', 'surface', 'surf', 's'}
                s = val;
                plot_s = 1;
            case {'Control', 'control', 'u'}
                u = val;
                plot_u = 1;
            case {'Perfomance function', 'perform', 'rho'}
                rho = val;
                plot_rho = 1;
            case {'State estimates', 'state estim', 'xhat'}
                xhat = val;
                plot_xhat = 1;
            case {'Surface estimate', 'surf estim' , 'shat'}
                shat = val;
                plot_shat = 1;
            case {'Peaking time', 'peak', 'tp'}
                t_p = t <= val;
                plot_peak = 1;
            otherwise
                warning([desc ' is not a valid option, ignored.']);
        end
    end

    % Plot states and their estimates
    if plot_x
        figure('Position', [0 0 300 200]);
        for i = 1:rank
            if rank == 2, subplot_tight(2, 1, i);
            elseif rank == 3, subplot_tight(3, 1, i);
            elseif rank == 4, subplot_tight(2, 2, i);
            end
            box on; 
            hold on; 
            
            ymin = min(x(:, i));
            ymax = max(x(:, i));
            
            if ymin < 0, ymin = 1.1*ymin; 
            else, ymin = 0.9*ymin; end
            
            if ymax > 0, ymax = 1.1*ymax; 
            else, ymax = 0.9*ymax; end
                
            axis([0, max(t), ymin, ymax]);
            plot(t, x(:, i), 'k');
            if plot_xhat
                plot(t, xhat(:, i), '--k');
                ylabel(['$x_' num2str(i) '(t), \hat{x}_' num2str(i) '(t)$'], ...
                        'Interpreter', 'Latex');
            else
                ylabel(['$x_' num2str(i) '(t)$'], 'Interpreter', 'Latex');
            end
            if (rank == 4 && i >= 3) || (rank == 2 && i == 2)
                xlabel('Time, $t$', 'Interpreter', 'Latex');
            else
                set(gca,'XTick',[])
            end
            % Peaking plot
            if plot_peak
                subplot_tight_pos = get_subplot_tight_pos(rank, i);
                axes('position', subplot_tight_pos); 
                box on;
                hold on;
                axis tight;
                plot(t(t_p), x(t_p, i), 'k');
                if plot_xhat
                    plot(t(t_p), xhat(t_p, i), '--k');
                end
            end            
        end
        tightfig;
    end

    % Plot surface and/or control input
    if plot_s || plot_u
        figure('Position', [0 0 300 200]);

        % If there are two plots, then create the appropriate subplot_tights
        if plot_s && plot_u, subplot_tight(2, 1, 1); end

        if plot_s
            box on;
            hold on;
            plot(t, s, 'k');
            if plot_rho
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

            % Check if there is another plot below, in order to know where
            % to put the x-label
            if ~(plot_s && plot_u)
                xlabel('Time, $t$', 'Interpreter', 'Latex');
            else
                set(gca,'XTick',[])
            end

            % Peaking plot
            if plot_peak
                axes('position', get_subplot_tight_pos(2, 1)); 
                box on;
                hold on;
                axis tight;
                plot(t(t_p), s(t_p), 'k');
                if plot_shat
                    plot(t(t_p), shat(t_p), '--k');
                end
                plot([t(t_p), t(t_p)], [rho(t(t_p)), -rho(t(t_p))], ':k');
            end
        end

        % If there are two plots, then create the appropriate subfigures
        if plot_s && plot_u, subplot_tight(2, 1, 2); end

        if plot_u
            box on;
            hold on;
            plot(t, u, 'k')
            ylabel('$u(t)$', 'Interpreter', 'Latex');
            xlabel('Time, $t$', 'Interpreter', 'Latex');

            % Peaking plot
            if plot_peak
                axes('position', get_subplot_tight_pos(2, 2)); 
                box on;
                hold on;
                ymin = min(u);
                ymax = max(u);
                
                if ymin < 0, ymin = 1.1*ymin; 
                else, ymin = 0.9*ymin; end
                
                if ymax > 0, ymax = 1.1*ymax; 
                else, ymax = 0.9*ymax; end
                
                axis([0, max(t), ymin, ymax]);
                axis([0, max(t(t_p)), ymin, ymax]);
                plot(t(t_p), u(t_p), 'k')
            end
        end
        tightfig;
    end
end

function [pos] = get_subplot_tight_pos(n, i)
    switch n
        case 1, pos = [.5 .5 .2 .2];
        case 2
            switch i
                case 1, pos = [.675 .675 .2 .2];
                case 2, pos = [.675 .200 .2 .2];
            end
        case 4
            switch i
                case 1, pos = [.200 .675 .2 .2];
                case 2, pos = [.675 .675 .2 .2];
                case 3, pos = [.200 .200 .2 .2];
                case 4, pos = [.675 .200 .2 .2];
            end
    end
end
