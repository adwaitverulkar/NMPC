function thrust_fcn = fit_engine_data(kernel, num_pts)

    addpath casadi-3.6.4-windows64-matlab2018b\
    import casadi.*
    
    engine_data = readmatrix("engine_data.csv");
    
    speeds = engine_data(1, 2:end).';
    throttles = engine_data(2:end,1);
    thrusts = engine_data(2:end, 2:end);
    
    [speeds, throttles] = meshgrid(speeds, throttles);
    
    %% Fit: 'engine_fit'.
    [xData, yData, zData] = prepareSurfaceData( speeds, throttles, thrusts );
    
    % Set up fittype and options.
    ft = 'thinplateinterp';
    opts = fitoptions( 'Method', 'ThinPlateInterpolant' );
    opts.ExtrapolationMethod = 'thinplate';
    opts.Normalize = 'on';
    
    % Fit model to data.
    [fitresult, gof] = fit( [xData, yData], zData, ft, opts );
    
    if false
        % Plot fit with data.
        figure( 'Name', 'engine_fit' );
        h = plot( fitresult, [xData, yData], zData );
        legend( h, 'engine_fit', 'thrusts vs. speeds, throttles', 'Location', 'NorthEast', 'Interpreter', 'none' );
        % Label axes
        xlabel( 'speeds', 'Interpreter', 'none' );
        ylabel( 'throttles', 'Interpreter', 'none' );
        zlabel( 'thrusts', 'Interpreter', 'none' );
        grid on
        view( -42.5, 43.9 );
    end
    
    speeds = linspace(speeds(1), speeds(end), num_pts);
    speeds = speeds.';

    throttles = linspace(throttles(1), throttles(end), num_pts);
    throttles = throttles.';

    [speeds, throttles] = meshgrid(speeds, throttles);

    thrusts = fitresult(speeds, throttles);
    
    speeds = reshape(speeds, [], 1);
    throttles = reshape(throttles, [], 1);
    thrusts = reshape(thrusts, [], 1);
    
    xvals = [speeds, throttles].';
    rvals = dist(xvals);
    eps = 1/mean(rvals, 'all');

    if kernel == "gaussian"
        psivals = exp(-(eps*rvals).^2);
    elseif kernel == "tps"
        psivals = rvals.*log(rvals.^rvals);
    elseif kernel == "invmq"
        psivals = 1./sqrt(1+(eps*rvals).*(eps*rvals));
    elseif kernel == "linear"
        psivals = rvals;
    elseif kernel == "cubic"
        psivals = rvals.^3;
    end

    wvals = psivals\thrusts;
    
    X = SX.sym('X');
    Y = SX.sym('Y');

    rvals_sym = sqrt(sum(([X; Y] - xvals).^2));
    
    if kernel == "gaussian"
        thrusts_sym = exp(-(eps*rvals_sym).^2) * wvals;
    elseif kernel == "tps"    
        thrusts_sym = rvals_sym.*log(rvals_sym.^rvals_sym) * wvals;
    elseif kernel == "invmq"
        thrusts_sym = 1./sqrt(1+(eps*rvals_sym).*(eps*rvals_sym))* wvals;
    elseif kernel == "linear"
        thrusts_sym = rvals_sym * wvals;
    elseif kernel == "cubic"
        thrusts_sym = rvals_sym.^3 * wvals;
    end
    
    thrust_fcn = Function('thrusts', {X, Y}, {thrusts_sym});

end

