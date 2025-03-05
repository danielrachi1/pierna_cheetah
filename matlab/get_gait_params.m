function P = get_gait_params(step_type)
    % Defines the stepDuration, stanceFrac, swingFrac, stepHeight,
    % and an approximate stepDistance based on 'walk', 'run', or 'gallop'.

    switch lower(step_type)
        case 'walk'
            P.stanceFrac    = 0.7;
            P.swingFrac     = 0.3;
            P.stepHeight    = 30;   % mm
            P.stepDistance  = 25;   % mm behind initial foot
        case 'run'
            P.stanceFrac    = 0.5;
            P.swingFrac     = 0.5;
            P.stepHeight    = 60;   % mm
            P.stepDistance  = 45;   % mm behind initial foot
        case 'gallop'
            P.stanceFrac    = 0.4;
            P.swingFrac     = 0.6;
            P.stepHeight    = 80;   % mm
            P.stepDistance  = 65;   % mm behind initial foot
        otherwise
            error('Unrecognized step_type: %s', step_type);
    end
end

