function [theta2_array, theta3_array] = plan_angles( ...
            n_points, ...
            theta2_start, theta3_start, ...
            theta2_end,   theta3_end,   ...
            stanceFrac, A2, A3)

    % Number of points for stance vs. swing
    n_stance = max(1, round(stanceFrac * n_points));
    n_swing  = n_points - n_stance;

    theta2_array = zeros(n_points, 1);
    theta3_array = zeros(n_points, 1);

    % 1) Stance portion (hold the start angles)
    for i = 1:n_stance
        theta2_array(i) = theta2_start;
        theta3_array(i) = theta3_start;
    end

    % 2) Swing portion: smoothly move from (theta2_start,theta3_start) 
    %                   to   (theta2_end,  theta3_end)

    for i = 1:n_swing
        alpha = (i-1)/(n_swing-1);   % goes 0..1
        % Use a half-sine smooth factor that goes from 0 up to 1 and back to 0:
        s = 0.5 - 0.5*cos(pi * alpha);
        
        % Here we add an extra offset term. 
        % By subtracting A*sin(pi*alpha), you invert the offset relative to a positive A.
        theta2_array(n_stance + i) = theta2_start + s*(theta2_end - theta2_start) + A2*sin(pi * alpha);
        theta3_array(n_stance + i) = theta3_start + s*(theta3_end - theta3_start) + A3*sin(pi * alpha);
    end

end
