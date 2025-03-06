function commands = compute_trajectory_commands(angles, max_speeds)
    num_points = size(angles,1)  
  
    command_list = [];

    for i = 2:num_points
        % Angle difference from previous to current
        angle_diff = angles(i,:) - angles(i-1,:);
        % Speeds are 50 * (absolute angle difference)
        raw_speeds = abs(angle_diff) * 50;

        % Motor2 = index 2 in angle vector => clamp with max_speeds(1)
        sp2 = min(raw_speeds(2), max_speeds(1));
        % Motor3 = index 3 in angle vector => clamp with max_speeds(2)
        sp3 = min(raw_speeds(3), max_speeds(2));

        % Current absolute angles (positions)
        ang2 = angles(i,2);
        ang3 = angles(i,3);

        % -----------------------------------------------------------------
        % If speed for motor2 is nonzero, create command [2, position, speed].
        % -----------------------------------------------------------------
        if sp2 > 1e-9   % or some small tolerance
            command_list = [command_list; [2, ang2, sp2]]; %#ok<AGROW>
        end

        % -----------------------------------------------------------------
        % If speed for motor3 is nonzero, create command [3, position, speed].
        % -----------------------------------------------------------------
        if sp3 > 1e-9
            command_list = [command_list; [3, ang3, sp3]]; %#ok<AGROW>
        end
    end

    commands = command_list;

end
