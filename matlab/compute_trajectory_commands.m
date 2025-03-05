function commands = compute_trajectory_commands(trajectory, max_speeds)
%COMPUTE_TRAJECTORY_COMMANDS Generates motor commands from YZ trajectory.
%   commands = compute_trajectory_commands(trajectory, max_speeds)
%
%   Inputs:
%       trajectory  - Nx2 matrix, each row is [Y, Z] in mm.
%       max_speeds  - 1x2 array specifying maximum allowed speed (deg/s)
%                     for motor2 and motor3, respectively. E.g. [720, 720].
%
%   Output:
%       commands    - Mx3 matrix of [motor_id, position_deg, speed_deg_s],
%                     where each row is a valid move command. M could be
%                     up to 2*(N-1), but commands with zero speed are skipped.
%
%   Notes:
%   - We assume your "ikine_planar" function returns angles [theta1, theta2, theta3],
%     in degrees, with theta1 always being 90° (for motor1).
%   - Movement speed for each motor is set to (angle_change_in_degrees * 8),
%     then clamped by the corresponding max_speeds entry.
%   - If speed = 0 (i.e., angle did not change), we skip that command entirely.
%   - Motor1 is constant (90°) and does not get commands here.

    % Number of trajectory points
    num_points = size(trajectory, 1);
    if num_points < 2
        % With only one point, there's no motion to command
        commands = [];
        return;
    end

    % ---------------------------------------------------------------------
    % 1) Compute the motor angles (in degrees) for each (Y,Z) via ikine_planar
    % ---------------------------------------------------------------------
    trajectory_motor_angles = zeros(num_points, 3);
    for i = 1:num_points
        y_i = trajectory(i, 1);
        z_i = trajectory(i, 2);
        % The ikine_planar function returns [theta1, theta2, theta3], in deg
        trajectory_motor_angles(i,:) = ikine_planar(y_i, z_i);
        % Typically that will yield motor1=90°, plus motor2, motor3.
    end

    % ---------------------------------------------------------------------
    % 2) Convert angle changes to speed commands, skipping zero-speed moves
    % ---------------------------------------------------------------------
    % We'll accumulate commands in a dynamic array (cell or numeric),
    % then convert to numeric at the end.
    command_list = [];

    for i = 2:num_points
        % Angle difference from previous to current
        angle_diff = trajectory_motor_angles(i,:) - trajectory_motor_angles(i-1,:);
        % Speeds are 8 * (absolute angle difference)
        raw_speeds = abs(angle_diff) * 8;

        % Motor2 = index 2 in angle vector => clamp with max_speeds(1)
        sp2 = min(raw_speeds(2), max_speeds(1));
        % Motor3 = index 3 in angle vector => clamp with max_speeds(2)
        sp3 = min(raw_speeds(3), max_speeds(2));

        % Current absolute angles (positions)
        ang2 = trajectory_motor_angles(i,2);
        ang3 = trajectory_motor_angles(i,3);

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
