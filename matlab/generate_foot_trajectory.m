function trajectory = generate_foot_trajectory(n_points, params)
	%GENERATE_FOOT_TRAJECTORY Produces an [n_points x 2] foot step trajectory.
	%   trajectory = generate_foot_trajectory(n_points, params)
	%
	%   Inputs:
	%       n_points (scalar)  - The total number of trajectory points to generate.
	%       params   (struct)  - Must contain:
	%            params.stanceFrac    (fraction of total step in stance phase)
	%            params.swingFrac     (fraction of total step in swing phase)
	%            params.stepHeight    (peak foot height above the baseline, mm)
	%            params.stepDistance  (stride length, mm)
	%
	%   Output:
	%       trajectory (n_points x 2) - Each row is [Y, Z] in mm.
	%
	%   The foot is assumed to start at Y=-136, Z=100, move to Y=(-136 - stepDistance),
	%   Z=100 during stance, and then return to the initial position in a smooth
	%   quadratic (Bézier) swing that peaks at (Z + stepHeight).
	%
	%   X is assumed to remain constant at 175.5.

	%% Basic checks
	if n_points < 2
	    error('n_points must be >= 2.');
	end

	% Number of points allocated to stance vs swing
	stance_points = max(2, round(params.stanceFrac * n_points));
	swing_points  = max(2, n_points - stance_points + 1);

	%% Define key (Y, Z) positions
	% Initial foot position in YZ
	initialYZ = [-136, 100];

	% "Behind" foot position: move stepDistance further in negative Y
	behindYZ  = [ initialYZ(1) - params.stepDistance,  initialYZ(2) ];

	% Midpoint for swing apex (average Y, Z + stepHeight)
	midYZ = [ ...
	    0.5*(initialYZ(1) + behindYZ(1)), ...
	    0.5*(initialYZ(2) + behindYZ(2)) + params.stepHeight ...
	];

	%% Generate stance trajectory (linear from initialYZ -> behindYZ)
	stanceTraj = [ ...
	    linspace(initialYZ(1), behindYZ(1), stance_points)', ...
	    linspace(initialYZ(2), behindYZ(2), stance_points)' ...
	];

	%% Generate swing trajectory (quadratic Bézier from behindYZ -> initialYZ)
	swingTraj = zeros(swing_points, 2);
	for i = 1:swing_points
	    t = (i - 1) / (swing_points - 1);  % normalized [0..1]
	    swingTraj(i, :) = ...
		(1 - t)^2 * behindYZ  + ...
		2 * (1 - t) * t * midYZ + ...
		t^2        * initialYZ;
	end

	%% Combine stance + swing
	% Omit the last row of stance to avoid duplicating 'behindYZ'
	stanceOnly = stanceTraj(1:end-1, :);
	trajectory = [stanceOnly; swingTraj];

	%% Ensure exact number of points in final output
	if size(trajectory, 1) > n_points
	    % If we have too many, truncate
	    trajectory = trajectory(1:n_points, :);
	elseif size(trajectory, 1) < n_points
	    % If we have fewer, pad by repeating the last point
	    shortfall = n_points - size(trajectory, 1);
	    trajectory = [trajectory; repmat(trajectory(end, :), shortfall, 1)];
	end
end
