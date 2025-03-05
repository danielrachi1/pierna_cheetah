function q = ikine_planar(y, z)
	% Simplified inverse kinematics, operating as a planar robot
	% with theta1 fixed to 90 degrees and x = 175.5.
	% returns degrees

	offset_y = 136;
    	offset_z = 450;
    	L1 = 220;
    	L2 = 175;

	theta1 = pi/2;

	D = ((y + offset_y)^2 + (z - offset_z)^2 - L1^2 - L2^2) / (2*L1*L2);
    	theta3 = acos(D);

	theta2 = atan2(z - offset_z, y + offset_y) - atan2(L2*sin(theta3), L1 + L2*cos(theta3));

	q = [rad2deg(theta1), rad2deg(theta2), rad2deg(theta3)];
end

