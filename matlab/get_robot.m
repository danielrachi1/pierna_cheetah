function robot = get_robot()
	% Using Modified DH, following Craig's textbook
	L(1) = Link('d', 136, 'a', 87.5, 'alpha', pi/2, 'offset', 0, 'modified', 'qlim', [0, pi]);
	L(2) = Link('d',  88, 'a',    0, 'alpha', pi/2, 'offset', -pi/2, 'modified', 'qlim', [-pi, 0]);
	L(3) = Link('d',   0, 'a',  220, 'alpha', 0,    'offset', 0, 'modified', 'qlim', [deg2rad(-135), deg2rad(135)]);

	robot = SerialLink(L, 'name', 'PiernaCheetah');

	% The aluminum structure elevates the first motor
	robot.base = transl(0, 0, 450);

	% The shoe, or the TCP is 175mm away from the knee
	T_tcp = transl(175, 0, 0);
	robot.tool = T_tcp;
end
