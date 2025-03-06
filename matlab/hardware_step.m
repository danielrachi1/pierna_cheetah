clear
clc
close all

n_points = 500;
stanceFrac = 0.0;  % 60% stance, 40% swing

qf = ikine_planar(-236, 100);
q0  = ikine_planar(-136, 100);

A2 = 5;  % amplitude offset for theta2
A3 = 20;   % amplitude offset for theta3

[theta2_array, theta3_array] = plan_angles( ...
    n_points, qf(2), qf(3), q0(2), q0(3), stanceFrac, A2, A3);

hardware_angles = [
    q0; 
    qf;
    90 max(theta2_array) max(theta3_array);
    q0;
];

commands = compute_trajectory_commands(hardware_angles, [1440, 1440]);

payload_batch = build_batch_payload(commands);

% starting point
commands_start = [[1 q0(1) 720]; [2 q0(2) 180]; [3 q0(3) 180]];
payload_start = build_batch_payload(commands_start);


response_on = api_call('on', []);

response_start = api_call('batch', payload_start);

pause(2)

n_steps = 5;
response_steps = [];
for i= 1:n_steps
    response_step = api_call('batch', payload_batch);
    response_steps = [response_steps, response_step];
end

response_off = api_call('off', []);