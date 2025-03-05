% starting point
q_start = ikine_planar(-136, 100);
batch_start = [[1 q_start(1) 720]; [2 q_start(2) 720]; [3 q_start(3) 720]];
payload_start = build_batch_payload(batch_start);

% step
step_type = 'walk'; % walk | gallop | run;

params = get_gait_params(step_type);
trajectory = generate_foot_trajectory(20, params);
trajectory_batch = compute_trajectory_commands(trajectory, [720, 720]);
payload_batch = build_batch_payload(trajectory_batch);

response_on = api_call('on', []);

response_start = api_call('batch', payload_start);
response_step = api_call('batch', payload_batch);

response_off = api_call('off', []);
