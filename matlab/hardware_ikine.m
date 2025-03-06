clear

% x_target = 175.5
y_target = -136
z_target = 100

q = ikine_planar(y_target, z_target)

commands = [1 q(1) 720; 2 q(2) 360; 3 q(3) 180]
payload = build_batch_payload(commands)

[response_on, response_batch, response_off] = on_batch_off(payload)