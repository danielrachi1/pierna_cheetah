function payload = build_batch_payload(commands)
    % commands is expected to be an Nx3 matrix, where each row represents [motor_id, position, speed]

    num_commands = size(commands, 1);
    data.batch(num_commands) = struct('motor_id', [], 'position', [], 'speed', []);

    for command_index = 1:num_commands
        data.batch(command_index).motor_id = commands(command_index, 1);
        data.batch(command_index).position = commands(command_index, 2);
        data.batch(command_index).speed = commands(command_index, 3);
    end
    
    payload = jsonencode(data);
end
