clear

commands = [1 90 720; 2 -90 360; 3 -90 180]
batch_payload = build_batch_payload(commands)

[response_on, response_batch, response_off] = on_batch_off(batch_payload)
