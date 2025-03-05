commands = [1 90 90; 2 -90 90; 3 -90 90; 1 0 90; 2 0 90; 3 0 90]
batch_payload = build_batch_payload(commands)

[response_on, response_batch, response_off] = on_batch_off(batch_payload)
