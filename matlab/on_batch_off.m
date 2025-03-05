function [response_on, response_batch, response_off] = on_batch_off(batch_payload)
	response_on = api_call("on", []);

	response_batch = api_call("batch", batch_payload);

	response_off = api_call("off", []);
end
