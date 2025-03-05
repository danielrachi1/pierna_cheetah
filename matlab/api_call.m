function response = api_call(endpoint, payload)
	base_url = 'http://cheetah.local';

	switch endpoint
		case "on"
			endpoint = '/api/robot/on';
		case "off"
			endpoint = '/api/robot/off';
		case "batch"
			endpoint = '/api/command/batch';
	end

	url = [base_url, endpoint];

	options = weboptions('MediaType', 'application/json','Timeout', 120);

	response = webwrite(url, options, payload);
end
