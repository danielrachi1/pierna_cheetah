function showNotification(message, type) {
    var notification = document.getElementById('notification');
    notification.textContent = message;
    notification.className = '';
    notification.classList.add(type === 'error' ? 'error' : 'success', 'show');

    setTimeout(function () {
        notification.classList.remove('show');
        notification.classList.add('hidden');
    }, 3000);
}

/**
 * Sends a JSON command to /api/command
 * @param {number} motorId 
 * @param {string} commandName 
 * @param {number} positionDeg 
 */
function sendJsonCommand(motorId, commandName, positionDeg) {
    let payload = {
        motor_id: motorId,
        command: commandName
    };

    if (commandName === 'go_to_position') {
        let angle = parseFloat(positionDeg);
        if (isNaN(angle) || angle < 0 || angle > 360) {
            showNotification("Position must be 0-360 degrees.", 'error');
            return;
        }
        payload.position = angle;
    }

    fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    })
        .then(res => res.text().then(txt => ({ status: res.status, text: txt })))
        .then(({ status, text }) => {
            if (status >= 200 && status < 300) {
                let jsonResp;
                try {
                    jsonResp = JSON.parse(text);
                } catch (e) {
                    showNotification("Malformed JSON response", 'error');
                    return;
                }

                if (jsonResp.status === 'ok') {
                    showNotification(jsonResp.message, 'success');
                } else {
                    showNotification(jsonResp.message, 'error');
                }
            } else {
                showNotification("HTTP " + status + ": " + text, 'error');
            }
        })
        .catch(err => {
            console.error(err);
            showNotification("Fetch error: " + err.message, 'error');
        });
}

function sendSpecialCommand(cmdName) {
    let motorId = parseInt(document.getElementById('motor_id_special').value);
    sendJsonCommand(motorId, cmdName, null);
}

function sendPositionOnlyCommand() {
    let motorId = parseInt(document.getElementById('motor_id_position').value);
    let deg = parseFloat(document.getElementById('position_only').value);
    sendJsonCommand(motorId, 'go_to_position', deg);
}
