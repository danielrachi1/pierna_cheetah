/**
 * Converts degrees to radians.
 * @param {number} degrees 
 * @returns {number} radians
 */
function degreesToRadians(degrees) {
    return degrees * (Math.PI / 180);
}

/**
 * Displays a notification message.
 * @param {string} message - The message to display.
 * @param {string} type - The type of message: 'success' or 'error'.
 */
function showNotification(message, type) {
    var notification = document.getElementById('notification');
    notification.textContent = message;
    notification.className = ''; // Reset classes
    notification.classList.add(type === 'error' ? 'error' : 'success', 'show');

    // Automatically hide the notification after 3 seconds
    setTimeout(function () {
        notification.classList.remove('show');
        notification.classList.add('hidden');
    }, 3000);
}

function sendPositionOnlyCommand() {
    var motor_id = parseInt(document.getElementById('motor_id_position').value);
    var position_deg = parseFloat(document.getElementById('position_only').value);

    // Validate position within 0° to 360°
    if (position_deg < 0 || position_deg > 360) {
        showNotification('Position must be between 0° and 360°.', 'error');
        return;
    }

    // Invert the sign only if motor_id is 1
    if (motor_id === 1) {
        position_deg = -position_deg;
    }

    var position_rad = degreesToRadians(position_deg);

    var command = {
        "motor_id": motor_id,
        "position": position_rad
    };

    fetch('/send_command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(command)
    })
    .then(response => {
        if (!response.ok) {
            return response.text().then(text => { throw new Error(text) });
        }
        return response.text();
    })
    .then(data => showNotification(data, 'success'))
    .catch(error => {
        console.error('Error:', error);
        showNotification('Failed to send command: ' + error.message, 'error');
    });
}

function sendSpecialCommand(commandName) {
    var motor_id = parseInt(document.getElementById('motor_id_special').value);

    var command = {
        "motor_id": motor_id,
        "command": commandName
    };

    fetch('/send_command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(command)
    })
        .then(response => {
            if (!response.ok) {
                return response.text().then(text => { throw new Error(text) });
            }
            return response.text();
        })
        .then(data => showNotification(data, 'success'))
        .catch(error => {
            console.error('Error:', error);
            showNotification('Failed to send command: ' + error.message, 'error');
        });
}
