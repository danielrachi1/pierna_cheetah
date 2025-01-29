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

    // Validate motor_id (should be 1, 2, or 3)
    if (![1, 2, 3].includes(motor_id)) {
        showNotification('Motor ID must be 1, 2, or 3.', 'error');
        return;
    }

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

function sendAdvancedCommand() {
    var motor_id = parseInt(document.getElementById('motor_id_command').value);
    var position = parseFloat(document.getElementById('position_advanced').value);
    var velocity = parseFloat(document.getElementById('velocity_advanced').value);
    var kp = parseFloat(document.getElementById('kp_advanced').value);
    var kd = parseFloat(document.getElementById('kd_advanced').value);
    var feed_forward_torque = parseFloat(document.getElementById('torque_advanced').value);

    // Validate motor_id
    if (![1, 2, 3].includes(motor_id)) {
        showNotification('Motor ID must be 1, 2, or 3.', 'error');
        return;
    }

    // Validate ranges
    if (position < -95.5 || position > 95.5) {
        showNotification('Position out of range (-95.5 to 95.5 rad).', 'error');
        return;
    }

    if (velocity < -45 || velocity > 45) {
        showNotification('Velocity out of range (-45 to 45 rad/s).', 'error');
        return;
    }

    if (kp < 0 || kp > 500) {
        showNotification('Kp out of range (0 to 500 N·m/rad).', 'error');
        return;
    }

    if (kd < 0 || kd > 5) {
        showNotification('Kd out of range (0 to 5 N·m·s/rad).', 'error');
        return;
    }

    if (feed_forward_torque < -18 || feed_forward_torque > 18) {
        showNotification('Torque out of range (-18 to 18 N·m).', 'error');
        return;
    }

    var command = {
        "motor_id": motor_id,
        "position": position,
        "velocity": velocity,
        "kp": kp,
        "kd": kd,
        "feed_forward_torque": feed_forward_torque
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

    if (![1, 2, 3].includes(motor_id)) {
        showNotification('Motor ID must be 1, 2, or 3.', 'error');
        return;
    }

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

/**
 * Toggles the visibility of the advanced command section.
 */
function toggleAdvancedSection() {
    var advancedSection = document.getElementById('advanced-section');
    if (advancedSection.classList.contains('show')) {
        advancedSection.classList.remove('show');
    } else {
        advancedSection.classList.add('show');
    }
}
