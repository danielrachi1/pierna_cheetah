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

function updatePositionInput() {
    var positionSlider = document.getElementById('position_slider');
    var positionInput = document.getElementById('position');
    positionInput.value = positionSlider.value;
}

function updatePositionSlider() {
    var positionSlider = document.getElementById('position_slider');
    var positionInput = document.getElementById('position');
    positionSlider.value = positionInput.value;
}

function updateVelocityInput() {
    var velocitySlider = document.getElementById('velocity_slider');
    var velocityInput = document.getElementById('velocity');
    velocityInput.value = velocitySlider.value;
}

function updateVelocitySlider() {
    var velocitySlider = document.getElementById('velocity_slider');
    var velocityInput = document.getElementById('velocity');
    velocitySlider.value = velocityInput.value;
}

function updateKpInput() {
    var kpSlider = document.getElementById('kp_slider');
    var kpInput = document.getElementById('kp');
    kpInput.value = kpSlider.value;
}

function updateKpSlider() {
    var kpSlider = document.getElementById('kp_slider');
    var kpInput = document.getElementById('kp');
    kpSlider.value = kpInput.value;
}

function updateKdInput() {
    var kdSlider = document.getElementById('kd_slider');
    var kdInput = document.getElementById('kd');
    kdInput.value = kdSlider.value;
}

function updateKdSlider() {
    var kdSlider = document.getElementById('kd_slider');
    var kdInput = document.getElementById('kd');
    kdSlider.value = kdInput.value;
}

function updateTorqueInput() {
    var torqueSlider = document.getElementById('feed_forward_torque_slider');
    var torqueInput = document.getElementById('feed_forward_torque');
    torqueInput.value = torqueSlider.value;
}

function updateTorqueSlider() {
    var torqueSlider = document.getElementById('feed_forward_torque_slider');
    var torqueInput = document.getElementById('feed_forward_torque');
    torqueSlider.value = torqueInput.value;
}

function sendCommand() {
    // Retrieve and parse input values
    var motor_id = parseInt(document.getElementById('motor_id_command').value);
    var position_deg = parseFloat(document.getElementById('position').value);
    var velocity_deg = parseFloat(document.getElementById('velocity').value);
    var kp = parseFloat(document.getElementById('kp').value);
    var kd = parseFloat(document.getElementById('kd').value);
    var feed_forward_torque = parseFloat(document.getElementById('feed_forward_torque').value);

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

    // Validate velocity within -180°/s to 180°/s
    if (velocity_deg < -180 || velocity_deg > 180) {
        showNotification('Velocity must be between -180°/s and 180°/s.', 'error');
        return;
    }

    // Validate kp within 0 to 500
    if (kp < 0 || kp > 500) {
        showNotification('Kp must be between 0 and 500 N·m/rad.', 'error');
        return;
    }

    // Validate kd within 0 to 5
    if (kd < 0 || kd > 5) {
        showNotification('Kd must be between 0 and 5 N·m·s/rad.', 'error');
        return;
    }

    // Validate feed_forward_torque within -18 to 18
    if (feed_forward_torque < -18 || feed_forward_torque > 18) {
        showNotification('Feed Forward Torque must be between -18 and 18 N·m.', 'error');
        return;
    }

    // Convert degrees to radians for position and velocity
    var position_rad = degreesToRadians(position_deg);
    var velocity_rad = degreesToRadians(velocity_deg);

    // Construct the command object
    var command = {
        "motor_id": motor_id,
        "position": position_rad,
        "velocity": velocity_rad,
        "kp": kp,
        "kd": kd,
        "feed_forward_torque": feed_forward_torque
    };

    // Send the command to the server
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

    // Validate motor_id (should be 1, 2, or 3)
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
