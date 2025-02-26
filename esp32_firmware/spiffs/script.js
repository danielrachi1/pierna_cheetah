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
 * Switch UI to Off view
 */
function showOffView() {
    document.getElementById('robot-off').classList.remove('hidden');
    document.getElementById('robot-on').classList.add('hidden');
}

/**
 * Switch UI to On view
 */
function showOnView() {
    document.getElementById('robot-off').classList.add('hidden');
    document.getElementById('robot-on').classList.remove('hidden');
}

/**
 * Turn the robot on (POST /api/robot/on)
 */
function turnRobotOn() {
    fetch('/api/robot/on', { method: 'POST' })
        .then(res => res.json())
        .then(jsonResp => {
            if (jsonResp.status === 'ok') {
                showNotification("Robot turned on", 'success');
                showOnView();
            } else {
                showNotification(jsonResp.message, 'error');
            }
        })
        .catch(err => {
            console.error(err);
            showNotification("Error: " + err.message, 'error');
        });
}

/**
 * Turn the robot off (POST /api/robot/off)
 */
function turnRobotOff() {
    fetch('/api/robot/off', { method: 'POST' })
        .then(res => res.json())
        .then(jsonResp => {
            if (jsonResp.status === 'ok') {
                showNotification("Robot turned off", 'success');
                showOffView();
            } else {
                showNotification(jsonResp.message, 'error');
            }
        })
        .catch(err => {
            console.error(err);
            showNotification("Error: " + err.message, 'error');
        });
}

/**
 * Send a position command to the currently selected motor
 */
function sendPositionCommand() {
    let motorId = parseInt(document.getElementById('motor_id').value);
    let deg = parseFloat(document.getElementById('position_deg').value);

    let payload = {
        motor_id: motorId,
        command: "go_to_position",
        position: deg
    };

    fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    })
        .then(res => res.json())
        .then(jsonResp => {
            if (jsonResp.status === 'ok') {
                showNotification(jsonResp.message, 'success');
            } else {
                showNotification(jsonResp.message, 'error');
            }
        })
        .catch(err => {
            console.error(err);
            showNotification("Error: " + err.message, 'error');
        });
}

// On page load, assume Off
document.addEventListener('DOMContentLoaded', function () {
    showOffView();
});
