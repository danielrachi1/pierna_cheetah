// Display a notification message at the top-right
function showNotification(message, type) {
    var notification = document.getElementById('notification');
    notification.textContent = message;
    notification.className = ''; // Reset classes
    notification.classList.add(type === 'error' ? 'error' : 'success', 'show');
    setTimeout(function () {
        notification.classList.remove('show');
    }, 3000);
}

// Functions to switch between views
function showOffView() {
    document.getElementById('robot-off').classList.remove('hidden');
    document.getElementById('robot-on').classList.add('hidden');
    document.getElementById('robot-recovery').classList.add('hidden');
}

function showOnView() {
    document.getElementById('robot-off').classList.add('hidden');
    document.getElementById('robot-on').classList.remove('hidden');
    document.getElementById('robot-recovery').classList.add('hidden');
}

function showRecoveryView() {
    document.getElementById('robot-off').classList.add('hidden');
    document.getElementById('robot-on').classList.add('hidden');
    document.getElementById('robot-recovery').classList.remove('hidden');
}

// API call helper function
async function apiPost(url, payload = {}) {
    try {
        const response = await fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
        const jsonResp = await response.json();
        return jsonResp;
    } catch (err) {
        console.error(err);
        throw new Error("Network or server error: " + err.message);
    }
}

// Turn robot on
async function turnRobotOn() {
    try {
        const resp = await apiPost('/api/robot/on');
        if (resp.status === 'ok') {
            showNotification("Robot turned on", 'success');
            showOnView();
        } else if (resp.status === 'error' && resp.message.includes("Recovery needed")) {
            showNotification(resp.message, 'error');
            showRecoveryView();
        } else {
            showNotification(resp.message, 'error');
        }
    } catch (err) {
        showNotification("Error: " + err.message, 'error');
    }
}

// Turn robot off
async function turnRobotOff() {
    try {
        const resp = await apiPost('/api/robot/off');
        if (resp.status === 'ok') {
            showNotification("Robot turned off", 'success');
            showOffView();
        } else if (resp.status === 'error' && resp.message.includes("Recovery needed")) {
            showNotification(resp.message, 'error');
            showRecoveryView();
        } else {
            showNotification(resp.message, 'error');
        }
    } catch (err) {
        showNotification("Error: " + err.message, 'error');
    }
}

// Send a position command to the selected motor
async function sendPositionCommand() {
    let motorId = parseInt(document.getElementById('motor_id').value);
    let deg = parseFloat(document.getElementById('position_deg').value);
    let speed = parseFloat(document.getElementById('speed').value);

    const payload = {
        motor_id: motorId,
        speed: speed,
        command: "go_to_position",
        position: deg
    };

    try {
        const resp = await apiPost('/api/command', payload);
        if (resp.status === 'ok') {
            showNotification(resp.message, 'success');
        } else if (resp.status === 'error' && resp.message.includes("Recovery needed")) {
            showNotification(resp.message, 'error');
            showRecoveryView();
        } else {
            showNotification(resp.message, 'error');
        }
    } catch (err) {
        showNotification("Error: " + err.message, 'error');
    }
}

// Clear recovery state via API
async function clearRecovery() {
    try {
        const resp = await apiPost('/api/recovery/clear');
        if (resp.status === 'ok') {
            showNotification(resp.message, 'success');
            // After recovery is cleared, show Off view so the user can then turn on the robot.
            showOffView();
        } else {
            showNotification(resp.message, 'error');
        }
    } catch (err) {
        showNotification("Error: " + err.message, 'error');
    }
}

// On page load, default to Off view.
document.addEventListener('DOMContentLoaded', function () {
    showOffView();
});
