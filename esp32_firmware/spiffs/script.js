function sendCommand() {
    var motor_id = parseInt(document.getElementById('motor_id_command').value);
    var position = parseFloat(document.getElementById('position').value);
    var velocity = parseFloat(document.getElementById('velocity').value);
    var kp = parseFloat(document.getElementById('kp').value);
    var kd = parseFloat(document.getElementById('kd').value);
    var feed_forward_torque = parseFloat(document.getElementById('feed_forward_torque').value);

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
        .then(response => response.text())
        .then(data => alert(data))
        .catch(error => console.error('Error:', error));
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
        .then(response => response.text())
        .then(data => alert(data))
        .catch(error => console.error('Error:', error));
}
