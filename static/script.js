// Send hardware commands to the Flask Server
function sendCommand(subsystem, action) {
  fetch(`/cmd/${subsystem}/${action}`, { method: 'POST' }).catch((err) =>
    console.error(`Error sending command to ${subsystem}:`, err),
  );
}

// Chassis Movement Logic
function startMove(dir) {
  sendCommand('chassis', dir);
}
function stopMove() {
  sendCommand('chassis', 'x');
}

// Snap high-res photo logic
function takePhoto() {
  const btn = document.getElementById('captureBtn');
  const img = document.getElementById('lastPhoto');
  const text = document.getElementById('snapshotText');

  btn.innerHTML = '⏳ Processing...';
  btn.disabled = true;

  fetch('/capture', { method: 'POST' })
    .then((response) => response.json())
    .then((data) => {
      if (data.success) {
        img.src = '/images/' + data.filename + '?t=' + new Date().getTime();
        img.style.display = 'block';
        text.style.display = 'none';
        btn.innerHTML = '📸 SNAP HIGH-RES TARGET';
        btn.disabled = false;
      }
    })
    .catch((err) => {
      alert('Error taking photo!');
      btn.innerHTML = '📸 SNAP HIGH-RES TARGET';
      btn.disabled = false;
    });
}

// Fetch and display live angles from the server
function fetchAngles() {
  fetch('/api/arm/angles')
    .then((response) => response.json())
    .then((data) => {
      document.getElementById('angleDisplay').innerText =
        `B:${data.base}° S:${data.shoulder}° E:${data.elbow}°\nP:${data.wpitch}° R:${data.wroll}° G:${data.gripper}°`;
    })
    .catch((err) => console.error('Could not load angles', err));
}

// Fetch angles immediately, then update every 2 seconds
fetchAngles();
setInterval(fetchAngles, 2000);

// Send the manual move command
function sendManualMove() {
  const joint = document.getElementById('jointSelect').value;
  const angle = document.getElementById('angleInput').value;

  if (angle === '') {
    alert('Please enter an angle!');
    return;
  }

  fetch('/cmd/arm/move', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ joint: joint, angle: parseInt(angle) }),
  });

  // Clear the input and fetch updated angles shortly after moving
  document.getElementById('angleInput').value = '';
  setTimeout(fetchAngles, 1000);
}

// Precise Distance Logic
function sendDistanceMove(direction) {
  const distInput = document.getElementById('distanceInput').value;
  const distance = parseFloat(distInput);

  if (isNaN(distance) || distance <= 0) {
    alert('Please enter a valid distance in centimeters (e.g., 50).');
    return;
  }

  fetch('/cmd/chassis/distance', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ distance: distance, direction: direction }),
  }).catch((err) => console.error('Error sending distance move:', err));

  // Optional: clear the input after sending
  document.getElementById('distanceInput').value = '';
}

// Send the sweep sequence command
function sendSweep() {
  const distInput = document.getElementById('distanceInput').value;
  const grid_size = parseFloat(distInput);

  if (isNaN(grid_size) || grid_size < 20) {
    alert('Please enter a valid grid size (minimum 20cm).');
    return;
  }

  fetch('/cmd/chassis/sweep', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ distance: grid_size }),
  }).catch((err) => console.error('Error sending sweep move:', err));

  document.getElementById('distanceInput').value = '';
}

// Auto-Align Logic
let isTracking = false;
function toggleTracking() {
  isTracking = !isTracking;
  const btn = document.getElementById('trackBtn');
  const state = isTracking ? 'on' : 'off';

  fetch(`/cmd/chassis/track/${state}`, { method: 'POST' }).catch((err) =>
    console.error('Error toggling tracking:', err),
  );

  if (isTracking) {
    btn.innerHTML = '🛑 STOP AUTO-ALIGN';
    btn.style.backgroundColor = '#ff4757'; // Red
    btn.style.color = 'white';
  } else {
    btn.innerHTML = '🎯 START AUTO-ALIGN';
    btn.style.backgroundColor = '#feca57'; // Yellow
    btn.style.color = '#111';
  }
}

// Update the Blue Response Zone live
function updateResponseZone() {
  const data = {
    bottom_width: document.getElementById('rz_bw').value,
    top_width: document.getElementById('rz_tw').value,
    height: document.getElementById('rz_h').value,
    offset_y: document.getElementById('rz_y').value,
  };

  fetch('/cmd/vision/response_zone', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  })
    .then((response) => response.json())
    .then((data) => {
      console.log('Zone updated:', data);
    })
    .catch((err) => console.error('Error updating response zone:', err));
}

function fetchTelemetry() {
  fetch('/api/status')
    .then((response) => response.json())
    .then((data) => {
      // Update the text on the screen
      document.getElementById('hud-status').innerText = data.action;
      document.getElementById('hud-yaw').innerText = data.yaw;
      document.getElementById('hud-pitch').innerText = data.pitch;
      document.getElementById('hud-roll').innerText = data.roll;

      // Change IMU status color based on health
      const imuLabel = document.getElementById('hud-imu');
      if (data.mpu_ok) {
        imuLabel.innerText = 'ONLINE';
        imuLabel.style.color = '#0f0'; // Green
      } else {
        imuLabel.innerText = 'OFFLINE/ERROR';
        imuLabel.style.color = '#f00'; // Red
      }
    })
    .catch((err) => console.error('Telemetry error:', err));
}

// Start the loop! Updates twice a second.
setInterval(fetchTelemetry, 500);
