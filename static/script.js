// Tab switching
function switchTab(name, btn) {
  document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('tab-' + name).classList.add('active');
  btn.classList.add('active');
}

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
    .catch(() => {
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
  const distInput = document.getElementById('distanceInput').value.trim();
  const grid_size = parseFloat(distInput);
  const hasDistance = distInput !== '' && !isNaN(grid_size) && grid_size >= 20;

  if (distInput !== '' && (isNaN(grid_size) || grid_size < 20)) {
    alert('Grid size must be at least 20 cm, or leave blank to run until boundary.');
    return;
  }

  const payload = hasDistance
    ? { distance: grid_size }
    : { boundary_mode: true };

  fetch('/cmd/chassis/sweep', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  }).catch((err) => console.error('Error sending sweep move:', err));

  document.getElementById('distanceInput').value = '';
}

function stopSweep() {
  fetch('/cmd/chassis/stop_sweep', { method: 'POST' })
    .catch((err) => console.error('Error stopping sweep:', err));
}

function skipRest() {
  fetch('/cmd/chassis/skip_rest', { method: 'POST' })
    .catch((err) => console.error('Error skipping rest:', err));
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
// --- ZONE TUNING LOGIC ---
let activeZone = 'response';

function switchZone(zone) {
  activeZone = zone;
  const rzInputs = document.getElementById('responseZoneInputs');
  const gzInputs = document.getElementById('grabZoneInputs');
  const btnRz = document.getElementById('btnResponseZone');
  const btnGz = document.getElementById('btnGrabZone');

  if (zone === 'response') {
    rzInputs.style.display = 'block';
    gzInputs.style.display = 'none';
    btnRz.className = 'btn btn-blue';
    btnGz.className = 'btn btn-grey';
  } else {
    rzInputs.style.display = 'none';
    gzInputs.style.display = 'block';
    btnRz.className = 'btn btn-grey';
    btnGz.className = 'btn btn-blue';
  }
}

function updateActiveZone() {
  if (activeZone === 'response') {
    updateResponseZone();
  } else {
    updateGrabZone();
  }
}

function updateResponseZone() {
  const data = {
    bottom_width: document.getElementById('rz_bw').value,
    top_width: document.getElementById('rz_tw').value,
    height: document.getElementById('rz_h').value,
    offset_x: document.getElementById('rz_x').value,
    offset_y: document.getElementById('rz_y').value,
  };

  fetch('/cmd/vision/response_zone', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  })
    .then((response) => response.json())
    .then((data) => console.log('Response Zone updated:', data))
    .catch((err) => console.error('Error updating response zone:', err));
}

function updateGrabZone() {
  const data = {
    width: document.getElementById('gz_w').value,
    height: document.getElementById('gz_h').value,
    offset_x: document.getElementById('gz_x').value,
    offset_y: document.getElementById('gz_y').value,
    angle: document.getElementById('gz_a').value,
  };

  fetch('/cmd/vision/grab_zone', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  })
    .then((response) => response.json())
    .then((data) => console.log('Grab Zone updated:', data))
    .catch((err) => console.error('Error updating grab zone:', err));
}
// -------------------------

function fetchTelemetry() {
  fetch('/api/status')
    .then((response) => response.json())
    .then((data) => {
      // Update the text on the screen
      document.getElementById('hud-status').innerText = data.action;
      document.getElementById('hud-yaw').innerText = data.yaw;
      document.getElementById('hud-pitch').innerText = data.pitch;
      document.getElementById('hud-roll').innerText = data.roll;

      const tiltLabel = document.getElementById('hud-tilt');
      if (data.tilt_warning) {
        tiltLabel.innerText = 'TILT!';
        tiltLabel.style.color = '#f00';
      } else {
        tiltLabel.innerText = 'OK';
        tiltLabel.style.color = '#0f0';
      }

      const usLeft  = document.getElementById('hud-us-left');
      const usRight = document.getElementById('hud-us-right');
      if (data.sensor_left !== undefined) {
        const threshold = 40; // cm — mirrors DIST_THRESH
        usLeft.innerText  = `${data.sensor_left}cm`;
        usRight.innerText = `${data.sensor_right}cm`;
        usLeft.style.color  = data.sensor_left  < threshold ? '#f00' : '#0f0';
        usRight.style.color = data.sensor_right < threshold ? '#f00' : '#0f0';
      }

      // Change IMU status color based on health
      const imuLabel = document.getElementById('hud-imu');
      if (data.mpu_ok) {
        imuLabel.innerText = 'ONLINE';
        imuLabel.style.color = '#0f0'; // Green
      } else {
        imuLabel.innerText = 'OFFLINE/ERROR';
        imuLabel.style.color = '#f00'; // Red
      }

      // --- NEW: BATTERY STATUS ---
      const batteryLabel = document.getElementById('hud-battery');
      if (data.battery_percent !== undefined) {
        batteryLabel.innerText = `${data.battery_percent}% (${data.battery_voltage}V)`;
        if (data.battery_alert) {
          batteryLabel.style.color = '#f00'; // Red for alert
        } else if (data.battery_percent < 20) {
          batteryLabel.style.color = '#ff9f43'; // Orange for low
        } else {
          batteryLabel.style.color = '#0f0'; // Green for good
        }
      } else {
        batteryLabel.innerText = 'N/A';
        batteryLabel.style.color = '#777';
      }
      // --- EMERGENCY STOP BANNER ---
      const banner = document.getElementById('stop-banner');
      if (data.stop_reason) {
        banner.innerText = '⛔ AUTO-STOPPED: ' + data.stop_reason;
        banner.style.display = 'block';
      } else {
        banner.style.display = 'none';
      }

      // --- SKIP REST BUTTON ---
      const skipRestBtn = document.getElementById('btn-skip-rest');
      if (skipRestBtn) {
        skipRestBtn.style.display = data.is_resting ? 'block' : 'none';
      }
    })
    .catch((err) => console.error('Telemetry error:', err));
}

// Start the loop! Updates twice a second.
setInterval(fetchTelemetry, 500);

// --- BIN VOLUME ---
function fetchBinVolumes() {
  fetch('/api/bin_volumes')
    .then((r) => r.json())
    .then((data) => {
      const p = Math.min(100, data.plastic || 0);
      const g = Math.min(100, data.glass   || 0);
      document.getElementById('plastic-pct').innerText = p.toFixed(1) + '%';
      document.getElementById('glass-pct').innerText   = g.toFixed(1) + '%';
      document.getElementById('plastic-bar').style.width = p + '%';
      document.getElementById('glass-bar').style.width   = g + '%';

      const pBar = document.getElementById('plastic-bar');
      pBar.style.background = p >= 90 ? 'var(--red)' : p >= 70 ? 'var(--amber)' : 'var(--cyan)';

      const gBar = document.getElementById('glass-bar');
      gBar.style.background = g >= 90 ? 'var(--red)' : g >= 70 ? 'var(--yellow)' : 'var(--amber)';
    })
    .catch((err) => console.error('Bin volume error:', err));
}

function resetBin(bin) {
  fetch('/api/bin_volumes/reset', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ bin: bin }),
  })
    .then(() => fetchBinVolumes())
    .catch((err) => console.error('Reset error:', err));
}

function testSMS() {
  const btn = document.getElementById('testSmsBtn');
  const status = document.getElementById('testSmsStatus');
  btn.disabled = true;
  btn.innerText = '⏳ Sending…';
  status.innerText = '';
  fetch('/api/test_sms', { method: 'POST' })
    .then((r) => r.json())
    .then((d) => {
      if (d.status === 'sent') {
        btn.innerText = '✅ SMS Sent!';
        status.innerText = 'Check your phone (09398840532)';
        status.style.color = 'var(--green)';
      } else {
        btn.innerText = '❌ Send Failed';
        status.innerText = 'Check PhilSMS token in config.json';
        status.style.color = 'var(--red)';
      }
      setTimeout(() => {
        btn.innerText = '📱 Send Test SMS';
        btn.disabled = false;
        status.innerText = '';
      }, 4000);
    })
    .catch(() => {
      btn.innerText = '❌ Error';
      btn.disabled = false;
    });
}

function testSMSCritical() {
  const btn = document.getElementById('testSmsCritBtn');
  const status = document.getElementById('testSmsStatus');
  btn.disabled = true;
  btn.innerText = '⏳ Sending…';
  status.innerText = '';
  fetch('/api/test_sms_critical', { method: 'POST' })
    .then((r) => r.json())
    .then((d) => {
      if (d.status === 'sent') {
        btn.innerText = '✅ Alerts Sent!';
        status.innerText = `Sent ${d.sent_count} critical alert(s) — check your phone.`;
        status.style.color = 'var(--green)';
      } else {
        btn.innerText = '❌ Send Failed';
        status.innerText = 'Check PhilSMS token in config.json';
        status.style.color = 'var(--red)';
      }
      setTimeout(() => {
        btn.innerText = '🚨 Test Critical Alerts';
        btn.disabled = false;
        status.innerText = '';
      }, 4000);
    })
    .catch(() => {
      btn.innerText = '❌ Error';
      btn.disabled = false;
    });
}

fetchBinVolumes();
setInterval(fetchBinVolumes, 3000);
// ------------------

// --- WEATHER & TIDE ---
function fetchWeather() {
  fetch('/api/weather')
    .then((r) => r.json())
    .then((d) => {
      const cur = d.current  || {};
      const fh  = d.forecast_1h || {};
      const tide = d.tide    || {};
      const warn = d.warnings || {};

      // -- Current conditions --
      document.getElementById('wx-label').innerText =
        cur.weather_label || '--';
      document.getElementById('wx-label').style.color =
        warn.rain ? 'var(--red)' : warn.bad_weather ? 'var(--amber)' : 'var(--green)';

      document.getElementById('wx-temp').innerText =
        cur.temperature != null ? cur.temperature.toFixed(1) + '°C' : '--';
      document.getElementById('wx-humidity').innerText =
        cur.humidity != null ? cur.humidity + '%' : '--';
      document.getElementById('wx-rain').innerText =
        cur.rain != null ? cur.rain.toFixed(1) + ' mm' : '--';
      document.getElementById('wx-rain').style.color =
        cur.rain > 0 ? 'var(--red)' : 'var(--cyan)';
      document.getElementById('wx-wind').innerText =
        cur.wind_speed != null ? cur.wind_speed.toFixed(0) + ' km/h' : '--';

      // -- Next-hour forecast --
      document.getElementById('fh-label').innerText =
        fh.weather_label || '--';
      document.getElementById('fh-prob').innerText =
        fh.precipitation_prob != null ? fh.precipitation_prob + '% chance of rain' : '--';
      document.getElementById('fh-prob').style.color =
        (fh.precipitation_prob || 0) >= 60 ? 'var(--amber)' : 'var(--cyan)';
      document.getElementById('fh-rain').innerText =
        fh.rain != null ? 'Expected rain: ' + fh.rain.toFixed(1) + ' mm' : 'Expected rain: --';

      // -- Tide --
      if (tide.enabled) {
        document.getElementById('tide-status').innerText =
          tide.is_peak ? '🌊 PEAK HIGH TIDE NOW' : 'Normal';
        document.getElementById('tide-status').style.color =
          tide.is_peak ? 'var(--amber)' : 'var(--green)';
        document.getElementById('tide-next').innerText =
          tide.next_high_in_min != null
            ? `Next high tide in ${tide.next_high_in_min} min  (${tide.next_high_height}m)`
            : 'Next high tide: --';
      } else {
        document.getElementById('tide-status').innerText = 'Tide API not configured';
        document.getElementById('tide-status').style.color = 'var(--text-muted)';
        document.getElementById('tide-next').innerText = 'Add worldtides_api_key to config.json';
      }

      // -- Overlay warnings on video feed --
      const rainOverlay  = document.getElementById('overlay-rain');
      const tideOverlay  = document.getElementById('overlay-tide');
      const badwxOverlay = document.getElementById('overlay-badwx');

      rainOverlay.style.display  = warn.rain      ? 'block' : 'none';
      tideOverlay.style.display  = (!warn.rain && warn.peak_tide)  ? 'block' : 'none';
      badwxOverlay.style.display = (!warn.rain && !warn.peak_tide && warn.bad_weather) ? 'block' : 'none';

      // -- Updated time --
      if (d.last_updated) {
        document.getElementById('wx-updated').innerText = 'Updated: ' + d.last_updated;
      }
    })
    .catch((err) => console.error('Weather fetch error:', err));
}

fetchWeather();
setInterval(fetchWeather, 30000);
// ----------------------

// --- UPDATED: RECTANGULAR RED TAPE BOUNDARY ---
// --- UPDATED: ROTATED RECTANGULAR RED TAPE BOUNDARY ---
function updateRedTapeZone() {
  const left = document.getElementById('slider_rt_left').value;
  const right = document.getElementById('slider_rt_right').value;
  const top = document.getElementById('slider_rt_top').value;
  const bottom = document.getElementById('slider_rt_bottom').value;
  const angle = document.getElementById('slider_rt_angle').value;
  
  fetch('/cmd/vision/red_tape_limit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ 
      rt_left: parseInt(left), 
      rt_right: parseInt(right),
      rt_top: parseInt(top),
      rt_bottom: parseInt(bottom),
      rt_angle: parseInt(angle)
    })
  }).catch(err => console.error('Error updating tape zone:', err));
}