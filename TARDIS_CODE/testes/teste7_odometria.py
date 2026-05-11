import bluerobotics_navigator as navigator
import time
import threading
import numpy as np
from math import cos, sin, atan2, sqrt, pi
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit

# ------------------- CONFIGURATION -------------------
GRAVITY = 9.80665
UPDATE_INTERVAL = 0.1
MAX_HISTORY = 200

# Complementary filter gain (high value trusts gyro more)
ALPHA = 0.98
BETA = 1.0 - ALPHA

# Global state for orientation
roll_filtered = 0.0
pitch_filtered = 0.0
yaw_filtered = 0.0
last_time = None

# Initialize Navigator
navigator.init()
print("Navigator initialized.")

# Data buffer
data_lock = threading.Lock()
sensor_data = {
    'timestamp': [],
    'accel_x': [], 'accel_y': [], 'accel_z': [],
    'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
    'mag_x': [], 'mag_y': [], 'mag_z': [],
    'roll': [], 'pitch': [], 'yaw': [],
    'pos_x': [], 'pos_y': []
}

vel_x, vel_y = 0.0, 0.0

# ------------------- HELPER FUNCTIONS -------------------
def rotate_vector(v, yaw):
    """Rotate 3D vector from body to world (only yaw)."""
    c, s = cos(yaw), sin(yaw)
    return np.array([c*v[0] - s*v[1], s*v[0] + c*v[1], v[2]])

def acc_to_roll_pitch(ax, ay, az):
    """Compute roll and pitch from accelerometer data (radians)."""
    roll = atan2(ay, az)
    pitch = atan2(-ax, sqrt(ay*ay + az*az))
    return roll, pitch

def tilt_compensated_magnetometer(mx, my, mz, roll, pitch):
    """Apply tilt compensation to magnetometer to get heading."""
    # Rotate magnetometer vector by roll and pitch (from body to horizontal)
    mx2 = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch)
    my2 = my * cos(roll) - mz * sin(roll)
    yaw = atan2(my2, mx2)  # heading from magnetic north
    return yaw

def complementary_filter(prev_angle, gyro_rate, sensor_angle, dt):
    """Complementary filter update."""
    return ALPHA * (prev_angle + gyro_rate * dt) + BETA * sensor_angle

# ------------------- SENSOR THREAD -------------------
def update_sensor_data():
    global roll_filtered, pitch_filtered, yaw_filtered, last_time
    global vel_x, vel_y

    while True:
        now = time.time()
        dt = now - last_time if last_time else 0.02
        last_time = now

        # Read sensors (correct methods)
        acc = navigator.read_accel()
        gyro = navigator.read_gyro()
        mag = navigator.read_mag()

        ax, ay, az = acc.x, acc.y, acc.z
        gx, gy, gz = gyro.x, gyro.y, gyro.z
        mx, my, mz = mag.x, mag.y, mag.z

        # ---- Complementary filter for orientation ----
        # 1. Roll and pitch from accelerometer (gravity vector)
        roll_acc, pitch_acc = acc_to_roll_pitch(ax, ay, az)

        # 2. Integrate gyroscope rates
        roll_gyro = roll_filtered + gx * dt
        pitch_gyro = pitch_filtered + gy * dt
        yaw_gyro = yaw_filtered + gz * dt

        # 3. Combine roll and pitch
        roll_filtered = complementary_filter(roll_filtered, gx, roll_acc, dt)
        pitch_filtered = complementary_filter(pitch_filtered, gy, pitch_acc, dt)

        # 4. Yaw: use tilt-compensated magnetometer heading
        yaw_mag = tilt_compensated_magnetometer(mx, my, mz, roll_filtered, pitch_filtered)

        # Combine yaw
        yaw_filtered = complementary_filter(yaw_filtered, gz, yaw_mag, dt)

        # Normalize yaw to [-pi, pi]
        yaw_filtered = atan2(sin(yaw_filtered), cos(yaw_filtered))

        # ---- Odometry (using world‑frame acceleration) ----
        # Remove gravity from accelerometer (using filtered roll/pitch)
        # Convert body acceleration to world frame using full orientation (roll, pitch, yaw)
        # For simplicity we still use only yaw rotation (as before), but we could use full rotation.
        a_body = np.array([ax, ay, az])
        # Remove gravity component based on current roll/pitch
        # gravity_vector_body = [ -sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch) ]
        # but simpler: rotate a_body to world and subtract [0,0,9.81] in world?
        # Actually, we can keep the previous method that subtracts gravity from Z in body frame.
        # This is approximate but works for small inclinations.
        # Let's use the same approach as before (remove 9.81 from az) because full rotation might accumulate error.
        a_body[2] -= GRAVITY

        # Rotate to world using only yaw (horizontal components)
        a_world = rotate_vector(a_body, yaw_filtered)

        vel_x += a_world[0] * dt
        vel_y += a_world[1] * dt

        with data_lock:
            if sensor_data['pos_x']:
                pos_x = sensor_data['pos_x'][-1] + vel_x * dt
                pos_y = sensor_data['pos_y'][-1] + vel_y * dt
            else:
                pos_x = vel_x * dt
                pos_y = vel_y * dt

            # Append to buffers
            sensor_data['timestamp'].append(now)
            sensor_data['accel_x'].append(ax); sensor_data['accel_y'].append(ay); sensor_data['accel_z'].append(az)
            sensor_data['gyro_x'].append(gx); sensor_data['gyro_y'].append(gy); sensor_data['gyro_z'].append(gz)
            sensor_data['mag_x'].append(mx); sensor_data['mag_y'].append(my); sensor_data['mag_z'].append(mz)
            sensor_data['roll'].append(roll_filtered); sensor_data['pitch'].append(pitch_filtered); sensor_data['yaw'].append(yaw_filtered)
            sensor_data['pos_x'].append(pos_x); sensor_data['pos_y'].append(pos_y)

            # Keep history limited
            for key in sensor_data:
                if len(sensor_data[key]) > MAX_HISTORY:
                    sensor_data[key].pop(0)

        # Optional print for debugging (uncomment if needed)
        # print(f"Roll: {roll_filtered:.2f}, Pitch: {pitch_filtered:.2f}, Yaw: {yaw_filtered:.2f}")

        time.sleep(0.01)   # high rate reading

# ------------------- WEB SERVER -------------------
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')  # simpler async

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Navigator Real‑time Telemetry</title>
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
        .container { display: flex; flex-wrap: wrap; gap: 20px; }
        .plot { background: white; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); padding: 10px; flex: 1 1 400px; }
        h3 { margin: 0 0 10px 0; }
    </style>
</head>
<body>
    <h1>BlueRobotics Navigator – Odometria em Tempo Real</h1>
    <div class="container">
        <div class="plot"><h3>Aceleração (m/s²)</h3><div id="accelPlot"></div></div>
        <div class="plot"><h3>Giroscópio (rad/s)</h3><div id="gyroPlot"></div></div>
        <div class="plot"><h3>Magnetômetro (µT)</h3><div id="magPlot"></div></div>
        <div class="plot"><h3>Trajetória (x,y) e Yaw</h3><div id="trajPlot"></div></div>
    </div>
    <script>
        const socket = io();

        const accelDiv = document.getElementById('accelPlot');
        const gyroDiv = document.getElementById('gyroPlot');
        const magDiv = document.getElementById('magPlot');
        const trajDiv = document.getElementById('trajPlot');

        let timeData = [];
        let accelX = [], accelY = [], accelZ = [];
        let gyroX = [], gyroY = [], gyroZ = [];
        let magX = [], magY = [], magZ = [];
        let posX = [], posY = [], yawData = [];

        const accelLayout = { title: '', xaxis: { title: 'Tempo (s)' }, yaxis: { title: 'm/s²' }, showlegend: true };
        const gyroLayout = { title: '', xaxis: { title: 'Tempo (s)' }, yaxis: { title: 'rad/s' }, showlegend: true };
        const magLayout = { title: '', xaxis: { title: 'Tempo (s)' }, yaxis: { title: 'µT' }, showlegend: true };
        const trajLayout = { title: '', xaxis: { title: 'X (m)', scaleanchor: 'y' }, yaxis: { title: 'Y (m)' }, showlegend: true };

        Plotly.newPlot(accelDiv, [], accelLayout);
        Plotly.newPlot(gyroDiv, [], gyroLayout);
        Plotly.newPlot(magDiv, [], magLayout);
        Plotly.newPlot(trajDiv, [], trajLayout);

        socket.on('update', function(data) {
            console.log("Received update", data.t.length);  // debug
            timeData = data.t;
            accelX = data.ax; accelY = data.ay; accelZ = data.az;
            gyroX = data.gx; gyroY = data.gy; gyroZ = data.gz;
            magX = data.mx; magY = data.my; magZ = data.mz;
            posX = data.px; posY = data.py;
            yawData = data.yaw;

            Plotly.react(accelDiv, [
                { x: timeData, y: accelX, mode: 'lines', name: 'ax' },
                { x: timeData, y: accelY, mode: 'lines', name: 'ay' },
                { x: timeData, y: accelZ, mode: 'lines', name: 'az' }
            ], accelLayout);

            Plotly.react(gyroDiv, [
                { x: timeData, y: gyroX, mode: 'lines', name: 'gx' },
                { x: timeData, y: gyroY, mode: 'lines', name: 'gy' },
                { x: timeData, y: gyroZ, mode: 'lines', name: 'gz' }
            ], gyroLayout);

            Plotly.react(magDiv, [
                { x: timeData, y: magX, mode: 'lines', name: 'mx' },
                { x: timeData, y: magY, mode: 'lines', name: 'my' },
                { x: timeData, y: magZ, mode: 'lines', name: 'mz' }
            ], magLayout);

            const traceTraj = { x: posX, y: posY, mode: 'lines+markers', name: 'Trajetória', marker: { size: 3 } };
            if(posX.length > 0) {
                const lastX = posX[posX.length-1];
                const lastY = posY[posY.length-1];
                const lastYaw = yawData[yawData.length-1];
                const arrowLength = 0.3;
                const arrowX = [lastX, lastX + arrowLength * Math.cos(lastYaw)];
                const arrowY = [lastY, lastY + arrowLength * Math.sin(lastYaw)];
                const traceArrow = { x: arrowX, y: arrowY, mode: 'lines+markers', name: 'Orientação', line: { color: 'red', width: 3 }, marker: { size: 0 } };
                Plotly.react(trajDiv, [traceTraj, traceArrow], trajLayout);
            } else {
                Plotly.react(trajDiv, [traceTraj], trajLayout);
            }
        });

        // Optional: periodic reconnect check
        socket.on('connect', () => console.log('Socket connected'));
        socket.on('disconnect', () => console.log('Socket disconnected'));
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

def send_data_thread():
    """Thread that periodically emits sensor data to all clients."""
    while True:
        with data_lock:
            t = sensor_data['timestamp'][:]
            ax = sensor_data['accel_x'][:]; ay = sensor_data['accel_y'][:]; az = sensor_data['accel_z'][:]
            gx = sensor_data['gyro_x'][:]; gy = sensor_data['gyro_y'][:]; gz = sensor_data['gyro_z'][:]
            mx = sensor_data['mag_x'][:]; my = sensor_data['mag_y'][:]; mz = sensor_data['mag_z'][:]
            px = sensor_data['pos_x'][:]; py = sensor_data['pos_y'][:]
            yaw = sensor_data['yaw'][:]

        data = {
            't': t,
            'ax': ax, 'ay': ay, 'az': az,
            'gx': gx, 'gy': gy, 'gz': gz,
            'mx': mx, 'my': my, 'mz': mz,
            'px': px, 'py': py,
            'yaw': yaw
        }
        socketio.emit('update', data)
        # print(f"Sent update: {len(t)} points")  # debug
        time.sleep(UPDATE_INTERVAL)

if __name__ == '__main__':
    # Start sensor reading thread
    sensor_thread = threading.Thread(target=update_sensor_data, daemon=True)
    sensor_thread.start()

    # Start data sending thread
    send_thread = threading.Thread(target=send_data_thread, daemon=True)
    send_thread.start()

    print("Web server started. Access http://<RASPBERRY_PI_IP>:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)