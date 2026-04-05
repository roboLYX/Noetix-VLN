""" @MrRobotoW at The RoboVerse Discord """
""" robert.wagoner@gmail.com """
""" 01/30/2025 """
""" Inspired from lidar_stream.py by @legion1581 at The RoboVerse Discord """

VERSION = "1.0.18"

import asyncio
import logging
import csv
import numpy as np
from flask import Flask, jsonify, render_template_string
from flask_socketio import SocketIO
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection, WebRTCConnectionMethod
import argparse
from datetime import datetime
import os
import sys
import ast
import time
from threading import Lock

# Increase the field size limit for CSV reading
csv.field_size_limit(sys.maxsize)

# Flask app and SocketIO setup
app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

logging.basicConfig(level=logging.INFO)
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# Constants to enable/disable features
ENABLE_POINT_CLOUD = True
SAVE_LIDAR_DATA = True

# File paths
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
LIDAR_CSV_FILE = f"lidar_data_{timestamp}.csv"

# Global variables
lidar_csv_file = None
lidar_csv_writer = None

lidar_buffer = []
message_count = 0  # Counter for processed LIDAR messages
reconnect_interval = 5  # Time (seconds) before retrying connection
latest_lidar_frame = None
latest_lidar_lock = Lock()
latest_lidar_status = {"timestamp": None, "message": None}
latest_lidar_status_lock = Lock()

# Constants
MAX_RETRY_ATTEMPTS = 10

ROTATE_X_ANGLE = np.pi / 2  # 90 degrees
ROTATE_Z_ANGLE = np.pi      # 90 degrees

LIDAR_TOPIC_OPTIONS = {
    "voxel_map": "rt/utlidar/voxel_map",
    "voxel_map_compressed": "rt/utlidar/voxel_map_compressed",
}

minYValue = 0
maxYValue = 100

# Parse command-line arguments
parser = argparse.ArgumentParser(description=f"LIDAR Viz v{VERSION}")
parser.add_argument("--version", action="version", version=f"LIDAR Viz v{VERSION}")
parser.add_argument("--cam-center", action="store_true", help="Put Camera at the Center")
parser.add_argument("--type-voxel", action="store_true", help="Voxel View")
parser.add_argument("--csv-read", type=str, help="Read from CSV files instead of WebRTC")
parser.add_argument("--csv-write", action="store_true", help="Write CSV data file")
parser.add_argument("--skip-mod", type=int, default=1, help="Skip messages using modulus (default: 1, no skipping)")
parser.add_argument(
    "--lidar-topic",
    choices=["auto", "voxel_map", "voxel_map_compressed"],
    default="auto",
    help="LiDAR WebRTC topic to subscribe to. 'auto' subscribes to both voxel topics and uses the first one that produces frames.",
)
parser.add_argument('--minYValue', type=int, default=0, help='Minimum Y value for the plot')
parser.add_argument('--maxYValue', type=int, default=100, help='Maximum Y value for the plot')
args = parser.parse_args()

minYValue = args.minYValue
maxYValue = args.maxYValue
SAVE_LIDAR_DATA = args.csv_write


def get_type_flag_binary():
    typeFlag = 0b0101  # default iso cam & point cloud
    if args.cam_center:
        typeFlag |= 0b0010
    if args.type_voxel:
        typeFlag &= ~0b0100
        typeFlag |= 0b1000
    return format(typeFlag, "04b")


def get_requested_lidar_topics():
    if args.lidar_topic == "auto":
        return [
            LIDAR_TOPIC_OPTIONS["voxel_map_compressed"],
            LIDAR_TOPIC_OPTIONS["voxel_map"],
        ]
    return [LIDAR_TOPIC_OPTIONS[args.lidar_topic]]


def store_latest_lidar_frame(points, scalars, center, total_points, unique_points_count, source, topic):
    global latest_lidar_frame

    payload = {
        "ready": True,
        "seq": message_count,
        "source": source,
        "topic": topic,
        "timestamp": time.time(),
        "total_points": int(total_points),
        "unique_points": int(unique_points_count),
        "points": points.tolist(),
        "scalars": scalars.tolist(),
        "center": center,
    }

    with latest_lidar_lock:
        latest_lidar_frame = payload


def store_latest_lidar_status(message):
    global latest_lidar_status

    with latest_lidar_status_lock:
        latest_lidar_status = {
            "timestamp": time.time(),
            "message": message,
        }

@socketio.on('check_args')
def handle_check_args():
    global ack_received
    socketio.emit("check_args_ack", {"type": get_type_flag_binary()})

def setup_csv_output():
    """Set up CSV files for LIDAR output."""
    global lidar_csv_file, lidar_csv_writer

    if SAVE_LIDAR_DATA:
        lidar_csv_file = open(LIDAR_CSV_FILE, mode='w', newline='', encoding='utf-8')
        lidar_csv_writer = csv.writer(lidar_csv_file)
        lidar_csv_writer.writerow(['stamp', 'frame_id', 'resolution', 'src_size', 'origin', 'width', 
                                   'point_count', 'positions'])
        lidar_csv_file.flush()  # Ensure the header row is flushed to disk

def close_csv_output():
    """Close CSV files."""
    global lidar_csv_file

    if lidar_csv_file:
        lidar_csv_file.close()
        lidar_csv_file = None

def filter_points(points, percentage):
    """Filter points to skip plotting points within a certain percentage of distance to each other."""
    return points  # No filtering

def rotate_points(points, x_angle, z_angle):
    """Rotate points around the x and z axes by given angles."""
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(x_angle), -np.sin(x_angle)],
        [0, np.sin(x_angle), np.cos(x_angle)]
    ])
    
    rotation_matrix_z = np.array([
        [np.cos(z_angle), -np.sin(z_angle), 0],
        [np.sin(z_angle), np.cos(z_angle), 0],
        [0, 0, 1]
    ])
    
    points = points @ rotation_matrix_x.T
    points = points @ rotation_matrix_z.T
    return points

async def lidar_webrtc_connection():
    """Connect to WebRTC and process LIDAR data."""
    global lidar_buffer, message_count
    retry_attempts = 0

    # checkArgs()

    while retry_attempts < MAX_RETRY_ATTEMPTS:
        try:
            # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.123.18")  # WebRTC IP
            conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D5000P817E704", username="liuyixuan163@gmail.com", password="Lyx20050528")
            # conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)

            # Connect to WebRTC
            logging.info("Connecting to WebRTC...")
            await conn.connect()
            logging.info("Connected to WebRTC.")
            retry_attempts = 0  # Reset retry attempts on successful connection

            # Disable traffic saving mode
            await conn.datachannel.disableTrafficSaving(True)

            # Turn LIDAR sensor on
            conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")

            # Set up CSV outputs
            setup_csv_output()
            lidar_frame_seen = False
            no_frame_warning_logged = False
            subscription_started_at = time.time()
            active_lidar_topic = None
            subscribed_lidar_topics = get_requested_lidar_topics()

            async def lidar_callback_task(message, topic):
                """Task to process incoming LIDAR data."""
                nonlocal active_lidar_topic, lidar_frame_seen
                if not ENABLE_POINT_CLOUD:
                    return

                try:
                    if active_lidar_topic is not None and topic != active_lidar_topic:
                        return

                    lidar_frame_seen = True
                    global message_count
                    if message_count % args.skip_mod != 0:
                        message_count += 1
                        return

                    decoded_data = message["data"]["data"]
                    positions = decoded_data.get("positions")
                    raw_points = decoded_data.get("points")

                    if positions is not None:
                        points = np.array([positions[i:i+3] for i in range(0, len(positions), 3)], dtype=np.float32)
                    elif raw_points is not None:
                        points = np.array(raw_points, dtype=np.float32)
                    else:
                        logging.warning(f"LiDAR message from {topic} does not contain 'positions' or 'points'")
                        return

                    total_points = len(points)
                    unique_points = np.unique(points, axis=0)

                    if active_lidar_topic is None:
                        active_lidar_topic = topic
                        logging.info(f"Using LiDAR topic: {active_lidar_topic}")

                    # Save to CSV
                    if SAVE_LIDAR_DATA and lidar_csv_writer:
                        lidar_csv_writer.writerow([
                            message["data"]["stamp"],
                            message["data"]["frame_id"],
                            message["data"]["resolution"],
                            message["data"]["src_size"],
                            message["data"]["origin"],
                            message["data"]["width"],
                            len(unique_points),
                            unique_points.tolist()  # Save full data
                        ])
                        lidar_csv_file.flush()  # Flush data to disk

                    points = rotate_points(unique_points, ROTATE_X_ANGLE, ROTATE_Z_ANGLE)  # Rotate points
                    points = points[(points[:, 1] >= minYValue) & (points[:, 1] <= maxYValue)]

                    if points.size == 0:
                        logging.warning("LiDAR frame is empty after filtering. Adjust --minYValue/--maxYValue if needed.")
                        message_count += 1
                        store_latest_lidar_frame(
                            np.empty((0, 3), dtype=np.float32),
                            np.empty((0,), dtype=np.float32),
                            {"x": 0.0, "y": 0.0, "z": 0.0},
                            total_points,
                            len(unique_points),
                            "webrtc",
                            topic,
                        )
                        return

                    # Calculate center coordinates
                    center_x = float(np.mean(points[:, 0]))
                    center_y = float(np.mean(points[:, 1]))
                    center_z = float(np.mean(points[:, 2]))

                    # Offset points by center coordinates
                    offset_points = points - np.array([center_x, center_y, center_z])

                    # Count and log points
                    message_count += 1
                    print(f"LIDAR Message {message_count}: Total points={total_points}, Unique points={len(unique_points)}")

                    # Emit data to Socket.IO
                    scalars = np.linalg.norm(offset_points, axis=1)  # Color by distance
                    center = {"x": center_x, "y": center_y, "z": center_z}
                    store_latest_lidar_frame(
                        offset_points,
                        scalars,
                        center,
                        total_points,
                        len(unique_points),
                        "webrtc",
                        topic,
                    )
                    socketio.emit("lidar_data", {
                        "points": offset_points.tolist(),
                        "scalars": scalars.tolist(),
                        "center": center,
                        "topic": topic,
                    })

                except Exception as e:
                    logging.error(f"Error in LIDAR callback: {e}")

            def lidar_state_callback(message):
                store_latest_lidar_status(message)
                logging.info(f"LIDAR state update: {message}")

            # Subscribe to the requested LiDAR WebRTC topics.
            for topic in subscribed_lidar_topics:
                conn.datachannel.pub_sub.subscribe(
                    topic,
                    lambda message, source_topic=topic: asyncio.create_task(
                        lidar_callback_task(message, source_topic)
                    )
                )
                logging.info(f"Subscribed to LiDAR topic: {topic}")
            conn.datachannel.pub_sub.subscribe("rt/utlidar/lidar_state", lidar_state_callback)

            # Keep the connection active
            while True:
                if not lidar_frame_seen and not no_frame_warning_logged and conn.isConnected:
                    if time.time() - subscription_started_at > 5:
                        logging.warning(
                            "WebRTC is connected, but no LiDAR frames arrived within 5 seconds after subscription. "
                            f"Subscribed topics: {subscribed_lidar_topics}. "
                            "Likely causes: current robot/firmware does not expose these WebRTC LiDAR topics, "
                            "LiDAR service is not active, or this model does not support this topic over WebRTC."
                        )
                        no_frame_warning_logged = True
                await asyncio.sleep(1)

        except Exception as e:
            logging.error(f"An error occurred: {e}")
            logging.info(f"Reconnecting in {reconnect_interval} seconds... (Attempt {retry_attempts + 1}/{MAX_RETRY_ATTEMPTS})")
            close_csv_output()
            try:
                await conn.disconnect()
            except Exception as e:
                logging.error(f"Error during disconnect: {e}")
            await asyncio.sleep(reconnect_interval)
            retry_attempts += 1

    logging.error("Max retry attempts reached. Exiting.")

async def read_csv_and_emit(csv_file):
    """Continuously read CSV files and emit data without delay."""
    global message_count

    # checkArgs()

    while True:  # Infinite loop to restart at EOF
        try:
            total_messages = sum(1 for _ in open(csv_file)) - 1  # Calculate total messages

            with open(csv_file, mode='r', newline='', encoding='utf-8') as lidar_file:
                lidar_reader = csv.DictReader(lidar_file)

                for lidar_row in lidar_reader:
                    if message_count % args.skip_mod == 0:
                        try:
                            # Extract and validate positions
                            positions = ast.literal_eval(lidar_row.get("positions", "[]"))
                            if isinstance(positions, list) and all(isinstance(item, list) and len(item) == 3 for item in positions):
                                points = np.array(positions, dtype=np.float32)
                            else:
                                points = np.array([item for item in positions if isinstance(item, list) and len(item) == 3], dtype=np.float32)

                            # Extract and compute origin, resolution, width, and center
                            origin = np.array(eval(lidar_row.get("origin", "[]")), dtype=np.float32)
                            resolution = float(lidar_row.get("resolution", 0.05))
                            width = np.array(eval(lidar_row.get("width", "[128, 128, 38]")), dtype=np.float32)
                            center = origin + (width * resolution) / 2

                            # Process points
                            if points.size > 0:
                                points = rotate_points(points, ROTATE_X_ANGLE, ROTATE_Z_ANGLE)
                                points = points[(points[:, 1] >= minYValue) & (points[:, 1] <= maxYValue)]
                                unique_points = np.unique(points, axis=0)
                                if unique_points.size > 0:
                                    # Calculate center coordinates
                                    center_x = float(np.mean(unique_points[:, 0]))
                                    center_y = float(np.mean(unique_points[:, 1]))
                                    center_z = float(np.mean(unique_points[:, 2]))

                                    # Offset points by center coordinates
                                    offset_points = unique_points - np.array([center_x, center_y, center_z])
                                else:
                                    center_x = 0.0
                                    center_y = 0.0
                                    center_z = 0.0
                                    offset_points = np.empty((0, 3), dtype=np.float32)
                            else:
                                center_x = 0.0
                                center_y = 0.0
                                center_z = 0.0
                                unique_points = np.empty((0, 3), dtype=np.float32)
                                offset_points = unique_points

                            # Emit data to Socket.IO
                            scalars = np.linalg.norm(offset_points, axis=1)
                            center = {"x": center_x, "y": center_y, "z": center_z}
                            store_latest_lidar_frame(
                                offset_points,
                                scalars,
                                center,
                                len(points),
                                len(unique_points),
                                "csv",
                                "csv-replay",
                            )
                            socketio.emit("lidar_data", {
                                "points": offset_points.tolist(),
                                "scalars": scalars.tolist(),
                                "center": center,
                                "topic": "csv-replay",
                            })

                            # Print message details
                            print(f"LIDAR Message {message_count}/{total_messages}: Unique points={len(unique_points)}")

                        except Exception as e:
                            logging.error(f"Exception during processing: {e}")

                    # Increment message count
                    message_count += 1

            # Restart file reading when EOF is reached
            message_count = 0  # Reset counter if needed

        except Exception as e:
            logging.error(f"Error reading CSV file: {e}")


@app.route("/api/config")
def config():
    response = jsonify({
        "type": get_type_flag_binary(),
        "view": "voxel" if args.type_voxel else "point_cloud",
        "lidarTopicMode": args.lidar_topic,
        "requestedTopics": get_requested_lidar_topics(),
        "minYValue": minYValue,
        "maxYValue": maxYValue,
    })
    response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/api/lidar/latest")
def latest_lidar():
    with latest_lidar_lock:
        payload = dict(latest_lidar_frame) if latest_lidar_frame is not None else {
            "ready": False,
            "seq": message_count,
            "source": "webrtc" if not args.csv_read else "csv",
            "topic": None,
            "timestamp": None,
            "total_points": 0,
            "unique_points": 0,
            "points": [],
            "scalars": [],
            "center": {"x": 0.0, "y": 0.0, "z": 0.0},
        }

    with latest_lidar_status_lock:
        payload["lidar_state"] = dict(latest_lidar_status)
    payload["requestedTopics"] = get_requested_lidar_topics()

    response = jsonify(payload)
    response.headers["Cache-Control"] = "no-store"
    return response

@app.route("/")
def index():
    return render_template_string("""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <title>LIDAR Viz v{{ version }}</title>
        <style>
            :root {
                color-scheme: dark;
                --bg: #0a0f14;
                --panel: rgba(10, 15, 20, 0.78);
                --border: rgba(128, 194, 255, 0.28);
                --text: #e9f4ff;
                --muted: #9cb6c9;
                --accent: #5cc8ff;
            }

            * {
                box-sizing: border-box;
            }

            body {
                margin: 0;
                overflow: hidden;
                background:
                    radial-gradient(circle at top, rgba(92, 200, 255, 0.12), transparent 32%),
                    linear-gradient(180deg, #0d141b 0%, #070b10 100%);
                font-family: "Segoe UI", "PingFang SC", sans-serif;
                color: var(--text);
            }

            #lidar-canvas {
                display: block;
                width: 100vw;
                height: 100vh;
            }

            .hud {
                position: fixed;
                top: 18px;
                left: 18px;
                padding: 14px 16px;
                min-width: 320px;
                max-width: min(92vw, 520px);
                border: 1px solid var(--border);
                border-radius: 14px;
                background: var(--panel);
                backdrop-filter: blur(12px);
                box-shadow: 0 20px 50px rgba(0, 0, 0, 0.35);
            }

            .title {
                margin: 0 0 8px;
                font-size: 18px;
                font-weight: 700;
                letter-spacing: 0.02em;
            }

            .status {
                margin: 0 0 8px;
                color: var(--accent);
                font-size: 14px;
            }

            .stats,
            .tips {
                margin: 0;
                color: var(--muted);
                font-size: 13px;
                line-height: 1.45;
            }

            .tips {
                margin-top: 8px;
            }
        </style>
    </head>
    <body>
        <canvas id="lidar-canvas"></canvas>
        <div class="hud">
            <p class="title">Go2 LiDAR Viewer v{{ version }}</p>
            <p id="status" class="status">Connecting to local viewer...</p>
            <p id="stats" class="stats">Waiting for LiDAR frames.</p>
            <p class="tips">Offline mode: this page does not use CDN assets. Drag to rotate, wheel to zoom.</p>
        </div>
        <script>
            const canvas = document.getElementById("lidar-canvas");
            const ctx = canvas.getContext("2d");
            const statusEl = document.getElementById("status");
            const statsEl = document.getElementById("stats");

            let latestFrame = null;
            let lastSeq = -1;
            let pointStyle = "point";
            let rotationX = -0.6;
            let rotationY = -0.85;
            let zoom = 5.5;
            let dragging = false;
            let lastMouseX = 0;
            let lastMouseY = 0;

            function resizeCanvas() {
                const ratio = window.devicePixelRatio || 1;
                const width = window.innerWidth;
                const height = window.innerHeight;
                canvas.width = Math.floor(width * ratio);
                canvas.height = Math.floor(height * ratio);
                canvas.style.width = width + "px";
                canvas.style.height = height + "px";
                ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
            }

            function updateHud(frame) {
                if (!frame || !frame.ready) {
                    statusEl.textContent = "Waiting for LiDAR frames...";
                    const stateText = frame && frame.lidar_state && frame.lidar_state.message
                        ? " | lidar_state: " + JSON.stringify(frame.lidar_state.message)
                        : "";
                    const waitingTopicText = frame && frame.requestedTopics
                        ? " | requested: " + frame.requestedTopics.join(", ")
                        : "";
                    statsEl.textContent = "No decoded frame yet. If this stays empty, check terminal logs and Y filters." + waitingTopicText + stateText;
                    return;
                }

                const ageMs = Math.max(0, Math.round((Date.now() / 1000 - frame.timestamp) * 1000));
                const stateText = frame.lidar_state && frame.lidar_state.message
                    ? " | lidar_state: " + JSON.stringify(frame.lidar_state.message)
                    : "";
                statusEl.textContent = "LiDAR stream active";
                statsEl.textContent =
                    "Frame " + frame.seq +
                    " | source: " + frame.source +
                    " | topic: " + (frame.topic || "unknown") +
                    " | total: " + frame.total_points +
                    " | unique: " + frame.unique_points +
                    " | shown: " + frame.points.length +
                    " | age: " + ageMs + " ms" +
                    stateText;
            }

            async function fetchConfig() {
                try {
                    const response = await fetch("/api/config", { cache: "no-store" });
                    const config = await response.json();
                    const typeFlag = parseInt(config.type, 2);
                    pointStyle = (typeFlag & 0b1000) ? "voxel" : "point";
                    statusEl.textContent = "Viewer ready. Waiting for LiDAR frames...";
                    statsEl.textContent = "Requested WebRTC LiDAR topics: " + config.requestedTopics.join(", ");
                } catch (error) {
                    statusEl.textContent = "Failed to load viewer config";
                    statsEl.textContent = String(error);
                }
            }

            async function pollLatestFrame() {
                try {
                    const response = await fetch("/api/lidar/latest", { cache: "no-store" });
                    const frame = await response.json();
                    if (frame.ready && frame.seq !== lastSeq) {
                        latestFrame = frame;
                        lastSeq = frame.seq;
                    } else if (!frame.ready) {
                        latestFrame = frame;
                    }
                    updateHud(frame);
                } catch (error) {
                    statusEl.textContent = "Polling failed";
                    statsEl.textContent = String(error);
                } finally {
                    window.setTimeout(pollLatestFrame, 250);
                }
            }

            function rotatePoint(point) {
                let x = point[0];
                let y = point[1];
                let z = point[2];

                const cosY = Math.cos(rotationY);
                const sinY = Math.sin(rotationY);
                const rotatedX = x * cosY - z * sinY;
                const rotatedZ = x * sinY + z * cosY;

                const cosX = Math.cos(rotationX);
                const sinX = Math.sin(rotationX);
                const finalY = y * cosX - rotatedZ * sinX;
                const finalZ = y * sinX + rotatedZ * cosX;

                return [rotatedX, finalY, finalZ];
            }

            function projectPoint(point) {
                const rotated = rotatePoint(point);
                const cameraDistance = 150;
                const depth = rotated[2] + cameraDistance;
                const scale = (zoom * Math.min(window.innerWidth, window.innerHeight)) / Math.max(depth, 30);
                return {
                    x: window.innerWidth / 2 + rotated[0] * scale,
                    y: window.innerHeight / 2 - rotated[1] * scale,
                    depth: depth,
                };
            }

            function drawAxis(start, end, color) {
                const from = projectPoint(start);
                const to = projectPoint(end);
                ctx.strokeStyle = color;
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.moveTo(from.x, from.y);
                ctx.lineTo(to.x, to.y);
                ctx.stroke();
            }

            function drawBackground() {
                const gradient = ctx.createLinearGradient(0, 0, 0, window.innerHeight);
                gradient.addColorStop(0, "#101922");
                gradient.addColorStop(1, "#04070b");
                ctx.fillStyle = gradient;
                ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);

                ctx.strokeStyle = "rgba(92, 200, 255, 0.08)";
                ctx.lineWidth = 1;
                for (let x = 0; x < window.innerWidth; x += 40) {
                    ctx.beginPath();
                    ctx.moveTo(x, 0);
                    ctx.lineTo(x, window.innerHeight);
                    ctx.stroke();
                }
                for (let y = 0; y < window.innerHeight; y += 40) {
                    ctx.beginPath();
                    ctx.moveTo(0, y);
                    ctx.lineTo(window.innerWidth, y);
                    ctx.stroke();
                }
            }

            function drawFrame() {
                drawBackground();

                drawAxis([0, 0, 0], [20, 0, 0], "#ff6b6b");
                drawAxis([0, 0, 0], [0, 20, 0], "#8cff8c");
                drawAxis([0, 0, 0], [0, 0, 20], "#6bb7ff");

                if (!latestFrame || !latestFrame.ready || !latestFrame.points.length) {
                    ctx.fillStyle = "rgba(233, 244, 255, 0.85)";
                    ctx.font = "16px Segoe UI";
                    ctx.fillText("No LiDAR points rendered yet.", 28, window.innerHeight - 28);
                    window.requestAnimationFrame(drawFrame);
                    return;
                }

                const points = latestFrame.points;
                const scalars = latestFrame.scalars || [];
                const maxScalar = Math.max(...scalars, 1);
                const projected = [];

                for (let i = 0; i < points.length; i++) {
                    const projection = projectPoint(points[i]);
                    projected.push({
                        x: projection.x,
                        y: projection.y,
                        depth: projection.depth,
                        scalar: scalars[i] || 0,
                    });
                }

                projected.sort((a, b) => b.depth - a.depth);

                for (const point of projected) {
                    const normalized = point.scalar / maxScalar;
                    const hue = 210 - normalized * 210;
                    const size = pointStyle === "voxel" ? 6 : 3;

                    ctx.fillStyle = "hsl(" + hue + ", 100%, 60%)";
                    if (pointStyle === "voxel") {
                        ctx.fillRect(point.x - size / 2, point.y - size / 2, size, size);
                    } else {
                        ctx.beginPath();
                        ctx.arc(point.x, point.y, size / 2, 0, Math.PI * 2);
                        ctx.fill();
                    }
                }

                window.requestAnimationFrame(drawFrame);
            }

            canvas.addEventListener("mousedown", (event) => {
                dragging = true;
                lastMouseX = event.clientX;
                lastMouseY = event.clientY;
            });

            window.addEventListener("mouseup", () => {
                dragging = false;
            });

            window.addEventListener("mousemove", (event) => {
                if (!dragging) {
                    return;
                }

                const dx = event.clientX - lastMouseX;
                const dy = event.clientY - lastMouseY;
                rotationY += dx * 0.01;
                rotationX += dy * 0.01;
                rotationX = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, rotationX));
                lastMouseX = event.clientX;
                lastMouseY = event.clientY;
            });

            canvas.addEventListener("wheel", (event) => {
                event.preventDefault();
                const factor = event.deltaY > 0 ? 0.92 : 1.08;
                zoom = Math.max(1.5, Math.min(24, zoom * factor));
            }, { passive: false });

            window.addEventListener("resize", resizeCanvas);

            resizeCanvas();
            fetchConfig();
            pollLatestFrame();
            window.requestAnimationFrame(drawFrame);
        </script>
    </body>
    </html>
    """, version=VERSION)

def start_webrtc():
    """Run WebRTC connection in a separate asyncio loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(lidar_webrtc_connection())

if __name__ == "__main__":
    import threading
    if args.csv_read:
        csv_thread = threading.Thread(target=lambda: asyncio.run(read_csv_and_emit(args.csv_read)), daemon=True)
        csv_thread.start()
    else:
        webrtc_thread = threading.Thread(target=start_webrtc, daemon=True)
        webrtc_thread.start()

    socketio.run(app, host="127.0.0.1", port=8080, debug=False)
