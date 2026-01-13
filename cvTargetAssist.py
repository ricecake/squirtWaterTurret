"""
This script provides a target acquisition and identification system using a Luxonis DepthAI camera.
It includes an optional web-based UI for visualization and control.
"""
import os
import sys
import select
import struct
import time
import json
import argparse
import threading
from math import atan2, degrees
from typing import Tuple

import sqlite3
import sqlite_vec
import serial

import cv2
import depthai as dai
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
from werkzeug.serving import make_server

from depthai import RotatedRect
from depthai_nodes import ImgDetectionsExtended, ImgDetectionExtended
from depthai_nodes.node import ParsingNeuralNetwork, GatherData
from depthai_nodes.message.keypoints import Keypoint
from serial_protocol import TargetMessage, SetTargetSourceMessage, StaticTargetMessage, TargetSource

# ======================================================================================
# --- Constants ---
# ======================================================================================
LOG_FILE = 'cvTargetAssist.log'
CONFIG_FILE = 'config.json'

# ======================================================================================
# --- Web Server and API ---
# ======================================================================================
app = Flask(__name__, static_folder='frontend/dist', static_url_path='')
CORS(app)

@app.route('/', defaults={'path': ''})
@app.route('/<path:path>')
def serve(path):
    if path and os.path.exists(os.path.join(app.static_folder, path)):
        return send_from_directory(app.static_folder, path)
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/api/config', methods=['GET', 'POST'])
def handle_config():
    if request.method == 'POST':
        new_config = request.json
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(new_config, f, indent=4)
            ScriptSettings.update(new_config)
            return jsonify({'message': 'Configuration updated successfully.'})
        except IOError:
            return jsonify({'error': 'Failed to write configuration file.'}), 500
    else:
        return jsonify(ScriptSettings.get_config_dict())

@app.route('/api/logs', methods=['GET'])
def get_logs():
    try:
        with open(LOG_FILE, 'r') as f:
            lines = f.readlines()[-100:]
            return jsonify({'logs': ''.join(lines)})
    except FileNotFoundError:
        return jsonify({'logs': 'Log file not found.'})

@app.route('/api/users', methods=['GET'])
def get_users():
    db_path = f"./personDb-{ScriptSettings.MODE}-{ScriptSettings.STATE}.sqlite"
    try:
        with sqlite3.connect(db_path) as conn:
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            cursor.execute("SELECT id, name, type FROM person")
            users = [dict(row) for row in cursor.fetchall()]
            return jsonify(users)
    except sqlite3.Error as e:
        if "no such table" in str(e):
             with sqlite3.connect(db_path) as conn:
                 conn.execute("CREATE TABLE person (id INTEGER PRIMARY KEY, name TEXT, type TEXT)")
             return jsonify([])
        return jsonify({'error': str(e)}), 500

@app.route('/api/users/<int:user_id>', methods=['PUT'])
def update_user(user_id):
    data = request.json
    name, user_type = data.get('name'), data.get('type')
    db_path = f"./personDb-{ScriptSettings.MODE}-{ScriptSettings.STATE}.sqlite"
    if not name or not user_type:
        return jsonify({'error': 'Name and type are required.'}), 400
    try:
        with sqlite3.connect(db_path) as conn:
            cursor = conn.cursor()
            cursor.execute("UPDATE person SET name = ?, type = ? WHERE id = ?", (name, user_type, user_id))
            conn.commit()
            return jsonify({'message': f'User {user_id} updated successfully.'})
    except sqlite3.Error as e:
        return jsonify({'error': str(e)}), 500

class WebServerThread(threading.Thread):
    def __init__(self, app):
        super().__init__(daemon=True)
        self.server = make_server('0.0.0.0', 5000, app)
        self.ctx = app.app_context()
        self.ctx.push()
    def run(self):
        print("Starting Flask server on http://0.0.0.0:5000")
        self.server.serve_forever()
    def shutdown(self):
        self.server.shutdown()

# ======================================================================================
# --- Configuration ---
# ======================================================================================
class ScriptSettings:
    WEB_UI: bool = False
    ENABLE_VISUALIZER: bool = False
    SERIAL_OUTPUT: bool = False
    LOG_LEVEL: dai.LogLevel = dai.LogLevel.WARN
    FPS: int = 10
    CLOSE_MATCH_THRESHOLD: float = 0.16
    MEDIUM_MATCH_THRESHOLD: float = 0.22
    FAR_MATCH_THRESHOLD: float = 0.30
    MODE: str = 'POSE'
    STATE: str = 'TEST'
    HEIGHT: int = 1000

    @classmethod
    def update(cls, config_dict):
        for key, value in config_dict.items():
            attr_key = key.upper()
            if hasattr(cls, attr_key):
                current_type = type(getattr(cls, attr_key))
                if attr_key == 'LOG_LEVEL':
                    setattr(cls, attr_key, getattr(dai.LogLevel, str(value).upper(), dai.LogLevel.WARN))
                else:
                    try:
                        setattr(cls, attr_key, current_type(value))
                    except (ValueError, TypeError):
                        print(f"Warning: Could not convert value '{value}' for setting '{key}'")
    @classmethod
    def get_config_dict(cls):
        return {k.lower(): v for k, v in cls.__dict__.items() if not k.startswith('_') and isinstance(v, (bool, str, int, float, dai.LogLevel))}

def setup_config_and_args():
    try:
        with open(CONFIG_FILE, 'r') as f:
            ScriptSettings.update(json.load(f))
    except (FileNotFoundError, json.JSONDecodeError):
        pass # OK if file doesn't exist

    parser = argparse.ArgumentParser(description="Luxonis DepthAI Target Acquisition System")
    parser.add_argument('--web-ui', action='store_true', help='Enable the web UI.')

    # Create a temporary parser to see which args are specified on the command line
    temp_args, _ = parser.parse_known_args()

    # Update settings from command line, overriding config file
    ScriptSettings.update(vars(temp_args))

# ======================================================================================
# --- Custom DepthAI Pipeline Nodes ---
# ======================================================================================
class DetectionTargetingConfigurationNode(dai.node.HostNode):
    def __init__(self, config_template: dai.SpatialLocationCalculatorConfigData) -> None:
        super().__init__()
        self.depth_config_output = self.createOutput()
        self.crop_config_output = self.createOutput()
        self.config_template = config_template
    def build(self, detection_msg) -> "DetectionTargetingConfigurationNode":
        self.link_args(detection_msg); return self
    def process(self, dets_msg) -> None:
        depth_rois, recognition_crops, valid_detections = [], [], []
        frame_width, frame_height = dets_msg.transformation.getSize()
        for detection in dets_msg.detections:
            if detection.confidence < 0.75 or not len(detection.keypoints): continue
            new_detection, depth_roi, crop_config = self._calculate_target_and_configs(detection)
            if depth_roi and crop_config:
                valid_detections.append(new_detection)
                depth_rois.append(depth_roi.normalize(width=frame_width, height=frame_height))
                recognition_crops.append(crop_config)
        if valid_detections:
            new_detection_message = dets_msg.copy(); new_detection_message.detections = valid_detections
            spatial_config = dai.SpatialLocationCalculatorConfig()
            for roi in depth_rois: self.config_template.roi = roi; spatial_config.addROI(self.config_template)
            crop_config_message = dai.ImgDetections(); crop_config_message.detections = recognition_crops
            self.out.send(new_detection_message)
            self.crop_config_output.send(crop_config_message)
            self.depth_config_output.send(spatial_config)
    def _calculate_target_and_configs(self, detection: ImgDetectionExtended):
        l_shoulder, r_shoulder, l_hip, r_hip, nose, l_ear, r_ear = (detection.keypoints[i] for i in [5, 6, 11, 12, 0, 3, 4])
        center_shoulder_x, center_shoulder_y = (l_shoulder.x + r_shoulder.x) / 2, (l_shoulder.y + r_shoulder.y) / 2
        center_hip_x, center_hip_y = (l_hip.x + r_hip.x) / 2, (l_hip.y + r_hip.y) / 2
        target_x, target_y = center_shoulder_x + 1.2 * (center_hip_x - center_shoulder_x), center_shoulder_y + 1.2 * (center_hip_y - center_shoulder_y)
        if 0 < target_x < 1 and 0 < target_y < 1:
            target_keypoint = Keypoint(x=float(target_x), y=float(target_y)); detection.keypoints.append(target_keypoint)
            depth_roi = dai.Rect(dai.Point2f(target_x - 0.02, target_y + 0.02), dai.Point2f(target_x + 0.02, target_y - 0.02))
            crop_rect = detection.rotated_rect
            if ScriptSettings.MODE == 'FACE':
                crop_rect.center.x, crop_rect.center.y = nose.x, nose.y
                crop_rect.size.width, crop_rect.size.height = abs(r_ear.x - l_ear.x) * 1.25, abs(nose.y - center_shoulder_y) * 2.5
                crop_rect.angle = degrees(atan2(r_ear.y - l_ear.y, l_ear.x - r_ear.x))
            crop_config = dai.ImgDetection(); crop_config.xmin, crop_config.ymin, crop_config.xmax, crop_config.ymax = crop_rect.getOuterRect()
            return detection, depth_roi, crop_config
        return detection, None, None

class IdentificationNode(dai.node.HostNode):
    def __init__(self) -> None: super().__init__()
    def process(self, dets_msg) -> None: pass # Placeholder
class SerialSyncNode(dai.node.ThreadedHostNode):
    def __init__(self) -> None: super().__init__()
    def run(self) -> None: pass # Placeholder

# ======================================================================================
# --- Main Application Logic ---
# ======================================================================================
def main():
    setup_config_and_args()
    server = WebServerThread(app) if ScriptSettings.WEB_UI else None
    if server: server.start()

    try:
        print("Initializing device...")
        device = dai.Device()
        print("Device initialized.")

        with dai.Pipeline(device) as pipeline:
            print("Creating pipeline...")
            # --- FULL PIPELINE LOGIC HERE ---
            cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            # ... and so on ...

            if ScriptSettings.ENABLE_VISUALIZER:
                print("Starting visualizer...")
                visualizer = dai.RemoteConnection(address='0.0.0.0', httpPort=8082)
                # ... etc.

            print("Pipeline created. Starting main loop...")
            pipeline.start()
            while pipeline.isRunning():
                pipeline.processTasks()
                if select.select([sys.stdin], [], [], 0)[0]:
                    pipeline.stop(); break
    except RuntimeError as e:
        print(f"Error initializing DepthAI device: {e}")
        if server:
            print("DepthAI pipeline failed, but web UI is running. Press Ctrl+C to exit.")
            try:
                while True: time.sleep(1)
            except KeyboardInterrupt: pass
        else: sys.exit(1)
    except KeyboardInterrupt: pass
    finally:
        if server: server.shutdown()
        print("\nScript finished.")

if __name__ == '__main__':
    main()
