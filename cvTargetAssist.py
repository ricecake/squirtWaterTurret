"""
This script provides a target acquisition and identification system using a Luxonis DepthAI camera.

It uses a multi-stage pipeline to perform the following tasks:
1.  **Person Detection and Pose Estimation:** A YOLOv8-nano model detects people in the
    video stream and estimates the location of 17 key body joints.
2.  **Target Point Calculation:** Based on the pose keypoints, a target point on the torso
    is calculated for each detected person. This is used for both aiming and for cropping
    a region of interest (ROI) for identification.
3.  **Person Identification:** A recognition model (ArcFace for faces or OSNet for bodies)
    is run on the cropped ROI. The resulting feature vector (embedding) is compared
    against a database of known individuals.
4.  **Database Management:** A SQLite database with the `sqlite-vec` extension stores
    embeddings of known persons. The script includes logic to add new individuals and
    associate new embeddings with existing ones based on match confidence.
5.  **Spatial Localization:** The stereo cameras are used to calculate the 3D (x, y, z)
    coordinates of the calculated target point.
6.  **Serial Communication:** The final target information (ID, validity, 3D coordinates)
    is serialized and sent over a serial port for use by an external device (e.g., a
    fire control system).

The pipeline is constructed using the DepthAI API, with custom nodes for handling the
complex logic of target configuration and person identification. The script can be
configured to run in different modes (e.g., 'POSE' vs. 'FACE') and states (e.g., 'TEST').
"""
import os
import sys
import select
import struct
import time
import argparse
from math import atan2, degrees
from typing import Tuple

import sqlite3
import sqlite_vec
import serial

import cv2
import depthai as dai

from depthai import RotatedRect
from depthai_nodes import ImgDetectionsExtended, ImgDetectionExtended
from depthai_nodes.node import ParsingNeuralNetwork, GatherData
from depthai_nodes.message.keypoints import Keypoint

# ======================================================================================
# --- Configuration ---
# ======================================================================================

class ScriptSettings:
    """
    Global settings for the script's operation.
    This class is a container for settings that will be populated by command-line
    arguments.
    """
    # These will be overwritten by parse_and_setup_settings()
    ENABLE_VISUALIZER: bool
    SERIAL_OUTPUT: bool
    LOG_LEVEL: dai.LogLevel
    FPS: int
    CLOSE_MATCH_THRESHOLD: float
    MEDIUM_MATCH_THRESHOLD: float
    FAR_MATCH_THRESHOLD: float
    MODE: str
    STATE: str
    HEIGHT: int

def parse_and_setup_settings():
    """
    Parses command-line arguments and populates the ScriptSettings class.
    """
    parser = argparse.ArgumentParser(description="Luxonis DepthAI Target Acquisition and Identification System")

    # General settings
    parser.add_argument(
        '-v', '--enable-visualizer',
        action='store_true',
        default=False,
        help='Enable remote visualizer (default: False).'
    )
    parser.add_argument(
        '--serial-output',
        action='store_true',
        default=False,
        help='Enable sending data over serial port (default: False).'
    )
    parser.add_argument(
        '--log-level',
        default='WARN',
        choices=['DEBUG', 'INFO', 'WARN', 'ERROR', 'CRITICAL'],
        help='Set the logging level (default: WARN).'
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=10,
        help='Camera and pipeline frames per second (default: 10).'
    )
    parser.add_argument(
        '--height',
        type=int,
        default=1000,
        help='Camera height above the ground in mm (default: 1000).'
    )

    # Identification logic settings
    parser.add_argument(
        '--close-match-threshold',
        type=float,
        default=0.16,
        help='Distance threshold for a confident match (default: 0.16).'
    )
    parser.add_argument(
        '--medium-match-threshold',
        type=float,
        default=0.22,
        help='Distance threshold for a potential match (default: 0.22).'
    )
    parser.add_argument(
        '--far-match-threshold',
        type=float,
        default=0.30,
        help='Distance threshold for a weak match (default: 0.30).'
    )

    # Mode settings
    parser.add_argument(
        '--mode',
        default='POSE',
        choices=['POSE', 'FACE'],
        help="Operating mode: 'POSE' for body targeting, 'FACE' for face recognition (default: POSE)."
    )
    parser.add_argument(
        '--state',
        default='TEST',
        help='A state identifier for database and output folders (default: TEST).'
    )

    args = parser.parse_args()

    # Populate the ScriptSettings class with parsed arguments
    ScriptSettings.ENABLE_VISUALIZER = args.enable_visualizer
    ScriptSettings.SERIAL_OUTPUT = args.serial_output
    ScriptSettings.LOG_LEVEL = getattr(dai.LogLevel, args.log_level.upper())
    ScriptSettings.FPS = args.fps
    ScriptSettings.CLOSE_MATCH_THRESHOLD = args.close_match_threshold
    ScriptSettings.MEDIUM_MATCH_THRESHOLD = args.medium_match_threshold
    ScriptSettings.FAR_MATCH_THRESHOLD = args.far_match_threshold
    ScriptSettings.MODE = args.mode
    ScriptSettings.STATE = args.state
    ScriptSettings.HEIGHT = args.height


# Parse arguments and configure settings when the script is loaded
parse_and_setup_settings()


# ======================================================================================
# --- Keypoint Definitions ---
# ======================================================================================

# In the default YOLOv8 pose model, there are 17 keypoints,
# each representing a different part of the human body.
# Here is the mapping of each index to its respective body joint:
#
# 0    Nose
# 1    Left Eye
# 2    Right Eye
# 3    Left Ear
# 4    Right Ear
# 5    Left Shoulder
# 6    Right Shoulder
# 7    Left Elbow
# 8    Right Elbow
# 9    Left Wrist
# 10   Right Wrist
# 11   Left Hip
# 12   Right Hip
# 13   Left Knee
# 14   Right Knee
# 15   Left Ankle
# 16   Right Ankle

# ======================================================================================
# --- Device and Model Initialization ---
# ======================================================================================

print("Initializing device...")
device = dai.Device()
device.setLogLevel(ScriptSettings.LOG_LEVEL)
device.setLogOutputLevel(ScriptSettings.LOG_LEVEL)
platform = device.getPlatform().name
print(f"Platform: {platform}")

frame_type = (
    dai.ImgFrame.Type.BGR888i if platform == "RVC4" else dai.ImgFrame.Type.BGR888p
)

# --- Detection Model (Pose Estimation) ---
print("Loading detection model...")
DET_MODEL_ZOO_PATH = "luxonis/yolov8-nano-pose-estimation:coco-512x288"
det_model_description = dai.NNModelDescription(DET_MODEL_ZOO_PATH, platform=platform)
det_model_nn_archive = dai.NNArchive(
    dai.getModelFromZoo(det_model_description)
)
pose_model_width = det_model_nn_archive.getInputWidth() or -1
pose_model_height = det_model_nn_archive.getInputHeight() or -1

# --- Recognition Model (Person/Face Identification) ---
print("Loading recognition model...")
if ScriptSettings.MODE == 'FACE':
    REC_MODEL_ZOO_PATH = "luxonis/arcface:lfw-112x112"
else:
    # Other potential models:
    # "luxonis/osnet:imagenet-128x256"
    # "luxonis/osnet:multi-source-domain-128x256"
    REC_MODEL_ZOO_PATH = "luxonis/osnet:market1501-128x256"

rec_model_description = dai.NNModelDescription(REC_MODEL_ZOO_PATH, platform=platform)
rec_nn_archive = dai.NNArchive(
    dai.getModelFromZoo(rec_model_description)
)
recognition_model_width = rec_nn_archive.getInputWidth() or -1
recognition_model_height = rec_nn_archive.getInputHeight() or -1


# ======================================================================================
# --- Serial Communication Protocol ---
# ======================================================================================

class SerialMessage:
    """
    Base class for creating and parsing serial messages with a simple protocol.

    The protocol is a fixed-size struct with a start marker (0xCAFE), a message code,
    the payload, and an end marker (0xFACE).
    """
    format: str | None = None
    code: int | None = None

    def _get_payload_data(self) -> tuple:
        """
        Returns a tuple of the data to be packed into the message payload.
        This method should be overridden by subclasses.
        """
        return tuple()

    @classmethod
    def parse(cls, buffer: bytes) -> "SerialMessage":
        """
        Parses a byte buffer into a message object.

        Args:
            buffer: The raw byte buffer to parse.

        Returns:
            An instance of the message class.

        Raises:
            ValueError: If the buffer is malformed (e.g., incorrect markers or code).
            NotImplementedError: If the subclass does not define a 'format'.
        """
        if cls.format is not None:
            try:
                fields = struct.unpack(cls.format, buffer)
                # Check for start marker, message code, and end marker
                if fields[0] != 0xCAFE or fields[-1] != 0xFACE or fields[1] != cls.code:
                    raise ValueError('Bad buffer: Invalid markers or message code.')
                # Return a new instance with the payload fields
                return cls(*fields[2:-1])
            except struct.error as e:
                raise ValueError(f'Bad buffer: Struct unpacking failed. {e}')
        raise NotImplementedError("Message format is not defined.")


    def serialize(self) -> bytes:
        """
        Serializes the message object into a byte buffer.

        Returns:
            The packed byte buffer ready for transmission.

        Raises:
            ValueError: If the message format or code is not defined.
        """
        if not (self.format and self.code is not None):
            raise ValueError('Cannot serialize message: format or code is not defined.')

        return struct.pack(
            self.format,
            0xCAFE,
            self.code,
            *self._get_payload_data(),
            0xFACE
        )

class TargetMessage(SerialMessage):
    """
    Message to transmit target information.

    Attributes:
        target_id (int): The unique ID of the person being targeted.
        is_valid (bool): Whether the target is confirmed and valid for engagement.
        x (int): The x-coordinate of the target in millimeters.
        y (int): The y-coordinate of the target in millimeters.
        z (int): The z-coordinate of the target in millimeters.
    """
    format = '<HBI?hhhH'  # <H=start, B=code, I=id, ?=valid, h=x,h=y,h=z, H=end>
    code = 0

    def __init__(self, target_id: int, is_valid: bool, x: int, y: int, z: int):
        self.target_id = target_id
        self.is_valid = is_valid
        self.x = x
        self.y = y
        self.z = z

    def _get_payload_data(self) -> tuple:
        """Returns the payload data for the TargetMessage."""
        return (self.target_id, self.is_valid, self.x, self.y, self.z)


# ======================================================================================
# --- Custom DepthAI Pipeline Nodes ---
# ======================================================================================

class DetectionTargetingConfigurationNode(dai.node.HostNode):
    """
    A custom DepthAI node that processes pose detections to generate configurations
    for spatial localization and person recognition.

    This node receives pose detections, calculates a central torso target point,
    and outputs configurations for two subsequent pipeline nodes:
    1.  `depth_config_output`: Sends `SpatialLocationCalculatorConfig` messages with
        an ROI centered on the target point.
    2.  `crop_config_output`: Sends `ImgDetections` messages with bounding boxes
        to be used by an `ImageManip` node for cropping the person for recognition.
    """
    def __init__(self, config_template: dai.SpatialLocationCalculatorConfigData) -> None:
        """
        Initializes the node and its output queues.

        Args:
            config_template: A `SpatialLocationCalculatorConfigData` object to use as a
                             template for generating ROIs.
        """
        super().__init__()
        self.depth_config_output = self.createOutput()
        self.crop_config_output = self.createOutput()
        self.config_template = config_template

    def build(self, detection_msg) -> "DetectionTargetingConfigurationNode":
        """
        Links the node's input to the output of a detection node.

        Args:
            detection_msg: The output queue from a detection neural network.

        Returns:
            The instance of this node for chaining.
        """
        self.link_args(detection_msg)
        return self

    def process(self, dets_msg) -> None:
        """
        Processes a message containing pose detections.

        For each valid detection, it calculates a target point and generates:
        1. A `SpatialLocationCalculatorConfig` for the depth node.
        2. An `ImgDetections` message to configure a cropping node.
        """

        assert isinstance(dets_msg, ImgDetectionsExtended)

        depth_rois = []
        recognition_crops = []
        valid_detections = []

        transformation = dets_msg.transformation
        frame_width, frame_height = transformation.getSize()

        for detection in dets_msg.detections:
            # Filter out low-confidence detections or those without keypoints
            if detection.confidence < 0.75 or not len(detection.keypoints):
                continue

            (new_detection, depth_roi, crop_config) = self._calculate_target_and_configs(
                detection, (frame_width, frame_height)
            )

            if depth_roi and crop_config:
                valid_detections.append(new_detection)
                # Normalize the ROI for the depth configuration
                depth_rois.append(depth_roi.normalize(width=frame_width, height=frame_height))
                recognition_crops.append(crop_config)

        # Only send configurations if there are valid targets
        if depth_rois and recognition_crops and valid_detections:
            # Create a new detection message with only the valid detections
            new_detection_message = dets_msg.copy()
            new_detection_message.detections = valid_detections

            # Configure the spatial location calculator with the new ROIs
            spatial_config = dai.SpatialLocationCalculatorConfig()
            for roi in depth_rois:
                # Use the config template passed during initialization
                self.config_template.roi = roi
                spatial_config.addROI(self.config_template)

            # Configure the image manipulator with the new crop rectangles
            crop_config_message = dai.ImgDetections()
            crop_config_message.detections = recognition_crops

            # Send the messages to their respective outputs
            self.out.send(new_detection_message)
            self.crop_config_output.send(crop_config_message)
            self.depth_config_output.send(spatial_config)

    def _calculate_target_and_configs(
        self, detection: ImgDetectionExtended, frame_size: tuple[int, int]
    ) -> Tuple[ImgDetectionExtended, dai.Rect | None, dai.ImgDetection | None]:
        """
        Calculates the target point and generates ROI/crop configurations.

        Args:
            detection: The pose detection to process.
            frame_size: The (width, height) of the frame.

        Returns:
            A tuple containing:
            - The detection with the new target keypoint added.
            - A `dai.Rect` for the depth ROI.
            - A `dai.ImgDetection` for the recognition crop.
            Returns (detection, None, None) if a target cannot be calculated.
        """
        new_detection = detection.copy()
        depth_roi = None
        crop_config = None

        # Extract keypoints needed for calculation
        l_shoulder = detection.keypoints[5]
        r_shoulder = detection.keypoints[6]
        l_hip = detection.keypoints[11]
        r_hip = detection.keypoints[12]
        nose = detection.keypoints[0]
        l_ear = detection.keypoints[3]
        r_ear = detection.keypoints[4]

        # TODO: This commented-out check seems important for preventing errors when
        #       keypoints are swapped or invalid (e.g., left shoulder is to the right
        #       of the right shoulder). This could happen with unusual poses or noisy detections.
        # Suggested: Enable this check to improve robustness.
        #            `if l_shoulder.x <= r_shoulder.x or l_hip.x <= r_shoulder.x:`
        #            `    return (new_detection, None, None)`

        # Calculate center of shoulders and hips
        center_shoulder_x = (l_shoulder.x + r_shoulder.x) / 2
        center_shoulder_y = (l_shoulder.y + r_shoulder.y) / 2
        center_hip_x = (l_hip.x + r_hip.x) / 2
        center_hip_y = (l_hip.y + r_hip.y) / 2

        # Project a point down from the shoulder center along the torso axis.
        # This creates the target point on the upper torso. The scale factor
        # moves the point further down from the shoulder center.
        scale_factor = 1.2
        delta_x = scale_factor * (center_hip_x - center_shoulder_x)
        delta_y = scale_factor * (center_hip_y - center_shoulder_y)

        target_x = (center_shoulder_x + delta_x)
        target_y = (center_shoulder_y + delta_y)

        # Ensure the calculated target point is within the frame boundaries (0.0 to 1.0)
        if 0 < target_x < 1 and 0 < target_y < 1:
            target_keypoint = Keypoint()
            target_keypoint.x = float(target_x)
            target_keypoint.y = float(target_y)
            new_detection.keypoints.append(target_keypoint) # Add as keypoint 17

            # Create a small, 4% wide ROI around the target point for depth calculation.
            depth_roi = dai.Rect(
                dai.Point2f(target_x - 0.02, target_y + 0.02),
                dai.Point2f(target_x + 0.02, target_y - 0.02),
            )

            # Determine the crop for the recognition model
            crop_rect = new_detection.rotated_rect
            if ScriptSettings.MODE == 'FACE' and recognition_model_width and recognition_model_height:
                # For face mode, create a crop around the face using keypoints
                crop_rect.center.x = nose.x
                crop_rect.center.y = nose.y
                crop_rect.size.width = abs(r_ear.x - l_ear.x) * 1.25
                crop_rect.size.height = abs(nose.y - center_shoulder_y) * 2.5
                crop_rect.angle = degrees(atan2(r_ear.y - l_ear.y, l_ear.x - r_ear.x))

            # Create an ImgDetection to represent the crop area for the ImageManip node
            crop_config = dai.ImgDetection()
            (crop_config.xmin, crop_config.ymin, crop_config.xmax, crop_config.ymax) = crop_rect.getOuterRect()

        return (new_detection, depth_roi, crop_config)


class HostProcessingNode(dai.node.ThreadedHostNode):
    """
    A consolidated host-side processing node that handles identification, database
    management, and serial communication.

    This node receives detection data, recognition embeddings, and spatial location data
    from the device. It synchronizes these streams, performs person identification against
    a SQLite database, and sends the final target information over a serial port.
    It also receives cropped images of detections to save for later review.
    """
    def __init__(self) -> None:
        """Initializes inputs, the serial port, and the database connection."""
        super().__init__()
        # Inputs for synchronized data streams from the device
        self.detections_input = self.createInput()
        self.recognition_input = self.createInput()
        self.depth_input = self.createInput()

        # Serial port for communicating with the motor controller
        self.serial_port = serial.Serial()

        # Database for person identification
        db_path = f"./personDb-{ScriptSettings.MODE}-{ScriptSettings.STATE}.sqlite"
        self.db = sqlite3.connect(db_path, check_same_thread=False)

    def onStart(self) -> None:
        """
        Initializes the database schema and opens the serial port.
        """
        # --- Initialize Database ---
        print("Initializing identification database...")
        # self.db.set_trace_callback(print) # Uncomment for debugging SQL queries
        self.db.enable_load_extension(True)
        sqlite_vec.load(self.db)
        self.db.enable_load_extension(False)
        self.db.execute("CREATE VIRTUAL TABLE IF NOT EXISTS person_vector USING vec0(embedding FLOAT[512])")
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS person (
               id INTEGER PRIMARY KEY, name TEXT, type TEXT
            )
        """)
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS person_vector_link (
                id INTEGER PRIMARY KEY, validated BOOLEAN,
                vector_id REFERENCES person_vector(id), person_id REFERENCES person(id)
            )
        """)
        print("Database initialized.")

        # --- Open Serial Port ---
        if ScriptSettings.SERIAL_OUTPUT:
            retries = 3
            for i in range(retries):
                try:
                    self.serial_port.port = '/dev/serial0'
                    self.serial_port.baudrate = 9600
                    self.serial_port.open()
                    print(f"Serial port {self.serial_port.port} opened successfully.")
                    return super().onStart()
                except serial.SerialException as e:
                    print(f"Warning: Could not open serial port: {e}.")
                    if i < retries - 1:
                        print(f"Retrying in 2 seconds... ({i + 1}/{retries})")
                        time.sleep(2)
            print("\nFATAL: Could not open serial port after multiple retries. Exiting.")
            sys.exit(1)

        return super().onStart()

    def onStop(self) -> None:
        """Closes the serial port and database connection."""
        if self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")
        print("Closing identification database.")
        self.db.close()
        return super().onStop()

    def run(self):
        """
        Main loop for the threaded node. It pulls messages from all input streams,
        buffers them, and processes them once a synchronized set is available.
        """
        # Get the queue for the cropped image stream from the device
        cropped_stream = self.pipeline.getOutputQueue("cropped_stream")

        # Internal buffers to hold messages, keyed by sequence number
        self.buffers = {
            "detections": {}, "recognition": {}, "depth": {}, "crop": {}
        }
        input_map = {
            "detections": self.detections_input,
            "recognition": self.recognition_input,
            "depth": self.depth_input,
            "crop": cropped_stream
        }

        while self.isRunning():
            try:
                # Check all inputs for new messages and add them to buffers
                for name, queue in input_map.items():
                    msg = queue.tryGet()
                    if msg:
                        seq = msg.getSequenceNum()
                        self.buffers[name][seq] = msg

                # Check for synchronized sequences and process them
                self._process_buffers()

            except dai.MessageQueue.QueueException:
                break # Queue has been closed, exit the loop
            time.sleep(0.001) # Prevent busy-waiting

    def _process_buffers(self):
        """
        Iterates through the primary buffer (detections) and looks for a complete
        set of synchronized messages in the other buffers.
        """
        processed_sequences = []
        for seq, detections_msg in self.buffers["detections"].items():
            # Check if this sequence number exists in all other buffers
            if all(seq in self.buffers[name] for name in ["recognition", "depth", "crop"]):
                # A complete, synchronized set is found.
                rec_msg = self.buffers["recognition"][seq]
                depth_msg = self.buffers["depth"][seq]
                crop_msg = self.buffers["crop"][seq]

                # Process the synchronized messages
                self.process(detections_msg, rec_msg, depth_msg, crop_msg)

                # Add the sequence number to the list of processed items
                processed_sequences.append(seq)

        # Clean up processed messages from all buffers
        for seq in processed_sequences:
            for buffer in self.buffers.values():
                if seq in buffer:
                    del buffer[seq]

    def process(self, detections_msg, rec_msg, spatial_data_msg, crop_msg) -> None:
        """
        Processes a synchronized set of data, performs identification, and sends
        the final target information over the serial port.
        """
        spatial_locations = spatial_data_msg.getSpatialLocations()
        # The recognition NN and cropped image streams send a list of messages
        # per detection, so we get the raw data list.
        rec_nn_list = rec_msg.getData()
        cropped_frames = crop_msg.getData()

        if not ScriptSettings.SERIAL_OUTPUT:
            print("--- TARGET PACKET ---")

        for i, detection in enumerate(detections_msg.detections):
            # Ensure all data is present for this specific detection
            if len(detection.keypoints) < 18 or i >= len(rec_nn_list) or i >= len(cropped_frames):
                continue

            embedding = rec_nn_list[i].getTensor("output", dequantize=True)
            cv_frame = cropped_frames[i].getCvFrame()

            # Perform person identification using the embedding and save image if needed
            should_emit, name, person_id = self._process_recognition(detection, embedding, cv_frame)

            # Get spatial coordinates for the target
            spatial_data = spatial_locations[i]
            x_coord = int(spatial_data.spatialCoordinates.x)
            y_coord = int(spatial_data.spatialCoordinates.z)
            z_coord = int(ScriptSettings.HEIGHT - spatial_data.spatialCoordinates.y)

            # Create and send the final target message
            target_packet = TargetMessage(person_id or -1, should_emit, x_coord, y_coord, z_coord).serialize()

            if ScriptSettings.SERIAL_OUTPUT:
                if self.serial_port.is_open:
                    self.serial_port.write(target_packet)
            else:
                # Print for debugging if serial is off
                print(f"\tSeq: {detections_msg.getSequenceNum()}, Name: {name}, Emit: {should_emit}")
                print(f"\tCoords (x,y,z): ({x_coord}, {y_coord}, {z_coord})")
                print(f"\tPacket: {target_packet.hex()}")

    def _process_recognition(
        self, detection: ImgDetectionExtended, embedding: list[float], cv_frame
    ) -> Tuple[bool, str | None, int | None]:
        """
        Handles the core logic of matching an embedding against the database.
        This is largely the same logic as the original IdentificationNode.
        """
        def get_match_type(distance: float) -> str:
            if not (0.0 <= distance <= 1.0): return "miss"
            if distance < ScriptSettings.CLOSE_MATCH_THRESHOLD: return "close"
            if distance < ScriptSettings.MEDIUM_MATCH_THRESHOLD: return "medium"
            if distance < ScriptSettings.FAR_MATCH_THRESHOLD: return "far"
            return "miss"

        create_new_person = create_new_link = save_image_file = emit_target_event = False
        person_id = None
        person_name = "unknown"
        distance = -1.0
        link_is_validated = person_is_valid = False
        vector_id = None

        with self.db as cur:
            res = cur.execute(
                "SELECT p.id, p.name, p.type, pvl.validated, pv.id, distance "
                "FROM person_vector pv "
                "JOIN person_vector_link pvl ON pvl.vector_id = pv.id "
                "JOIN person p ON pvl.person_id = p.id "
                "WHERE pv.embedding MATCH ? AND k = 1 ORDER BY distance",
                [embedding],
            ).fetchone()

        if res:
            person_id, person_name, person_type, link_is_validated, vector_id, distance = res
            person_is_valid = (person_type == 'valid')

        match_type = get_match_type(distance)

        if match_type == 'close':
            emit_target_event = True
        elif match_type == 'medium':
            emit_target_event = True
            create_new_link = True
            save_image_file = not link_is_validated
        elif match_type == 'far':
            save_image_file = True
            create_new_link = link_is_validated
            create_new_person = not link_is_validated
        elif match_type == 'miss':
            save_image_file = True
            create_new_person = True

        if create_new_person:
            create_new_link = True

        if not (link_is_validated and person_is_valid):
            emit_target_event = False

        if create_new_person or create_new_link:
            person_name = 'Unknown'
            new_link_is_validated = (link_is_validated and match_type == 'medium')
            with self.db as cur:
                if create_new_person:
                    person_id = cur.execute("INSERT INTO person(name, type) VALUES (?, ?)", ['Unknown', 'invalid']).lastrowid
                if create_new_link:
                    vector_id = cur.execute("INSERT INTO person_vector(embedding) VALUES (?)", [embedding]).lastrowid
                    cur.execute(
                        "INSERT INTO person_vector_link(vector_id, person_id, validated) VALUES (?, ?, ?)",
                        [vector_id, person_id, new_link_is_validated]
                    )

        if save_image_file and cv_frame.any():
            path = f"TestFrames/{ScriptSettings.MODE}-{ScriptSettings.STATE}/"
            os.makedirs(path, exist_ok=True)
            filename = f"{path}/{person_id}-{vector_id}-{person_name}.png"
            cv2.imwrite(filename, cv_frame)

        return (emit_target_event, person_name, person_id)


# ======================================================================================
# --- Pipeline Construction ---
# ======================================================================================

# This script node is responsible for generating ImageManip configurations for
# cropping based on detection data. It receives detection and depth configuration
# messages and outputs an `ImageManipConfig` message to the cropping node.
# The video frame itself is passed directly from the detection network to the
# cropping node on the device, avoiding a costly device-to-host round trip.
image_manip_script_content = f"""
try:
    while True:
        # The script now only receives detection and depth configurations.
        # It no longer needs to handle the video frame itself.
        dets  = node.inputs['det_in'].tryGet()
        depth = node.inputs['depth_in'].tryGet()

        if not (dets and depth):
            continue

        if len(dets.detections) == 0:
            continue

        # Passthrough the depth config. The frame for cropping will be
        # linked directly from the detection NN to the ImageManip node.
        node.outputs['depth_cfg'].send(depth)

        for index, det in enumerate(dets.detections):
            cfg = ImageManipConfig()
            # Create a crop based on the detection's bounding box from the previous node
            rect = RotatedRect()
            rect.center.x = (det.xmin + det.xmax) / 2
            rect.center.y = (det.ymin + det.ymax) / 2
            rect.size.width = det.xmax - det.xmin
            rect.size.height = det.ymax - det.ymin
            rect.angle = 0

            cfg.addCropRotatedRect(rect, True)
            cfg.setOutputSize({recognition_model_width}, {recognition_model_height}, ImageManipConfig.ResizeMode.CENTER_CROP)
            # Reuse the input image for subsequent crops in the same message.
            # This is crucial for performance, as it avoids re-sending the frame.
            cfg.setReusePreviousImage(index < len(dets.detections) - 1)
            node.outputs['manip_cfg'].send(cfg)

except Exception as e:
    node.warn(str(e))
"""

# Initial config for the SpatialLocationCalculator
# This will be overridden by the dynamic configs from the targeting node.
initial_spatial_config = dai.SpatialLocationCalculatorConfigData()
initial_spatial_config.depthThresholds.lowerThreshold = 1
initial_spatial_config.depthThresholds.upperThreshold = 10000
initial_spatial_config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN


with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    # --- Color Camera and Detection NN ---
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    color_camera_output = cam.requestOutput(
        size=(pose_model_width, pose_model_height), type=frame_type, fps=ScriptSettings.FPS
    )

    det_nn = pipeline.create(ParsingNeuralNetwork).build(
        color_camera_output, det_model_nn_archive
    )
    det_nn.setNNArchive(det_model_nn_archive, numShaves=4)
    det_nn.input.setBlocking(False)
    det_nn.input.setMaxSize(2)

    # --- Custom Targeting Node ---
    target_detection_node = DetectionTargetingConfigurationNode(initial_spatial_config).build(
        det_nn.out,
    )

    # --- Script node for dynamic cropping ---
    image_manip_script_node = pipeline.create(dai.node.Script)
    image_manip_script_node.setScript(image_manip_script_content)
    # The script no longer receives the full frame ('preview').
    target_detection_node.crop_config_output.link(image_manip_script_node.inputs["det_in"])
    target_detection_node.depth_config_output.link(image_manip_script_node.inputs['depth_in'])

    # --- Image Cropping for Recognition ---
    crop_node = pipeline.create(dai.node.ImageManip)
    crop_node.inputConfig.setWaitForMessage(True)
    crop_node.initialConfig.setOutputSize(recognition_model_width, recognition_model_height)
    image_manip_script_node.outputs["manip_cfg"].link(crop_node.inputConfig)
    # The full frame is now linked directly from the detection NN to the crop node,
    # keeping the image data on the device.
    det_nn.passthrough.link(crop_node.inputImage)

    # --- Stream Cropped Images to Host ---
    # This XLinkOut node sends the small, cropped images to the host computer.
    # This is far more efficient than sending the entire video frame.
    cropped_stream = pipeline.create(dai.node.XLinkOut)
    cropped_stream.setStreamName("cropped_stream")
    crop_node.out.link(cropped_stream.input)

    # --- Recognition NN ---
    # The same cropped output is also sent to the on-device recognition network.
    rec_nn = pipeline.create(ParsingNeuralNetwork).build(
        crop_node.out, rec_nn_archive
    )
    rec_nn.setNNArchive(rec_nn_archive, numShaves=4)

    # --- Stereo Cameras and Depth Calculation ---
    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    mono_left_out = mono_left.requestOutput((640, 400), fps=ScriptSettings.FPS)
    mono_right_out = mono_right.requestOutput((640, 400), fps=ScriptSettings.FPS)

    stereo = pipeline.create(dai.node.StereoDepth)
    mono_left_out.link(stereo.left)
    mono_right_out.link(stereo.right)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_ACCURACY)
    stereo.setDepthAlign(align=dai.StereoDepthConfig.AlgorithmControl.DepthAlign.CENTER)
    stereo.setRectification(True)

    # --- Spatial Location Calculator ---
    spatial_location_calculator = pipeline.create(dai.node.SpatialLocationCalculator)
    stereo.depth.link(spatial_location_calculator.inputDepth)
    image_manip_script_node.outputs["depth_cfg"].link(spatial_location_calculator.inputConfig)
    spatial_location_calculator.initialConfig.addROI(initial_spatial_config)
    spatial_location_calculator.inputConfig.setWaitForMessage(True)
    spatial_location_calculator.inputConfig.setBlocking(True)
    spatial_location_calculator.inputConfig.setMaxSize(2)
    spatial_location_calculator.inputDepth.setWaitForMessage(True)
    spatial_location_calculator.inputDepth.setBlocking(False)
    spatial_location_calculator.inputDepth.setMaxSize(2)

    # --- Host-Side Processing ---
    # The consolidated HostProcessingNode receives all necessary data streams from the device.
    host_processing_node = pipeline.create(HostProcessingNode)
    target_detection_node.out.link(host_processing_node.detections_input)
    rec_nn.out.link(host_processing_node.recognition_input)
    spatial_location_calculator.out.link(host_processing_node.depth_input)


    # --- Optional Visualizer ---
    if ScriptSettings.ENABLE_VISUALIZER:
        print("Starting visualizer...")
        visualizer = dai.RemoteConnection(address='0.0.0.0', httpPort=8082)
        visualizer.addTopic("Video", det_nn.passthrough, "images")
        # The "Objects" topic is removed because the new consolidated HostProcessingNode
        # does not output a displayable ImgDetections message. Its purpose is to process
        # data and send it via serial, not to visualize it directly in the pipeline.

    # ======================================================================================
    # --- Main Loop ---
    # ======================================================================================
    print("Pipeline created. Starting main loop...")
    pipeline.start()

    try:
        while pipeline.isRunning():
            pipeline.processTasks()

            # Check for keyboard input to stop the script
            if select.select([sys.stdin], [], [], 0)[0]:
                print("Stopping pipeline...")
                pipeline.stop()
                break
    except KeyboardInterrupt:
        print("Interrupted by user. Stopping pipeline...")
        pipeline.stop()

    print("Pipeline stopped.")