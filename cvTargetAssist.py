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
    """Global settings for the script's operation."""
    # --- General ---
    ENABLE_VISUALIZER = False  # Enable remote visualizer
    SERIAL_OUTPUT = True       # Enable sending data over serial port
    LOG_LEVEL = dai.LogLevel.WARN
    FPS = 10                   # Camera and pipeline frames per second

    # --- Identification Logic ---
    # TODO: These thresholds are critical to performance and may need tuning.
    #       They are currently hardcoded.
    # Suggested: Make these configurable via command-line arguments using argparse
    #            to allow for easier tuning without modifying the source code.
    CLOSE_MATCH_THRESHOLD = 0.16  # Distance for a confident match
    MEDIUM_MATCH_THRESHOLD = 0.22 # Distance for a potential match (requires validation)
    FAR_MATCH_THRESHOLD = 0.30    # Distance for a weak match (likely a new person)

    # --- Modes ---
    # TODO: The 'FACE' mode seems less developed than 'POSE' mode. The cropping logic
    #       for face recognition might need review and refinement.
    # Suggested: Add a command-line argument to switch between 'POSE' and 'FACE' modes.
    #            This would make the script more flexible for different use cases.
    MODE = 'POSE'  # 'POSE' for body-based targeting, 'FACE' for face recognition
    STATE = 'TEST' # A state identifier, used for naming the database and output folders.


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
    def __init__(self) -> None:
        """Initializes the node and its output queues."""
        super().__init__()
        self.depth_config_output = self.createOutput()
        self.crop_config_output = self.createOutput()

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

    def process(self, dets_msg: ImgDetectionsExtended) -> None:
        """
        Processes a message containing pose detections.

        For each valid detection, it calculates a target point and generates:
        1. A `SpatialLocationCalculatorConfig` for the depth node.
        2. An `ImgDetections` message to configure a cropping node.
        """
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
                # TODO: This uses a global `config` object (`initial_spatial_config`)
                #       which is not ideal. It makes the node less reusable and can
                #       lead to unexpected behavior if the global object is modified elsewhere.
                # Suggested: Replace `config.roi = roi` with `initial_spatial_config.roi = roi`
                #            and then `spatial_config.addROI(initial_spatial_config)`.
                #            Even better, pass the config object into this node's constructor
                #            to avoid global state entirely.
                config.roi = roi
                spatial_config.addROI(config)

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


class IdentificationNode(dai.node.HostNode):
    """
    A custom DepthAI node that identifies individuals using a recognition model
    and a persistent database.

    This node receives feature vectors (embeddings) from a recognition NN and
    compares them against a database of known individuals. It implements logic
    to decide whether a detection is a known person, a new person, or requires
    further validation. It then annotates the detection message with this
    identity information.
    """
    def __init__(self) -> None:
        """Initializes the node and the database connection."""
        super().__init__()
        db_path = f"./personDb-{ScriptSettings.MODE}-{ScriptSettings.STATE}.sqlite"
        self.db = sqlite3.connect(db_path, check_same_thread=False)

    def build(self, gather_data_msg, frame) -> "IdentificationNode":
        """
        Links the node's inputs.

        Args:
            gather_data_msg: The output from a `GatherData` node, containing synchronized
                             detections and recognition NN results.
            frame: The passthrough frame from the detection network, used for saving
                   images of new individuals.
        """
        self.link_args(gather_data_msg, frame)
        return self

    def onStart(self) -> None:
        """Initializes the database schema and loads the vector extension."""
        print("Initializing identification database...")
        # self.db.set_trace_callback(print) # Uncomment for debugging SQL queries
        self.db.enable_load_extension(True)
        sqlite_vec.load(self.db)
        self.db.enable_load_extension(False)

        # Create virtual table for fast vector similarity search
        self.db.execute("""
            CREATE VIRTUAL TABLE IF NOT EXISTS person_vector USING vec0(
                id INTEGER PRIMARY KEY,
                embedding FLOAT[512] DISTANCE_METRIC=cosine
            )
        """)
        # Create table for person metadata (e.g., name, validation status)
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS person (
               id INTEGER PRIMARY KEY,
               name TEXT,
               type TEXT  -- e.g., 'valid', 'invalid', 'guest'
            )
        """)
        # Create link table to associate multiple embeddings with a single person
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS person_vector_link (
                id INTEGER PRIMARY KEY,
                validated BOOLEAN,
                vector_id REFERENCES person_vector(id),
                person_id REFERENCES person(id)
            )
        """)
        print("Database initialized.")
        return super().onStart()

    def onStop(self) -> None:
        """Closes the database connection gracefully."""
        print("Closing identification database.")
        self.db.close()
        return super().onStop()

    def process(self, gather_data_msg, frame_msg) -> None:
        """
        Processes recognition results, compares them to the database, and updates
        the detection message with identity information.
        """
        dets_msg: ImgDetectionsExtended = gather_data_msg.reference_data
        rec_msg_list: list[dai.NNData] = gather_data_msg.gathered
        img_frame: dai.ImgFrame = frame_msg

        cv_frame = img_frame.getCvFrame()

        new_detection_message = dets_msg.copy()
        new_detection_message.detections = []

        for detection, rec_nn_data in zip(dets_msg.detections, rec_msg_list):
            if detection.confidence < 0.75 or not len(detection.keypoints):
                continue

            new_detection = detection.copy()
            embedding = rec_nn_data.getTensor("output", dequantize=True)
            should_emit, person_name, person_id = self._process_recognition(new_detection, embedding, cv_frame)

            # Encode the result into the label string for the next node to parse.
            # This is a simple way to pass structured data between host nodes.
            new_detection.label_name = f"{should_emit}-{person_name}-{person_id}"
            new_detection_message.detections.append(new_detection)

        self.out.send(new_detection_message)

    def _process_recognition(
        self, detection: ImgDetectionExtended, embedding: list[float], cv_frame
    ) -> Tuple[bool, str | None, str | None]:
        """
        Handles the core logic of matching an embedding against the database.

        This function implements a state machine to decide whether to:
        - Emit a "valid target" event.
        - Create a new person entry in the database.
        - Create a new vector link for an existing person.
        - Save an image of the person for later review/validation.

        Args:
            detection: The detection associated with the embedding.
            embedding: The feature vector from the recognition network.
            cv_frame: The full video frame, used for cropping and saving images.

        Returns:
            A tuple containing:
            - bool: Whether to emit a valid target event.
            - str: The name of the identified person.
            - str: The ID of the identified person.
        """
        # --- Decision Logic Summary ---
        #
        # | Match Type   | Link Status | Person Status | Action
        # |--------------|-------------|---------------|--------------------------------------------------
        # | Close        | Valid       | Valid         | Emit event.
        # | Medium       | Valid       | Valid         | Emit event, create new (validated) link.
        # | Medium       | *           | *             | Create new (unvalidated) link, save image.
        # | Far          | Valid       | *             | Create new (unvalidated) link, save image.
        # | Far          | Invalid     | *             | Create new person and link, save image.
        # | Miss         | -           | -             | Create new person and link, save image.
        #
        # Note: An event is ONLY emitted if both the link and the person are 'valid'.

        def get_match_type(distance: float) -> str:
            """Categorizes the embedding distance into 'close', 'medium', 'far', or 'miss'."""
            if not (0.0 <= distance <= 1.0):
                return "miss"
            if distance < ScriptSettings.CLOSE_MATCH_THRESHOLD:
                return "close"
            if distance < ScriptSettings.MEDIUM_MATCH_THRESHOLD:
                return "medium"
            if distance < ScriptSettings.FAR_MATCH_THRESHOLD:
                return "far"
            return "miss"

        # --- Initialize State Variables ---
        create_new_person = False
        create_new_link = False
        save_image_file = False
        emit_target_event = False

        person_id = None
        person_name = "unknown"
        distance = -1.0
        link_is_validated = False
        person_is_valid = False
        person_vector_link_id = None # Initialize to avoid UnboundLocalError
        vector_id = None

        # --- Query Database for Best Match ---
        with self.db as cur:
            res = cur.execute(
                """
                -- Find the single best match for the given embedding vector
                SELECT
                    pv.id as vector_id,
                    p.id as person_id,
                    p.name,
                    p.type,
                    pvl.validated,
                    distance -- This is a special column provided by sqlite-vec
                FROM person_vector pv
                JOIN person_vector_link pvl ON pvl.vector_id = pv.id
                JOIN person p ON pvl.person_id = p.id
                WHERE pv.embedding MATCH ? AND k = 1 -- k=1 returns the top 1 match
                ORDER BY distance
                """,
                [embedding],
            )
            row = res.fetchone()

        if row:
            # Unpack the results if a match was found
            (vector_id, person_id, person_name, person_type, link_is_validated, distance) = row
            person_is_valid = (person_type == 'valid')

        match_type = get_match_type(distance)

        # --- Determine Actions Based on Match Type and Status ---
        if match_type == 'close':
            emit_target_event = True
        elif match_type == 'medium':
            emit_target_event = True
            create_new_link = True
            save_image_file = not link_is_validated # Save if the matched link was not yet validated
        elif match_type == 'far':
            save_image_file = True
            if link_is_validated:
                create_new_link = True # Known person, but poor match. Add a new vector for review.
            else:
                create_new_person = True # Unvalidated link, poor match. Assume it's a new person.
        elif match_type == 'miss':
            save_image_file = True
            create_new_person = True

        if create_new_person:
            create_new_link = True # A new person always gets a new link.

        # Crucial final check: only emit a "valid target" event for a valid person
        # that was matched with a previously validated embedding link.
        if not (link_is_validated and person_is_valid):
            emit_target_event = False

        # --- Execute Database and File System Actions ---
        if create_new_person or create_new_link:
            person_name = 'Unknown'
            # A new link from a medium match to a valid person can be auto-validated.
            new_link_is_validated = (link_is_validated and match_type == 'medium')

            with self.db as cur:
                if create_new_person:
                    cur.execute("INSERT INTO person(name, type) VALUES (?, ?)", ['Unknown', 'invalid'])
                    person_id = cur.execute("SELECT last_insert_rowid()").fetchone()[0]

                if create_new_link:
                    cur.execute("INSERT INTO person_vector(embedding) VALUES (?)", [embedding])
                    vector_id = cur.execute("SELECT last_insert_rowid()").fetchone()[0]
                    cur.execute(
                        "INSERT INTO person_vector_link(vector_id, person_id, validated) VALUES (?, ?, ?)",
                        [vector_id, person_id, new_link_is_validated]
                    )
                    person_vector_link_id = cur.execute("SELECT last_insert_rowid()").fetchone()[0]
                cur.commit()

        if save_image_file:
            # TODO: The face cropping logic here is duplicated from the targeting node.
            #       This is inefficient and prone to bugs if one is updated and the other is not.
            # Suggested: Create a helper function or class method that takes keypoints and returns
            #            a RotatedRect for a face crop. Call this function from both
            #            `_calculate_target_and_configs` and here. Also, if MODE is 'POSE',
            #            this should save the body crop, not attempt to calculate a face crop.
            rect = detection.rotated_rect
            if ScriptSettings.MODE == 'FACE':
                l_shoulder = detection.keypoints[5]
                r_shoulder = detection.keypoints[6]
                nose = detection.keypoints[0]
                l_ear = detection.keypoints[3]
                r_ear = detection.keypoints[4]
                center_shoulder_y = (l_shoulder.y + r_shoulder.y) / 2

                rect = RotatedRect()
                rect.center.x = nose.x
                rect.center.y = nose.y
                rect.size.width = abs(r_ear.x - l_ear.x) * 1.25
                rect.size.height = abs(nose.y - center_shoulder_y) * 2.5
                rect.angle = degrees(atan2(r_ear.y - l_ear.y, l_ear.x - r_ear.x))

            path = f"TestFrames/{ScriptSettings.MODE}-{ScriptSettings.STATE}/"
            os.makedirs(path, exist_ok=True)

            min_x, min_y, max_x, max_y = rect.denormalize(
                width=cv_frame.shape[1], height=cv_frame.shape[0]
            ).getOuterRect()

            crop_image = cv_frame[int(min_y):int(max_y), int(min_x):int(max_x)]
            if crop_image.any():
                # Filename includes IDs for easier cross-referencing with the database.
                filename = f"{path}/{person_id}-{person_vector_link_id}-{vector_id}-{person_name}.png"
                cv2.imwrite(filename, crop_image)

        return (emit_target_event, person_name, person_id)


class SerialSyncNode(dai.node.ThreadedHostNode):
    """
    A custom node to synchronize detection/identification results with spatial data
    and send the final target information over a serial port.
    """
    def __init__(self) -> None:
        """Initializes the node, its inputs, and the serial port object."""
        super().__init__()
        self.detections_input = self.createInput()
        self.depth_input = self.createInput()
        self.serial_port = serial.Serial()

    def onStart(self) -> None:
        """
        Opens the serial port if enabled in the settings.
        It will retry a few times with a delay before exiting if the port fails to open.
        """
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
        """Closes the serial port if it is open."""
        if self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")
        return super().onStop()

    def run(self):
        """
        Main loop for the threaded node. It continuously tries to get synchronized
        messages from its input queues.
        """
        while self.isRunning():
            try:
                # Blocking get() to ensure we have both messages before processing
                detections_msg = self.detections_input.get()
                depth_msg = self.depth_input.get()
                self.process(detections_msg, depth_msg)
            except dai.MessageQueue.QueueException:
                # This exception is thrown when the queue is closed (e.g., pipeline stopping)
                break

    def process(self, detections_msg, spatial_data_msg) -> None:
        """
        Processes synchronized detection and depth messages, creates a `TargetMessage`,
        serializes it, and sends it over the serial port.
        """
        spatial_locations = spatial_data_msg.getSpatialLocations()
        if not ScriptSettings.SERIAL_OUTPUT:
            print("--- TARGET PACKET ---")

        for detection, spatial_data in zip(detections_msg.detections, spatial_locations):
            # Keypoint 17 is the calculated target point. If it's not present, skip.
            if len(detection.keypoints) < 18:
                continue

            # Extract coordinates and map them to the desired coordinate system.
            # DepthAI's coordinate system: X is right, Y is down, Z is forward.
            # Desired system: X is right, Y is forward, Z is up.
            x_coord = int(spatial_data.spatialCoordinates.x)
            y_coord = int(spatial_data.spatialCoordinates.z) # Map DepthAI 'z' to our 'y'
            z_coord = int(-1 * spatial_data.spatialCoordinates.y) # Map DepthAI 'y' to our 'z'

            # Parse identity info from the label string created by the IdentificationNode
            emit_str, name, id_str = detection.label_name.split('-')
            should_emit = (emit_str == 'True')
            target_id = int(id_str) if id_str != 'None' else -1

            # Create and serialize the message
            target_packet = TargetMessage(target_id, should_emit, x_coord, y_coord, z_coord).serialize()

            if ScriptSettings.SERIAL_OUTPUT:
                if self.serial_port.is_open:
                    self.serial_port.write(target_packet)
            else:
                # Print for debugging if serial is off
                print(f"\tSeq: {detections_msg.getSequenceNum()}, TS: {detections_msg.getTimestamp()}")
                print(f"\tID: {target_id}, Name: {name}, Emit: {should_emit}")
                print(f"\tCoords (x,y,z): ({x_coord}, {y_coord}, {z_coord})")
                print(f"\tPacket: {target_packet.hex()}")


# ======================================================================================
# --- Pipeline Construction ---
# ======================================================================================

# This script node is a workaround to dynamically generate ImageManip configs
# based on the output of the detection network. It acts as a bridge between
# the detection node and the ImageManip node for cropping.
# TODO: This `Script` node adds complexity and overhead. The logic it contains
#       (iterating detections and creating crop configs) is very similar to what's
#       in `DetectionTargetingConfigurationNode`.
# Suggested: It might be possible to merge this logic into the
#            `DetectionTargetingConfigurationNode` itself. The node could be modified
#            to output the `ImageManipConfig` directly, removing the need for this
#            intermediate script node and simplifying the pipeline graph.
image_manip_script_content = f"""
try:
    while True:
        frame = node.inputs['preview'].get()
        dets  = node.inputs['det_in'].tryGet()
        depth = node.inputs['depth_in'].tryGet()

        if not (frame and dets and depth):
            continue

        if len(dets.detections) == 0:
            continue

        # Passthrough the frame and depth config
        node.outputs['manip_img'].send(frame)
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
            # Reuse the input image for subsequent crops in the same message
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
    target_detection_node = pipeline.create(DetectionTargetingConfigurationNode).build(
        det_nn.out,
    )

    # --- Script node for dynamic cropping ---
    image_manip_script_node = pipeline.create(dai.node.Script)
    image_manip_script_node.setScript(image_manip_script_content)
    det_nn.passthrough.link(image_manip_script_node.inputs["preview"])
    target_detection_node.crop_config_output.link(image_manip_script_node.inputs["det_in"])
    target_detection_node.depth_config_output.link(image_manip_script_node.inputs['depth_in'])

    # --- Image Cropping for Recognition ---
    crop_node = pipeline.create(dai.node.ImageManip)
    crop_node.inputConfig.setWaitForMessage(True)
    crop_node.initialConfig.setOutputSize(recognition_model_width, recognition_model_height)
    image_manip_script_node.outputs["manip_cfg"].link(crop_node.inputConfig)
    image_manip_script_node.outputs["manip_img"].link(crop_node.inputImage)

    # --- Recognition NN ---
    rec_nn = pipeline.create(ParsingNeuralNetwork).build(
        crop_node.out, rec_nn_archive
    )
    rec_nn.setNNArchive(rec_nn_archive, numShaves=4)

    # --- Data Gathering and Identification ---
    # TODO: The GatherData node introduces latency by buffering messages to synchronize
    #       the recognition results with the original detection data.
    # Suggested: For improved performance, consider integrating the synchronization logic
    #            directly into the `IdentificationNode`. This would involve passing the
    #            detection messages directly to the `IdentificationNode` and managing
    #            a queue internally to match them with the recognition results.
    gather_data_node = pipeline.create(GatherData).build(camera_fps=ScriptSettings.FPS)
    rec_nn.out.link(gather_data_node.input_data)
    target_detection_node.out.link(gather_data_node.input_reference)

    id_node = pipeline.create(IdentificationNode).build(
        gather_data_msg=gather_data_node.out,
        frame=det_nn.passthrough,
    )

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

    # --- Final Sync and Serial Output ---
    sync_node = pipeline.create(SerialSyncNode)
    id_node.out.link(sync_node.detections_input)
    spatial_location_calculator.out.link(sync_node.depth_input)

    # --- Optional Visualizer ---
    if ScriptSettings.ENABLE_VISUALIZER:
        print("Starting visualizer...")
        visualizer = dai.RemoteConnection(address='0.0.0.0', httpPort=8082)
        visualizer.addTopic("Video", det_nn.passthrough, "images")
        visualizer.addTopic("Objects", id_node.out, "images")

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