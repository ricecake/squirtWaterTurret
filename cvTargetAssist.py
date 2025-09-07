import os
import sys
import select
import struct

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

"""
In the default YOLO11 pose model, there are 17 keypoints,
each representing a different part of the human body.
Here is the mapping of each index to its respective body joint:

0    Nose
1    Left Eye
2    Right Eye
3    Left Ear
4    Right Ear
5    Left Shoulder
6    Right Shoulder
7    Left Elbow
8    Right Elbow
9    Left Wrist
10   Right Wrist
11   Left Hip
12   Right Hip
13   Left Knee
14   Right Knee
15   Left Ankle
16   Right Ankle
"""

class SETTINGS:
    enable_visualizer = True
    SERIAL_OUTPUT = False
    log_level = dai.LogLevel.WARN
    FPS = 10
    CLOSE_MATCH = 0.16
    MEDIUM_MATCH = 0.22
    FAR_MATCH = 0.30
    MODE = 'POSE'   # 'POSE' or 'FACE'
    STATE = 'TEST'


device = dai.Device()
device.setLogLevel(SETTINGS.log_level)
device.setLogOutputLevel(SETTINGS.log_level)
platform = device.getPlatform().name
print(f"Platform: {platform}")

frame_type = (
    dai.ImgFrame.Type.BGR888i if platform == "RVC4" else dai.ImgFrame.Type.BGR888p
)

# detection model
DET_MODEL = "luxonis/yolov8-nano-pose-estimation:coco-512x288"
det_model_description = dai.NNModelDescription(DET_MODEL, platform=platform)
det_model_nn_archive = dai.NNArchive(
    dai.getModelFromZoo(det_model_description)
)
poseModelWidth = det_model_nn_archive.getInputWidth() or -1
poseModelHeight = det_model_nn_archive.getInputHeight() or -1

# recognition model
REC_MODEL = "luxonis/arcface:lfw-112x112" if SETTINGS.MODE == 'FACE' else "luxonis/osnet:market1501-128x256"
# "luxonis/osnet:imagenet-128x256"
# "luxonis/osnet:market1501-128x256"
# "luxonis/osnet:multi-source-domain-128x256"
rec_model_description = dai.NNModelDescription(REC_MODEL, platform=platform)
rec_nn_archive = dai.NNArchive(
    dai.getModelFromZoo(rec_model_description)
)
faceModelWidth = rec_nn_archive.getInputWidth() or -1
faceModelHeight = rec_nn_archive.getInputHeight() or -1

config = dai.SpatialLocationCalculatorConfigData()
config.depthThresholds.lowerThreshold = 1
config.depthThresholds.upperThreshold = 10000
config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN

script_content = """
try:
    while True:
        frame = node.inputs['preview'].get()
        dets  = node.inputs['det_in'].tryGet()
        depth = node.inputs['depth_in'].tryGet()

        if not (frame and dets and depth):
            continue

        if len(dets.detections) == 0:
            continue

        node.outputs['manip_img'].send(frame)
        node.outputs['depth_cfg'].send(depth)

        for index, det in enumerate(dets.detections):
            cfg = ImageManipConfig()
            rect = RotatedRect()
            rect.center.x = (det.xmin + det.xmax) / 2
            rect.center.y = (det.ymin + det.ymax) / 2
            rect.size.width = det.xmax - det.xmin
            rect.size.height = det.ymax - det.ymin
            rect.size.width = rect.size.width
            rect.size.height = rect.size.height
            rect.angle = 0

            cfg.addCropRotatedRect(rect, True)
            cfg.setOutputSize({w}, {h}, ImageManipConfig.ResizeMode.CENTER_CROP)
            cfg.setReusePreviousImage(index < len(dets.detections) - 1)
            node.outputs['manip_cfg'].send(cfg)

except Exception as e:
    node.warn(str(e))
""".format(w=faceModelWidth, h=faceModelHeight)

class Message():
    format = None
    code = None

    def _data(self):
        return tuple()

    @classmethod
    def parse(cls, buf: bytes):
        if cls.format is not None:
            fields = struct.unpack(cls.format, buf)
            if fields[0] != 0xCAFE or fields[-1] != 0xFACE or fields[1] != cls.code:
                raise ValueError('Bad buffer')

            return cls(*fields[2:-1])

    def serialize(self):
        if not (self.format and self.code is not None):
            raise ValueError('Bad struct')

        return struct.pack(self.format, 0xCAFE, self.code, *(self._data()), 0xFACE)

class TargetMessage(Message):
    format = '<HBI?hhhH'
    code = 0

    def __init__(self, id, valid, x, y, z):
        self.id: int = id
        self.valid: bool = valid
        self.x: int = x
        self.y: int = y
        self.z: int = z

    def _data(self):
        return (self.id, self.valid, self.x, self.y, self.z)

class DetectionTargetingConfigurationNode(dai.node.HostNode):
    def __init__(self) -> None:
        super().__init__()
        self.depth_config_output = self.createOutput()
        self.crop_config_output = self.createOutput()

    def build(self, detection_msg) -> "DetectionTargetingConfigurationNode":
        self.link_args(detection_msg)
        return self

    def process(self, dets_msg) -> None:
        depthConfigs = []
        recognitionConfigs = []

        transformation = dets_msg.transformation
        w, h = transformation.getSize()

        detections = []
        for detection in dets_msg.detections:
            if detection.confidence < 0.75:
                continue

            if not len(detection.keypoints):
                continue

            (new_detection, depth_config, crop_config) = self._transform_detection(detection, (w, h))

            if depth_config and crop_config:
                detections.append(new_detection)
                depthConfigs.append(depth_config.normalize(width=w, height=h))
                recognitionConfigs.append(crop_config)

        if depthConfigs and recognitionConfigs and detections:
            new_detection_message = dets_msg.copy()
            new_detection_message.detections = detections
            cfg = dai.SpatialLocationCalculatorConfig()
            for roi in depthConfigs:
                config.roi = roi
                cfg.addROI(config)

            configDet = dai.ImgDetections()
            configDet.detections = recognitionConfigs

            for conf in recognitionConfigs:
                configDet.detections.append(conf)

            self.out.send(new_detection_message)
            self.crop_config_output.send(configDet)
            self.depth_config_output.send(cfg)

    def _transform_detection(self, detection: ImgDetectionExtended, frameSize: tuple[int, int]) -> Tuple[ImgDetectionExtended, dai.Rect | None, dai.ImgDetection | None]:
        new_detection = detection.copy()
        depth_config = None
        cropConfig = None

        lShoulder = detection.keypoints[5]
        rShoulder = detection.keypoints[6]
        lHip = detection.keypoints[11]
        rHip = detection.keypoints[12]
        nose = detection.keypoints[0]
        lEar = detection.keypoints[3]
        rEar = detection.keypoints[4]

        # if lShoulder.x <= rShoulder.x or lHip.x <= rShoulder.x:
        #     return (new_detection, depth_config, cropConfig)

        cSX = (lShoulder.x + rShoulder.x) / 2
        cSY = (lShoulder.y + rShoulder.y) / 2
        cHX = (lHip.x + rHip.x) / 2
        cHY = (lHip.y + rHip.y) / 2

        s = 1.2
        dX = s * (cHX - cSX)
        dY = s * (cHY - cSY)

        tX = (cSX + dX)
        tY = (cSY + dY)

        if tX and tY and tY > 0 and tY < 1:
            targetKeypoint = Keypoint()
            targetKeypoint.x = float(tX)
            targetKeypoint.y = float(tY)
            new_detection.keypoints.append(targetKeypoint)

            depth_config = dai.Rect(
                dai.Point2f(tX - 0.02, tY + 0.02),
                dai.Point2f(tX + 0.02, tY - 0.02),
            )

            rect = new_detection.rotated_rect
            if faceModelWidth and faceModelHeight and SETTINGS.MODE == 'FACE':
                rect.center.x = nose.x
                rect.center.y = nose.y
                rect.size.width = abs(rEar.x - lEar.x) * 1.25
                rect.size.height = abs(nose.y - cSY) * 2.5
                rect.angle = degrees(atan2(rEar.y - lEar.y, lEar.x - rEar.x))

            cropConfig = dai.ImgDetection()
            (cropConfig.xmin, cropConfig.ymin, cropConfig.xmax, cropConfig.ymax) = rect.getOuterRect()  # rect.denormalize(width=frameSize[0], height=frameSize[1]).getOuterRect()

        return (new_detection, depth_config, cropConfig)


class IdentificationNode(dai.node.HostNode):
    def __init__(self) -> None:
        super().__init__()
        self.db = sqlite3.connect("./personDb-{}-{}.sqlite".format(SETTINGS.MODE, SETTINGS.STATE), check_same_thread=False)
        # self.db = sqlite3.connect(":memory:", check_same_thread=False)

    def build(self, gather_data_msg, frame) -> "IdentificationNode":
        self.link_args(gather_data_msg, frame)
        return self

    def onStart(self) -> None:
        # self.db.set_trace_callback(print)
        self.db.enable_load_extension(True)
        sqlite_vec.load(self.db)
        self.db.enable_load_extension(False)
        self.db.execute("""
            CREATE VIRTUAL TABLE IF NOT EXISTS
                person_vector USING
                vec0(
                    id integer primary key,
                    embedding float[512] distance_metric=cosine
                )
        """)
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS
                person (
                   id integer primary key,
                   name text,
                   type text
                )
        """)
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS
                person_vector_link (
                    id integer primary key,
                    validated boolean,
                    vector_id references person_vector(id),
                    person_id references person(id)
                )
        """)

        return super().onStart()

    def onStop(self) -> None:
        self.db.close()
        return super().onStop()

    def process(self, gather_data_msg, frame) -> None:
        dets_msg: ImgDetectionsExtended = gather_data_msg.reference_data
        assert isinstance(dets_msg, ImgDetectionsExtended)

        rec_msg_list = gather_data_msg.gathered
        assert isinstance(rec_msg_list, list)
        assert all(isinstance(msg, dai.NNData) for msg in rec_msg_list)

        assert isinstance(frame, dai.ImgFrame)

        img = frame.getCvFrame()

        new_detection_message = dets_msg.copy()
        new_detection_message.detections = []
        for detection, rec in zip(dets_msg.detections, rec_msg_list):
            if detection.confidence < 0.75:
                continue

            if not len(detection.keypoints):
                continue
            new_detection = detection.copy()
            emit, label, id = self._process_recognition(new_detection, rec, img)
            new_detection.label_name = '-'.join([str(emit), str(label), str(id)])
            new_detection_message.detections.append(new_detection)

        self.out.send(new_detection_message)

    def _process_recognition(self, detection, rec, img) -> Tuple[bool, str | None, str | None]:
        """
                        |Valid Link/Person          | Invalid Link/Valid Person   | Valid Link/Invalid Person | Invalid Link/Person
        ---------------------------------------------------------------------------------------------------------------------------------------
        | Close match   | Emit                      | No Emit                     | No Emit                   | No Emit
        | Medium match  | Emit/New link             | No Emit/New Link/Save Img   | No Emit/New link          | No Emit/New Link/Save Img
        | Far match     | No Emit/New Link/Save Img | No Emit/New Person/Save Img | No Emit/New Link/Save Img | No Emit/New Person/Save Img

        Miss: no emit, new person, save image.
        """
        def match_distance(distance):
            defined_segments = [
                (0.0, SETTINGS.CLOSE_MATCH, "close"),
                (SETTINGS.CLOSE_MATCH, SETTINGS.MEDIUM_MATCH, "medium"),
                (SETTINGS.MEDIUM_MATCH, SETTINGS.FAR_MATCH, "far"),
            ]
            if defined_segments[-1][1] <= distance <= defined_segments[0][0]:
                return "miss"

            for start, end, name in defined_segments:
                if start <= distance < end:
                    return name
            return "miss"

        create_person = False
        create_link = False
        save_file = False
        emit_event = False
        person_id = None
        person_vector_link_id = None
        vector_id = None
        person_name = "unknown"
        distance = -1
        match_type = 'miss'
        link_validated = False
        person_type = 'invalid'
        person_valid = False

        embedding = rec.getTensor("output", dequantize=True)

        with self.db as cur:
            res = cur.execute(
                """
                SELECT
                    pv.id,
                    p.id,
                    p.name,
                    p.type,
                    pvl.validated,
                    distance
                FROM person_vector pv
                join person_vector_link pvl on pvl.vector_id = pv.id
                join person p on pvl.person_id = p.id
                WHERE pv.embedding MATCH ?
                and k = 1
                ORDER BY distance
                """,
                [embedding],
            )

            rows = res.fetchone()

            if rows:
                (vector_id, person_id, person_name, person_type, link_validated, distance) = rows
                match_type = match_distance(distance)

        person_valid = person_type == 'valid'

        # print("(match_type: {}, vector_id: {} person_id: {} person_name: {} person_type: {} link_validated: {} distance: {})".format(match_type, vector_id, person_id, person_name, person_type, link_validated, distance))

        if match_type == 'close':
            emit_event = True
        elif match_type == 'medium':
            emit_event = True
            create_link = True
            save_file = not link_validated
        elif match_type == 'far':
            save_file = True
            create_link = link_validated
            create_person = not link_validated
        elif match_type == 'miss':
            save_file = True
            create_person = True

        if create_person:
            create_link = True

        if not (link_validated and person_valid):
            emit_event = False

        # print("[[person_name: {} person_type: {} link_validated: {} emit_event: {} create_link: {} create_person: {} save_file: {})".format(person_name, person_type, link_validated, emit_event, create_link, create_person, save_file))
        if create_person or create_link:
            person_name = 'Unknown'
            person_type = 'invalid'
            link_validated = link_validated and match_type == 'medium'
            with self.db as cur:
                if create_person:
                    cur.execute("insert into person(name, type) values (?, ?)", [person_name, person_type])
                    res = cur.execute(("select last_insert_rowid()"))
                    ret = res.fetchone()
                    person_id = ret[0]

                if create_link:
                    cur.execute("insert into person_vector(embedding) values (?)", [embedding])
                    res = cur.execute(("select last_insert_rowid()"))
                    ret = res.fetchone()
                    vector_id = ret[0]
                    cur.execute("insert into person_vector_link(vector_id, person_id, validated) values (?, ?, ?)", [vector_id, person_id, link_validated])
                    res = cur.execute(("select last_insert_rowid()"))
                    ret = res.fetchone()
                    person_vector_link_id = ret[0]
                cur.commit()

        if save_file:
            rect = detection.rotated_rect
            if SETTINGS.MODE == 'FACE':
                lShoulder = detection.keypoints[5]
                rShoulder = detection.keypoints[6]
                nose = detection.keypoints[0]
                lEar = detection.keypoints[3]
                rEar = detection.keypoints[4]

                cSY = (lShoulder.y + rShoulder.y) / 2

                rect = RotatedRect()
                rect.center.x = nose.x
                rect.center.y = nose.y
                rect.size.width = abs(rEar.x - lEar.x) * 1.25
                rect.size.height = abs(nose.y - cSY) * 2.5
                rect.angle = degrees(atan2(rEar.y - lEar.y, lEar.x - rEar.x))

            path = "TestFrames/{}-{}/".format(SETTINGS.MODE, SETTINGS.STATE)
            if not os.path.exists(path):
                os.makedirs(path)
            minX, minY, maxX, maxY = rect.denormalize(width=img.shape[1], height=img.shape[0]).getOuterRect()
            part = img[int(minY):int(maxY), int(minX):int(maxX)]
            if part.any():
                cv2.imwrite("{}/{}-{}-{}-{}.png".format(path, person_id, person_vector_link_id, vector_id, person_name), part)

        return (emit_event, person_name, person_id)

class SimpleSync(dai.node.ThreadedHostNode):
    def __init__(self) -> None:
        super().__init__()
        self.detections = self.createInput()
        self.depth = self.createInput()
        self.serial = serial.Serial()

    def onStart(self) -> None:
        if SETTINGS.SERIAL_OUTPUT:
            self.serial.port = '/dev/serial0'
            self.serial.baudrate = 115200
            self.serial.open()
        return super().onStart()

    def onStop(self) -> None:
        if self.serial.is_open:
            self.serial.close()
        return super().onStop()

    def run(self):
        while self.isRunning():
            try:
                detections = self.detections.get()
                depth = self.depth.get()
                self.process(detections, depth)
            except dai.MessageQueue.QueueException:
                break

    def process(self, detections, spatialOutput) -> None:
        spatialData = [spatialData.spatialCoordinates for spatialData in spatialOutput.getSpatialLocations()]
        if spatialData and not SETTINGS.SERIAL_OUTPUT:
            print("PACKET:")
        for detection, depthData in zip(detections.detections, spatialData):
            if len(detection.keypoints) < 18:
                continue
            xCoord = int(depthData.x)
            yCoord = int(depthData.z)
            zCoord = int(-1 * depthData.y)

            emit, name, id = detection.label_name.split('-')
            targetPacket = TargetMessage(int(id), bool(emit), xCoord, yCoord, zCoord).serialize()

            if SETTINGS.SERIAL_OUTPUT:
                self.serial.write(targetPacket)
            else:
                print("\t", (detections.getSequenceNum(), detections.getTimestamp(), name, int(id), bool(emit), xCoord, yCoord, zCoord))
                print("\t", targetPacket.hex())


with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    colorCameraOutput = cam.requestOutput(
        size=(poseModelWidth, poseModelHeight), type=frame_type, fps=SETTINGS.FPS
    )

    det_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
        colorCameraOutput, det_model_nn_archive
    )
    det_nn.setNNArchive(det_model_nn_archive, numShaves=4)
    det_nn.input.setBlocking(False)
    det_nn.input.setMaxSize(2)

    target_detection_node = pipeline.create(DetectionTargetingConfigurationNode).build(
        det_nn.out,
    )

    script_node = pipeline.create(dai.node.Script)
    script_node.setScript(script_content)
    det_nn.passthrough.link(script_node.inputs["preview"])
    target_detection_node.crop_config_output.link(script_node.inputs["det_in"])
    target_detection_node.depth_config_output.link(script_node.inputs['depth_in'])

    crop_node = pipeline.create(dai.node.ImageManip)
    crop_node.inputConfig.setWaitForMessage(True)
    crop_node.initialConfig.setOutputSize(
        faceModelWidth, faceModelHeight,
    )
    script_node.outputs["manip_cfg"].link(crop_node.inputConfig)
    script_node.outputs["manip_img"].link(crop_node.inputImage)

    rec_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
        crop_node.out, rec_nn_archive
    )
    rec_nn.setNNArchive(rec_nn_archive, numShaves=4)

    gather_data_node = pipeline.create(GatherData).build(camera_fps=SETTINGS.FPS)  # Should be able to find away to bake this into the IdentNode
    rec_nn.out.link(gather_data_node.input_data)
    target_detection_node.out.link(gather_data_node.input_reference)

    id_node = pipeline.create(IdentificationNode).build(
        gather_data_msg=gather_data_node.out,
        frame=det_nn.passthrough,
    )

    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    monoLeftOut = monoLeft.requestOutput((640, 400), fps=SETTINGS.FPS)
    monoRightOut = monoRight.requestOutput((640, 400), fps=SETTINGS.FPS)
    # monoLeftOut = monoLeft.requestFullResolutionOutput(fps=SETTINGS.FPS)
    # monoRightOut = monoRight.requestFullResolutionOutput(fps=SETTINGS.FPS)

    stereo = pipeline.create(dai.node.StereoDepth)
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_ACCURACY)
    stereo.setDepthAlign(align=dai.StereoDepthConfig.AlgorithmControl.DepthAlign.CENTER)
    stereo.setRectification(True)

    spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)
    stereo.depth.link(spatialLocationCalculator.inputDepth)
    script_node.outputs["depth_cfg"].link(spatialLocationCalculator.inputConfig)
    spatialLocationCalculator.initialConfig.addROI(config)
    spatialLocationCalculator.inputConfig.setWaitForMessage(True)
    spatialLocationCalculator.inputConfig.setBlocking(True)
    spatialLocationCalculator.inputConfig.setMaxSize(2)
    spatialLocationCalculator.inputDepth.setWaitForMessage(True)
    spatialLocationCalculator.inputDepth.setBlocking(False)
    spatialLocationCalculator.inputDepth.setMaxSize(2)

    sync_node = pipeline.create(SimpleSync)
    id_node.out.link(sync_node.detections)
    spatialLocationCalculator.out.link(sync_node.depth)

    if SETTINGS.enable_visualizer:
        visualizer = dai.RemoteConnection(
            address='0.0.0.0',
            httpPort=8082
        )
        visualizer.addTopic("Video", det_nn.passthrough, "images")
        visualizer.addTopic("Objects", id_node.out, "images")

    print("Pipeline created.")

    pipeline.start()

    while pipeline.isRunning():
        pipeline.processTasks()

        if select.select([sys.stdin], [], [], 0)[0]:
            print("Stopping pipeline...")
            pipeline.stop()
            break
