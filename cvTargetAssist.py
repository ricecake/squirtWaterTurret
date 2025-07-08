import sys
import select
import struct
import depthai as dai

from depthai import SpatialLocationCalculatorData
from depthai_nodes.message import ImgDetectionsExtended
from depthai_nodes.node import ParsingNeuralNetwork

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
    format = '<HBI?HHHH'
    code = 0

    def __init__(self, id, valid, x, y, z):
        self.id = id
        self.valid = valid
        self.x = x
        self.y = y
        self. z = z

    def _data(self):
        return (self.id, self.valid, self.x, self.y, self.z)


# by = TargetMessage(1, True, 2, 3, 0xFFFF).serialize()
# print(by.hex())
# msg = TargetMessage.parse(by)

# print(msg.id)
# print(msg.valid)
# print(msg.x)
# print(msg.y)
# print(msg.z)

# exit()

device = dai.Device()
platform = device.getPlatformAsString()
calibdata = device.readCalibration()

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    # model
    model_description = dai.NNModelDescription("luxonis/yolov8-nano-pose-estimation:coco-512x288")
    model_description.platform = platform
    nn_archive = dai.NNArchive(
        dai.getModelFromZoo(
            model_description,
        )
    )

    # media/camera input
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)
    spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)
    nn_with_parser = pipeline.create(ParsingNeuralNetwork).build(
        cam, nn_archive
    )

    # Linking
    monoLeftOut = monoLeft.requestOutput((640, 400))
    monoRightOut = monoRight.requestOutput((640, 400))
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)
    stereo.depth.link(spatialLocationCalculator.inputDepth)

    config = dai.SpatialLocationCalculatorConfigData()
    config.depthThresholds.lowerThreshold = 1
    config.depthThresholds.upperThreshold = 10000
    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN
    config.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))

    # spatialLocationCalculator.inputConfig.setWaitForMessage(False)
    spatialLocationCalculator.inputConfig.setWaitForMessage(True)
    spatialLocationCalculator.initialConfig.addROI(config)

    stereo.setLeftRightCheck(True)
    stereo.initialConfig.setConfidenceThreshold(50)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_ACCURACY)

    inputConfigQueue = spatialLocationCalculator.inputConfig.createInputQueue()

    xoutSpatialQueue = spatialLocationCalculator.out.createOutputQueue(maxSize=1)
    outputs = nn_with_parser.out.createOutputQueue()

    # Make a map with ROI data that maps it to data about the detection to keep things synced up
    # numDets = 0

    def outputCallback(msg: ImgDetectionsExtended):
        roiSet = False
        cfg = dai.SpatialLocationCalculatorConfig()
        transformation = msg.transformation
        w, h = transformation.getSize()
        for detection in msg.detections:
            if detection.confidence < 0.8:
                continue

            rotated_rect = detection.rotated_rect
            rotated_rect = rotated_rect.denormalize(w, h)
            points = rotated_rect.getPoints()
            points = [dai.Point2f(point.x, point.y) for point in points]

            if len(detection.keypoints):
                lShoulder = detection.keypoints[5]
                rShoulder = detection.keypoints[6]
                lHip = detection.keypoints[11]
                rHip = detection.keypoints[12]

                if lShoulder.x <= rShoulder.x or lHip.x <= rShoulder.x:
                    # print("No shooting in back")
                    continue

                cSX = (lShoulder.x + rShoulder.x) / 2
                cSY = (lShoulder.y + rShoulder.y) / 2
                cHX = (lHip.x + rHip.x) / 2
                cHY = (lHip.y + rHip.y) / 2

                s = 1.2
                dX = s * (cHX - cSX)
                dY = s * (cHY - cSY)

                tX = (cSX + dX)
                tY = (cSY + dY)

                if not (tX and tY):
                    continue

                roi = dai.Rect(
                    dai.Point2f(tX - 0.02, tY + 0.02),
                    dai.Point2f(tX + 0.02, tY - 0.02),
                )

                config.roi = roi.normalize(width=w, height=h)
                cfg.addROI(config)
                roiSet = True
        if roiSet:
            inputConfigQueue.send(cfg)

    def spatialCallback(spatialData: SpatialLocationCalculatorData):
        # if numDets == 0:
        #     return

        for depthData in spatialData.getSpatialLocations():
            print("X: {}, Y: {}, Z: {}".format(
                int(depthData.spatialCoordinates.x),
                int(depthData.spatialCoordinates.z),
                int(-1 * depthData.spatialCoordinates.y))
            )

    outputs.addCallback(outputCallback)
    xoutSpatialQueue.addCallback(spatialCallback)

    pipeline.start()

    print("Pipeline created.")

    while pipeline.isRunning():
        pipeline.processTasks()

        if select.select([sys.stdin], [], [], 0)[0]:
            print("Stopping pipeline...")
            pipeline.stop()
            break
