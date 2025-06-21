import depthai as dai

from depthai_nodes.message import ImgDetectionsExtended
from depthai_nodes.node import (
    ParsingNeuralNetwork,
    ImgFrameOverlay,
    ApplyColormap,
    DepthMerger,
    ImgDetectionsBridge
)

from objprint import op
import cv2
import numpy as np

color = (255, 255, 255)

device = dai.Device()
platform = device.getPlatformAsString()
print(f"Platform: {platform}")
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
    cam = pipeline.create(dai.node.Camera).build()
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
    config.depthThresholds.lowerThreshold = 10
    config.depthThresholds.upperThreshold = 10000
    calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN
    config.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))

    spatialLocationCalculator.inputConfig.setWaitForMessage(False)
    spatialLocationCalculator.initialConfig.addROI(config)

    stereo.setLeftRightCheck(True)
    stereo.initialConfig.setConfidenceThreshold(50)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_ACCURACY)

    xoutSpatialQueue = spatialLocationCalculator.out.createOutputQueue()
    outputDepthQueue = spatialLocationCalculator.passthroughDepth.createOutputQueue()
    inputConfigQueue = spatialLocationCalculator.inputConfig.createInputQueue()

    outputs = nn_with_parser.out.createOutputQueue()
    # colorImage = cam.requestOutput((640,480)).createOutputQueue()
    colorImage = nn_with_parser.passthrough.createOutputQueue()

    pipeline.start()
    print("Pipeline created.")

    while pipeline.isRunning():
        # outputDepthIMage: dai.ImgFrame = outputDepthQueue.get()
        # frameDepth = outputDepthIMage.getFrame()

        # depthFrameColor = cv2.normalize(frameDepth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        # depthFrameColor = cv2.equalizeHist(depthFrameColor)
        # depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_MAGMA)

        outputDepthIMage: dai.ImgFrame = colorImage.get()
        frameDepth = outputDepthIMage.getCvFrame()
        depthFrameColor = frameDepth

        roiSet = False
        msg: ImgDetectionsExtended = outputs.get()
        if len(msg.detections):
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

                    for p in points:
                        cv2.circle(depthFrameColor, (int(p.x), int(p.y)), 2, (0, 255, 0), -1)

                    for p in [lShoulder, rShoulder, lHip, rHip]:
                        cv2.circle(depthFrameColor, (int(p.x*w), int(p.y*h)), 5, (255, 0, 0), -1)

                    cSX = (lShoulder.x+rShoulder.x)/2
                    cSY = (lShoulder.y+rShoulder.y)/2
                    cHX = (lHip.x+rHip.x)/2
                    cHY = (lHip.y+rHip.y)/2

                    s = 1.2
                    dX = s*(cHX - cSX)
                    dY = s*(cHY - cSY)

                    tX = (cSX+dX)
                    tY = (cSY+dY)

                    cv2.circle(depthFrameColor, (int(tX*w), int(tY*h)), 5, (0, 0, 255), -1)


                    roi = dai.Rect(
                        dai.Point2f(tX-0.05, tY+0.05),
                        dai.Point2f(tX+0.05, tY-0.05),
                    )

                    config.roi = roi.normalize(width=w, height=h)
                    cfg.addROI(config)
                    roiSet = True
            if roiSet:
                inputConfigQueue.send(cfg)

        spatialData = xoutSpatialQueue.get().getSpatialLocations()
        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            fontType = cv2.FONT_HERSHEY_TRIPLEX
            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
            cv2.putText(depthFrameColor, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, color)
            cv2.putText(depthFrameColor, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, color)
            cv2.putText(depthFrameColor, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, color)
        # Show the frame
        cv2.imshow("depth", cv2.resize(depthFrameColor, (depthFrameColor.shape[1]*2, depthFrameColor.shape[0]*2)))

        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break

