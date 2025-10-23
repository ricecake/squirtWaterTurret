# CV System: The "Brains"

The computer vision component, `cvTargetAssist.py`, is a sophisticated Python application that serves as the "brains" of the sentry turret. It uses a Luxonis OAK-D camera and its powerful `depthai` library to perform real-time target detection, identification, and localization.

## Core Responsibilities

1.  **Detect and Track:** Identify human figures in the camera's field of view.
2.  **Identify:** Determine if a detected person is a known individual.
3.  **Localize:** Calculate the precise 3D coordinates of the target.
4.  **Communicate:** Send the final target information to the firmware for action.

## The DepthAI Pipeline

The script constructs a complex, multi-stage processing pipeline that runs largely on the OAK-D camera itself. This on-device processing is crucial for achieving real-time performance.

Here is a simplified flow of the pipeline:

```
                               +-----------------------------+
                               |    IdentificationNode       |
                               | (Host: Match to DB)         |
                               +-----------------------------+
                                      ^
                                      | (Embeddings)
+----------------+      +------------------+      +-----------------------------+      +----------------+
|  Color Camera  |----->|  Pose Detection  |----->| DetectionTargetingConfigNode|----->|  Recognition   |
| (CAM_A)        |      | (YOLOv8 NN)      |      | (Host: Calc Target Points)  |      | (OSNet/ArcFace)|
+----------------+      +------------------+      +-----------------------------+      +----------------+
      |                                                 |                |                     ^
      | (Passthrough Frame)                             |                | (Crop Config)       | (Cropped Image)
      |                                                 |                |                     |
      |                                                 |                v                     |
      |                                                 |      +----------------+      +-------------+
      |                                                 |      | ImageManip     |----->| Script Node |
      |                                                 |      | (On-Device Crop)      | (Dynamic Crop) |
      |                                                 |      +----------------+      +-------------+
      |                                                 |
      |                                                 | (Depth ROI Config)
      |                                                 v
+----------------+      +----------------+      +-----------------------------+
| Stereo Cameras |----->|  Stereo Depth  |----->|  Spatial Location Calc      |
| (CAM_B, CAM_C) |      | (On-Device)    |      | (On-Device 3D Calculation)  |
+----------------+      +----------------+      +-----------------------------+
      |                                                                 |
      | (Final Detections w/ ID)                                        | (3D Coordinates)
      v                                                                 v
+-------------------------------------------------------------------------+
|                              SerialSyncNode                             |
|                  (Host: Synchronize and Send to Firmware)               |
+-------------------------------------------------------------------------+

```

## Key Pipeline Stages and Custom Nodes

### 1. Pose Detection and Targeting (`DetectionTargetingConfigurationNode`)

-   **Input:** A raw video stream from the color camera.
-   **Process:** A `YOLOv8-nano-pose` neural network runs on the device, detecting people and locating 17 body keypoints.
-   **Custom Logic:** The `DetectionTargetingConfigurationNode` is a custom *Host Node* that receives these detections. For each person, it calculates a stable aiming point on the torso using the shoulder and hip keypoints.
-   **Output:** This node is critical for orchestrating the rest of the pipeline. It generates two sets of dynamic configurations:
    1.  **Spatial ROI Config:** A small Region of Interest (ROI) around the torso aiming point, sent to the `SpatialLocationCalculator`.
    2.  **Crop Config:** A bounding box around the person (or just their face, in `FACE` mode), sent to an `ImageManip` node.

### 2. Person Identification (`IdentificationNode`)

-   **Input:** A cropped image of the person from the `ImageManip` node.
-   **Process:** A recognition neural network (`OSNet` for bodies or `ArcFace` for faces) runs on the device, converting the cropped image into a feature vector, also known as an "embedding."
-   **Custom Logic:** The `IdentificationNode` is another custom *Host Node* that receives these embeddings.
-   **Database Matching:** It uses a local SQLite database supercharged with the `sqlite-vec` extension for high-speed vector similarity search. It compares the new embedding against a database of known individuals.
-   **Decision Engine:** This node contains a sophisticated decision-making algorithm to handle various scenarios:
    -   **Confident Match:** If the embedding is very close to a known, validated person, it flags the target as valid.
    -   **New Person:** If the embedding is unlike any known person, it creates a new entry in the database.
    -   **Ambiguous Match:** If the match is weak, it may associate the new embedding with an existing person but mark it as "unvalidated," pending human review.
-   **Output:** The original detection data, now annotated with the person's ID and name.

### 3. Spatial Localization and Final Sync (`SerialSyncNode`)

-   **Input 1:** The annotated detection data from the `IdentificationNode`.
-   **Input 2:** 3D coordinate data from the `SpatialLocationCalculator`, which uses the stereo cameras and the ROI from the targeting node to find the (x, y, z) position of the aiming point.
-   **Custom Logic:** The `SerialSyncNode` is a custom *Threaded Host Node* designed to solve a common problem in complex pipelines: synchronization. It waits until it has received *both* the identification results and the corresponding spatial data for a given frame.
-   **Output:** Once synchronized, it packages the final information (Target ID, validity, and 3D coordinates) into a binary message using the `TargetMessage` class and sends it over the serial port to the firmware.

## Database Management

-   **Technology:** `sqlite3` with the `sqlite-vec` extension.
-   **Schema:** The database consists of three main tables:
    -   `person`: Stores metadata about an individual (e.g., name, validation status).
    -   `person_vector`: A virtual table optimized to store the 512-dimensional embedding vectors and perform fast cosine-similarity searches.
    -   `person_vector_link`: A linking table that allows multiple embeddings to be associated with a single person. This is important because a person's appearance can vary, resulting in slightly different embeddings.
-   **Image Caching:** When a new person is detected or an ambiguous match occurs, the script saves the cropped image of the person to the local filesystem. The filename includes the database IDs, making it easy for a human to review the images and validate or correct the system's classifications.