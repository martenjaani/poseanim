## Human Substitution with Avatar Using Pose Estimation- Interactive Mirror

This application detects a human in webcam feed, and in real-time removes it and replaces it with a 3D avatar. It can also cycle between views of just the segmentation, keypoints, or avatar replacement.

![demotestvid](https://github.com/user-attachments/assets/395a5d8f-77de-4b36-8cfe-ebbeb9938527)
## How to use

Requires a webcam and a GPU.

The project has been tested on Nvidia GeForce RTX2070 and RTX3060Ti, and achieves a performance of about 60FPS and a delay of 30ms. It has only been tested on Windows operating system.

Works best when Webcam has any kind of auto configuration disabled, like exposure and focus. This can be achieved through feeding the webcam through OBS Virtual Camera, disabling the features and selecting the Virtual Camera as input when running the application.

## Link to the build

Link to the Windows build: <br>
https://drive.google.com/file/d/1sK-Oy6vpyHsJUZ8GV9DDdpGePz124jqg/view?usp=sharing


### Outline

This is a Unity 6000.0.36f1 project, can be opened with Unity Hub once cloned.

Unity Sentis is used to run the MediaPipe Pose Landmarker, which is a Single Human 3D Pose Estimation solution optimized for real time performance. It detects 33 keypoints and the human segmentation
https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

To integrate it into this project, Unity's sentis-blaze-pose project was utilized and heavily modified.
https://huggingface.co/unity/sentis-blaze-pose

### Workflow

The human movement is transfered to the Avatar by calculating directional vectors between the keypoints and applying them to the appropriate Avatar joints. The approach used here was inspired by ThreeDPoseUnityBarracuda project:

https://github.com/digital-standard/ThreeDPoseUnityBarracuda

The human is removed by storing the backround information (areas not covered by the human segmentation) and using it to inpaint the human segmentation.





