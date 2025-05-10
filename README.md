## Human Substitution with Avatar Using Pose Estimation- Interactive Mirror

This application detects a human in webcam feed, and in real-time removes it and replaces it with a 3D avatar. It can also cycle between views of just the segmentation, keypoints, or avatar replacement.

![demotestvid](https://github.com/user-attachments/assets/395a5d8f-77de-4b36-8cfe-ebbeb9938527) <br>
![image](https://github.com/user-attachments/assets/f6b3e08a-21c7-4a46-baba-61ddeab284ff)

## How to use

Requires a webcam and a GPU.

The project has been tested on Nvidia GeForce RTX 2070 and RTX 3060 Ti, and achieves a performance of about 60FPS and a delay of 30ms. It has only been tested on Windows operating system.

Works best when the Webcam has any kind of auto configuration disabled, like exposure and focus. This can be achieved through feeding the webcam through OBS Virtual Camera, disabling the features, and selecting the Virtual Camera as input when running the application.

### Link to the build

Link to the Windows build: <br>
https://drive.google.com/file/d/1lwLKUbnxKREkhIyDyJaCF-oiW6g6NFKk/view?usp=sharing

Extract the zip folder, then run PoseAnim.exe


## Outline

This is a Unity 6000.0.36f1 project, which can be opened with Unity Hub once cloned.

Unity Sentis is used to run [MediaPipe Pose Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker), which is a Single Human 3D Pose Estimation solution optimized for real time performance. It detects 33 keypoints and the human segmentation. Unity's [sentis-blaze-pose](https://huggingface.co/unity/sentis-blaze-pose) project was utilized and heavily modified to integrate it into this project.

The avatar used is [Unity-Chan!](https://assetstore.unity.com/packages/3d/characters/unity-chan-model-18705)

### Workflow

The human movement is transferred to the Avatar by calculating directional vectors between the keypoints and applying them to the appropriate Avatar joints. The approach used here was inspired by the [ThreeDPoseUnityBarracuda](https://github.com/digital-standard/ThreeDPoseUnityBarracuda) project.


The human is removed by storing the background information (areas not covered by the human segmentation) and using it to inpaint the human segmentation.





