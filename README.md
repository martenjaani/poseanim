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

Unity Sentis is used to run [MediaPipe Pose Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker), which is a Single Human 3D Pose Estimation solution optimized for real-time performance. It detects 33 keypoints and the human segmentation. Unity's [sentis-blaze-pose](https://huggingface.co/unity/sentis-blaze-pose) project was utilized and heavily modified to integrate it into this project.

The avatar used is [Unity-Chan!](https://assetstore.unity.com/packages/3d/characters/unity-chan-model-18705)

## Source code and workflow

The main scripts responsible for the workflow are as follows: 
- ```Assets/PoseDetection/Scripts/PoseDetection.cs```: Handles the running of the pose detection model. This is heavily modified from [sentis-blaze-pose](https://huggingface.co/unity/sentis-blaze-pose) to work continuously in real-time and apply Kalman and [1€ Filters](https://gery.casiez.net/1euro/), also takes the output of the segmentation, and transforms it into the image space.
- ```Assets/Scripts/AvatarController.cs```: Handles the transfer of detected keypoints into avatar movement, using a forward kinematic approach. The avatar is animated by calculating directional vectors between the keypoints and using them to find the rotations that turn the joints so they match the directional vectors. The forward kinematics approach used here was inspired by the [ThreeDPoseUnityBarracuda](https://github.com/digital-standard/ThreeDPoseUnityBarracuda) project.
- ```Assets/Scripts/SegmentationRenderer.cs```: Handles the processing of the human segmentation. Responsible for removing the human from output. The human is removed by storing the background information (areas not covered by the human segmentation) and using it to inpaint the human segmentation.
- ```Assets/Scripts/ViewController.cs```:  Cycles between views of just the segmentation, keypoints, or avatar replacement.

<br>



Also, ```Assets/Scripts/HandAvatarController.cs``` and ```Assets/HandDetection``` were used at a point also to animate the fingers of the avatar. This solution used a modified [sentis-blaze-hand](https://huggingface.co/unity/sentis-blaze-hand) project, which itself uses [Google Mediapipe's Hand Landmarker task](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker). The feature was left out since it did not work reliably in situations where the human was more than 2 meters away and hands were not clearly visible and facing the camera. Also, the existing code only worked for the [Mixamo avatar](https://www.mixamo.com/#/?page=3&type=Character), as seen in the video, and did not work for Unity-Chan!, due to different rigging. 


![ezgif-5caebe0b301be1](https://github.com/user-attachments/assets/4040762c-7f17-457d-80b6-463f2ceedbbf)






