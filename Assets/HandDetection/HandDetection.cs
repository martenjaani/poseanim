using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Mathematics;
using Unity.Sentis;
using UnityEngine;
using UnityEngine.UI;

public class HandDetection : MonoBehaviour
{
    public WebcamInput webcamInput;
    public ModelAsset handDetector;
    public ModelAsset handLandmarker;
    public TextAsset anchorsCSV;

    [Header("Detection Settings")]
    [Tooltip("Minimum score threshold for hand detection")]
    [Range(0f, 1f)]
    public float scoreThreshold = 0.5f;
    [Tooltip("IOU threshold for NMS filtering")]
    [Range(0f, 1f)]
    public float iouThreshold = 0.3f;
    [Tooltip("Maximum number of hands to detect")]
    [Range(1, 4)]
    public int maxHands = 2;

    [Header("Segmentation Integration")]
    public SegmentationRenderer segmentationRenderer;
    public bool useSegmentedPersonTexture = true;

    // Spheres to visualize each keypoint
    public GameObject spherePrefabRight;
    public GameObject spherePrefabLeft;
    private const int k_NumKeypoints = 21;
    private readonly List<GameObject> jointSpheresRight = new List<GameObject>();
    private readonly List<GameObject> jointSpheresLeft = new List<GameObject>();

    // Anchor points for hand detection
    const int k_NumAnchors = 2016;
    float[,] m_Anchors;

    // Input tensor dimensions
    const int detectorInputSize = 192;
    const int landmarkerInputSize = 224;

    // Sentis workers and tensors
    Worker m_HandDetectorWorker;
    Worker m_HandLandmarkerWorker;
    Tensor<float> m_DetectorInput;
    Tensor<float> m_LandmarkerInput;
    Awaitable m_DetectAwaitable;

    // Texture dimensions
    float m_TextureWidth;
    float m_TextureHeight;
    private Texture2D imageTexture;

    // Filtering Settings
    [Header("Filtering Settings")]
    [Tooltip("Enable/disable Kalman filtering")]
    public bool useKalmanFilter = true;

    // Kalman Filter Settings
    [Header("Kalman Filter Settings")]
    [Tooltip("Initial covariance of the filter")]
    public float kalmanInitialCovariance = 1.0f;
    [Tooltip("Process noise (higher = more responsive to changes)")]
    public float kalmanProcessNoise = 0.1f;
    [Tooltip("Measurement noise (higher = more smoothing)")]
    public float kalmanMeasurementNoise = 0.1f;

    // Kalman filters for right and left hands
    private KalmanFilter3D[] kalmanFiltersRight;
    private KalmanFilter3D[] kalmanFiltersLeft;

    [Header("One Euro Filtering")]
    [Tooltip("Enable/disable One Euro filtering")]
    public bool useOneEuroFilter = true;
    [Tooltip("Minimum cutoff frequency")]
    [Range(0.1f, 5.0f)]
    public float minCutoff = 1.0f;
    [Tooltip("Cutoff slope (higher = more aggressive filtering of fast movements)")]
    [Range(0, 0.1f)]
    public float beta = 0.007f;
    [Tooltip("Derivative cutoff frequency")]
    [Range(0.1f, 5.0f)]
    public float dCutoff = 1.0f;

    // One Euro filters for right and left hands
    private OneEuroFilter[] oneEuroFiltersRight;
    private OneEuroFilter[] oneEuroFiltersLeft;

    [Header("ROI Tracking")]
    [Tooltip("Enable ROI tracking between frames")]
    public bool useROITracking = true;
    [Tooltip("How many frames to wait between full detections")]
    [Range(1, 10)]
    public int detectionInterval = 3;
    [Tooltip("ROI expansion factor (1.5 = 50% larger than previous detection)")]
    [Range(1.1f, 2.0f)]
    public float roiExpansionFactor = 1.5f;
    [Tooltip("Score threshold for tracking mode (can be lower than detection)")]
    [Range(0.1f, 1.0f)]
    public float trackingScoreThreshold = 0.4f;
    [Tooltip("Maximum frames to track before forcing new detection")]
    [Range(10, 100)]
    public int maxTrackingFrames = 30;

    // Tracking state
    private bool isTracking = false;
    private int frameCounter = 0;
    private int trackingFrameCount = 0;
    private Rect leftHandROI = new Rect(0, 0, 0, 0);
    private Rect rightHandROI = new Rect(0, 0, 0, 0);
    private bool leftHandROIValid = false;
    private bool rightHandROIValid = false;
    private float2x3 leftHandTransform;
    private float2x3 rightHandTransform;

    // Output keypoint positions for right and left hands
    [Header("Hand Data")]
    public Vector3[] rightHandKeypoints = new Vector3[k_NumKeypoints];
    public Vector3[] leftHandKeypoints = new Vector3[k_NumKeypoints];
    public float[] rightHandConfidence = new float[k_NumKeypoints];
    public float[] leftHandConfidence = new float[k_NumKeypoints];

    // Hand tracking info
    public bool rightHandDetected = false;
    public bool leftHandDetected = false;
    public float rightHandScore = 0f;
    public float leftHandScore = 0f;

    [Tooltip("Scale factor for visualization")]
    public float scaleFactor = 20.0f;
    public bool showSpheres = false;
    [Header("Debug")]
    public bool showDebugInfo = false;

    public async void Start()
    {
        // Create spheres for right hand keypoints
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            GameObject sphere = Instantiate(spherePrefabRight, this.transform);
            sphere.name = "RightHandJoint_" + i;
            jointSpheresRight.Add(sphere);
            sphere.SetActive(false);
        }

        // Create spheres for left hand keypoints
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            GameObject sphere = Instantiate(spherePrefabLeft, this.transform);
            sphere.name = "LeftHandJoint_" + i;
            jointSpheresLeft.Add(sphere);
            sphere.SetActive(false);
        }

        // Initialize Kalman filters for right hand
        kalmanFiltersRight = new KalmanFilter3D[k_NumKeypoints];
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            kalmanFiltersRight[i] = new KalmanFilter3D(Vector3.zero, kalmanInitialCovariance, kalmanProcessNoise, kalmanMeasurementNoise);
        }

        // Initialize Kalman filters for left hand
        kalmanFiltersLeft = new KalmanFilter3D[k_NumKeypoints];
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            kalmanFiltersLeft[i] = new KalmanFilter3D(Vector3.zero, kalmanInitialCovariance, kalmanProcessNoise, kalmanMeasurementNoise);
        }

        // Initialize One Euro Filters for right hand
        oneEuroFiltersRight = new OneEuroFilter[k_NumKeypoints];
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            oneEuroFiltersRight[i] = new OneEuroFilter(minCutoff, beta, dCutoff);
        }

        // Initialize One Euro Filters for left hand
        oneEuroFiltersLeft = new OneEuroFilter[k_NumKeypoints];
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            oneEuroFiltersLeft[i] = new OneEuroFilter(minCutoff, beta, dCutoff);
        }

        m_Anchors = BlazeUtilsHand.LoadAnchors(anchorsCSV.text, k_NumAnchors);

        // Initialize the hand detection model with NMS filtering for multiple hands
        var handDetectorModel = ModelLoader.Load(handDetector);
        var graph = new FunctionalGraph();
        var input = graph.AddInput(handDetectorModel, 0);
        var outputs = Functional.Forward(handDetectorModel, input);
        var boxes = outputs[0]; // (1, 2016, 18)
        var scores = outputs[1]; // (1, 2016, 1)

        // Create anchors tensor for NMS filtering
        var anchorsData = new float[k_NumAnchors * 4];
        Buffer.BlockCopy(m_Anchors, 0, anchorsData, 0, anchorsData.Length * sizeof(float));
        var anchors = Functional.Constant(new TensorShape(k_NumAnchors, 4), anchorsData);

        // Apply NMS filtering
        var idx_scores_boxes = BlazeUtilsHand.NMSFiltering(boxes, scores, anchors, detectorInputSize, iouThreshold, scoreThreshold);
        handDetectorModel = graph.Compile(idx_scores_boxes.Item1, idx_scores_boxes.Item2, idx_scores_boxes.Item3);

        m_HandDetectorWorker = new Worker(handDetectorModel, BackendType.GPUCompute);

        // Initialize the hand landmark model
        var handLandmarkerModel = ModelLoader.Load(handLandmarker);
        m_HandLandmarkerWorker = new Worker(handLandmarkerModel, BackendType.GPUCompute);

        m_DetectorInput = new Tensor<float>(new TensorShape(1, detectorInputSize, detectorInputSize, 3));
        m_LandmarkerInput = new Tensor<float>(new TensorShape(1, landmarkerInputSize, landmarkerInputSize, 3));

        // Wait for webcam initialization
        while (webcamInput.GetFrame() == null)
        {
            Debug.Log("Waiting for webcam initialization...");
            await Task.Delay(100);
        }
        imageTexture = webcamInput.GetFrame();
        m_TextureWidth = imageTexture.width;
        m_TextureHeight = imageTexture.height;

        // Detection loop
        while (true)
        {
            try
            {
                frameCounter++;

                // Determine which mode to use: full detection or tracking
                bool shouldRunFullDetection = ShouldRunFullDetection();
                Texture textureToProcess = GetInputTexture();

                if (textureToProcess == null)
                {
                    await Task.Delay(100);
                    continue;
                }

                if (shouldRunFullDetection)
                {
                    // Run full hand detection on the entire frame
                    isTracking = false;
                    m_DetectAwaitable = DetectHands(textureToProcess);
                    await m_DetectAwaitable;
                    trackingFrameCount = 0;
                }
                else
                {
                    // Track hands using ROIs from previous detection
                    isTracking = true;
                    trackingFrameCount++;
                    m_DetectAwaitable = TrackHands(textureToProcess);
                    await m_DetectAwaitable;
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception e)
            {
                Debug.LogError($"Error in hand detection: {e.Message}");
            }
        }

        CleanupResources();
    }

    // Helper method to determine when to run full detection vs tracking
    private bool ShouldRunFullDetection()
    {
        // Always run detection if not using ROI tracking
        if (!useROITracking)
            return true;

        // Run detection on the first frame
        if (frameCounter == 1)
            return true;

        // Force detection after max tracking frames
        if (trackingFrameCount >= maxTrackingFrames)
            return true;

        // Run detection at regular intervals
        if (frameCounter % detectionInterval == 0)
            return true;

        // If no valid ROIs exist, run full detection
        if (!leftHandROIValid && !rightHandROIValid)
            return true;

        // Otherwise continue tracking
        return false;
    }

    // Helper method to get the input texture based on preferences
    private Texture GetInputTexture()
    {
        Texture textureToProcess = null;

        // Try to get the person-only texture first (highest priority)
        if (useSegmentedPersonTexture && segmentationRenderer != null)
        {
            textureToProcess = segmentationRenderer.GetPersonOnlyTexture();
            if (textureToProcess != null && showDebugInfo)
            {
                Debug.Log("Using segmented person-only texture for hand detection");
            }
        }

        // If segmentation texture not available, try pose detection crop
        if (textureToProcess == null)
        {
            // Use the full webcam feed
            textureToProcess = webcamInput.GetFeedFrameCopy();
            if (showDebugInfo && textureToProcess != null)
            {
                Debug.Log("Using full webcam feed for hand detection");
            }
        }

        return textureToProcess;
    }

    private void CleanupResources()
    {
        m_HandDetectorWorker.Dispose();
        m_HandLandmarkerWorker.Dispose();
        m_DetectorInput.Dispose();
        m_LandmarkerInput.Dispose();
    }

    Vector3 ImageToWorld(Vector2 position)
    {
        return (position - 0.5f * new Vector2(m_TextureWidth, m_TextureHeight)) / m_TextureHeight;
    }

    // Full detection method (refactored from original Detect method)
    async Awaitable DetectHands(Texture texture)
    {
        m_TextureWidth = texture.width;
        m_TextureHeight = texture.height;

        var size = Mathf.Max(texture.width, texture.height);

        // Transformation matrix for detector input
        var scale = size / (float)detectorInputSize;
        var M = BlazeUtilsHand.mul(
            BlazeUtilsHand.TranslationMatrix(0.5f * (new float2(texture.width, texture.height) + new float2(-size, size))),
            BlazeUtilsHand.ScaleMatrix(new float2(scale, -scale))
        );
        BlazeUtilsHand.SampleImageAffine(texture, m_DetectorInput, M);

        // Run hand detector
        m_HandDetectorWorker.Schedule(m_DetectorInput);

        // Get detector outputs
        var outputIndicesAwaitable = (m_HandDetectorWorker.PeekOutput(0) as Tensor<int>).ReadbackAndCloneAsync();
        var outputScoresAwaitable = (m_HandDetectorWorker.PeekOutput(1) as Tensor<float>).ReadbackAndCloneAsync();
        var outputBoxesAwaitable = (m_HandDetectorWorker.PeekOutput(2) as Tensor<float>).ReadbackAndCloneAsync();

        using var outputIndices = await outputIndicesAwaitable;
        using var outputScores = await outputScoresAwaitable;
        using var outputBoxes = await outputBoxesAwaitable;

        // Get number of detected hands (limited by maxHands)
        var numHands = Math.Min(outputIndices.shape.length, maxHands);

        // Reset hand detection flags
        rightHandDetected = false;
        leftHandDetected = false;
        rightHandROIValid = false;
        leftHandROIValid = false;

        // If no hands detected, return early
        if (numHands == 0)
        {
            await Awaitable.NextFrameAsync();
            return;
        }

        // Process each detected hand
        for (var handIndex = 0; handIndex < numHands; handIndex++)
        {
            var idx = outputIndices[handIndex];
            var anchorPosition = detectorInputSize * new float2(m_Anchors[idx, 0], m_Anchors[idx, 1]);

            var boxCentre_TensorSpace = anchorPosition + new float2(outputBoxes[0, handIndex, 0], outputBoxes[0, handIndex, 1]);
            var boxSize_TensorSpace = math.max(outputBoxes[0, handIndex, 2], outputBoxes[0, handIndex, 3]);

            var kp0_TensorSpace = anchorPosition + new float2(outputBoxes[0, handIndex, 4 + 2 * 0 + 0], outputBoxes[0, handIndex, 4 + 2 * 0 + 1]);
            var kp2_TensorSpace = anchorPosition + new float2(outputBoxes[0, handIndex, 4 + 2 * 2 + 0], outputBoxes[0, handIndex, 4 + 2 * 2 + 1]);
            var delta_TensorSpace = kp2_TensorSpace - kp0_TensorSpace;
            var up_TensorSpace = delta_TensorSpace / math.length(delta_TensorSpace);
            var theta = math.atan2(delta_TensorSpace.y, delta_TensorSpace.x);
            var rotation = 0.5f * Mathf.PI - theta;
            boxCentre_TensorSpace += 0.5f * boxSize_TensorSpace * up_TensorSpace;
            boxSize_TensorSpace *= 2.6f;

            var origin2 = new float2(0.5f * landmarkerInputSize, 0.5f * landmarkerInputSize);
            var scale2 = boxSize_TensorSpace / landmarkerInputSize;
            var M2 = BlazeUtilsHand.mul(
                M,
                BlazeUtilsHand.mul(
                    BlazeUtilsHand.mul(
                        BlazeUtilsHand.mul(
                            BlazeUtilsHand.TranslationMatrix(boxCentre_TensorSpace),
                            BlazeUtilsHand.ScaleMatrix(new float2(scale2, -scale2))
                        ),
                        BlazeUtilsHand.RotationMatrix(rotation)
                    ),
                    BlazeUtilsHand.TranslationMatrix(-origin2)
                )
            );

            // Calculate hand bounding box in image space for ROI tracking
            var handCenter_ImageSpace = BlazeUtilsHand.mul(M, boxCentre_TensorSpace);
            var handTopRight_ImageSpace = BlazeUtilsHand.mul(M, boxCentre_TensorSpace + new float2(boxSize_TensorSpace * 0.5f, boxSize_TensorSpace * 0.5f));
            var handBottomLeft_ImageSpace = BlazeUtilsHand.mul(M, boxCentre_TensorSpace - new float2(boxSize_TensorSpace * 0.5f, boxSize_TensorSpace * 0.5f));

            float roiWidth = Mathf.Abs(handTopRight_ImageSpace.x - handBottomLeft_ImageSpace.x) * roiExpansionFactor;
            float roiHeight = Mathf.Abs(handTopRight_ImageSpace.y - handBottomLeft_ImageSpace.y) * roiExpansionFactor;

            // Create ROI rectangle for this hand
            Rect handROI = new Rect(
                Mathf.Max(0, handCenter_ImageSpace.x - roiWidth * 0.5f),
                Mathf.Max(0, handCenter_ImageSpace.y - roiHeight * 0.5f),
                Mathf.Min(m_TextureWidth - handCenter_ImageSpace.x + roiWidth * 0.5f, roiWidth),
                Mathf.Min(m_TextureHeight - handCenter_ImageSpace.y + roiHeight * 0.5f, roiHeight)
            );

            // Sample image for landmarker
            BlazeUtilsHand.SampleImageAffine(texture, m_LandmarkerInput, M2);

            // Schedule landmarker
            m_HandLandmarkerWorker.Schedule(m_LandmarkerInput);

            // Get landmark positions
            var landmarksAwaitable = (m_HandLandmarkerWorker.PeekOutput("Identity") as Tensor<float>).ReadbackAndCloneAsync();

            // Get handedness classification
            var handednessAwaitable = (m_HandLandmarkerWorker.PeekOutput("Identity_2") as Tensor<float>).ReadbackAndCloneAsync();

            using var landmarks = await landmarksAwaitable;
            using var handedness = await handednessAwaitable;

            // Determine if this is a left or right hand
            // handedness[0] > 0.5 means right hand, < 0.5 means left hand
            bool isRightHand = handedness[0] < 0.5f;
            float handScore = outputScores[0, handIndex, 0];

            // Skip processing if we already have a higher-confidence hand of this type
            if (isRightHand)
            {
                if (rightHandDetected && rightHandScore >= handScore)
                    continue;

                rightHandDetected = true;
                rightHandScore = handScore;
                rightHandROI = handROI;
                rightHandTransform = M2;
                rightHandROIValid = true;
            }
            else
            {
                if (leftHandDetected && leftHandScore >= handScore)
                    continue;

                leftHandDetected = true;
                leftHandScore = handScore;
                leftHandROI = handROI;
                leftHandTransform = M2;
                leftHandROIValid = true;
            }

            // Process keypoints for the current hand
            ProcessHandKeypoints(landmarks, M2, isRightHand, handScore);

            if (showDebugInfo)
            {
                Debug.Log($"Detected {(isRightHand ? "RIGHT" : "LEFT")} hand with score {handScore:F2}, ROI: {handROI}");
            }
        }
    }

    // Tracking method for when we're using ROIs
    async Awaitable TrackHands(Texture texture)
    {
        m_TextureWidth = texture.width;
        m_TextureHeight = texture.height;

        // Track each hand separately based on their ROIs
        if (rightHandROIValid)
        {
            await TrackSingleHand(texture, true);
        }

        if (leftHandROIValid)
        {
            await TrackSingleHand(texture, false);
        }

        // If no valid ROIs, force full detection next frame
        if (!rightHandROIValid && !leftHandROIValid)
        {
            trackingFrameCount = maxTrackingFrames;
        }
    }

    // Track a single hand using its ROI from previous detection
    async Awaitable TrackSingleHand(Texture texture, bool isRightHand)
    {
        Rect handROI = isRightHand ? rightHandROI : leftHandROI;
        float2x3 handTransform = isRightHand ? rightHandTransform : leftHandTransform;

        // Check if ROI is valid and within texture bounds
        if (handROI.width <= 0 || handROI.height <= 0 ||
            handROI.x < 0 || handROI.y < 0 ||
            handROI.x + handROI.width > texture.width ||
            handROI.y + handROI.height > texture.height)
        {
            // Invalid ROI, mark it as invalid
            if (isRightHand)
                rightHandROIValid = false;
            else
                leftHandROIValid = false;

            return;
        }

        try
        {
            // Sample image directly using the saved transform from previous detection
            BlazeUtilsHand.SampleImageAffine(texture, m_LandmarkerInput, handTransform);

            // Run landmarker on this region
            m_HandLandmarkerWorker.Schedule(m_LandmarkerInput);

            // Get landmark positions and handedness
            var landmarksAwaitable = (m_HandLandmarkerWorker.PeekOutput("Identity") as Tensor<float>).ReadbackAndCloneAsync();
            var handednessAwaitable = (m_HandLandmarkerWorker.PeekOutput("Identity_2") as Tensor<float>).ReadbackAndCloneAsync();

            using var landmarks = await landmarksAwaitable;
            using var handedness = await handednessAwaitable;

            // Confirm this is still the expected hand type
            bool detectedRightHand = handedness[0] < 0.5f;

            // If the detected hand doesn't match the expected type, invalidate the ROI
            if (detectedRightHand != isRightHand)
            {
                if (isRightHand)
                    rightHandROIValid = false;
                else
                    leftHandROIValid = false;

                if (showDebugInfo)
                    Debug.Log($"Hand type mismatch during tracking: expected {(isRightHand ? "RIGHT" : "LEFT")}, got {(detectedRightHand ? "RIGHT" : "LEFT")}");

                return;
            }

            // Use a possibly lower confidence threshold for tracking
            float handScore = isRightHand ? rightHandScore : leftHandScore;

            // If confidence is too low, invalidate the ROI
            if (handScore < trackingScoreThreshold)
            {
                if (isRightHand)
                    rightHandROIValid = false;
                else
                    leftHandROIValid = false;

                if (showDebugInfo)
                    Debug.Log($"Confidence too low during tracking for {(isRightHand ? "RIGHT" : "LEFT")} hand: {handScore:F2}");

                return;
            }

            // Process keypoints
            ProcessHandKeypoints(landmarks, handTransform, isRightHand, handScore);

            if (showDebugInfo)
            {
                Debug.Log($"Tracked {(isRightHand ? "RIGHT" : "LEFT")} hand with score {handScore:F2}, frame {trackingFrameCount}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error tracking {(isRightHand ? "RIGHT" : "LEFT")} hand: {e.Message}");

            // Invalidate the ROI on error
            if (isRightHand)
                rightHandROIValid = false;
            else
                leftHandROIValid = false;
        }
    }

    // Extract the keypoint processing code into a separate method to avoid duplication
    private void ProcessHandKeypoints(Tensor<float> landmarks, float2x3 M2, bool isRightHand, float handScore)
    {
        for (var i = 0; i < k_NumKeypoints; i++)
        {
            var position_ImageSpace = BlazeUtilsHand.mul(M2, new float2(landmarks[3 * i + 0], landmarks[3 * i + 1]));

            // Z value is provided directly in the landmarks
            Vector3 position_WorldSpace = ImageToWorld(position_ImageSpace) + new Vector3(0, 0, landmarks[3 * i + 2] * 10 / m_TextureHeight);

            // Flip x and z to match pose detection coordinate system
            position_WorldSpace.x *= -1;
            position_WorldSpace.z *= -1;

            // Apply multi-stage filtering in sequence
            Vector3 filteredPosition = position_WorldSpace;

            // Apply One Euro filtering first (if enabled)
            if (useOneEuroFilter)
            {
                if (isRightHand && oneEuroFiltersRight != null && i < oneEuroFiltersRight.Length)
                {
                    filteredPosition = oneEuroFiltersRight[i].Filter(filteredPosition);
                }
                else if (!isRightHand && oneEuroFiltersLeft != null && i < oneEuroFiltersLeft.Length)
                {
                    filteredPosition = oneEuroFiltersLeft[i].Filter(filteredPosition);
                }
            }

            // Apply Kalman filtering as the second stage (if enabled)
            if (useKalmanFilter)
            {
                if (isRightHand && kalmanFiltersRight != null && i < kalmanFiltersRight.Length)
                {
                    filteredPosition = kalmanFiltersRight[i].Update(filteredPosition);
                }
                else if (!isRightHand && kalmanFiltersLeft != null && i < kalmanFiltersLeft.Length)
                {
                    filteredPosition = kalmanFiltersLeft[i].Update(filteredPosition);
                }
            }

            // Store keypoint position and update confidence
            if (isRightHand)
            {
                rightHandKeypoints[i] = filteredPosition;
                rightHandConfidence[i] = handScore;

                // Update visualization
                if (showSpheres && i < jointSpheresRight.Count)
                {
                    jointSpheresRight[i].transform.localPosition = filteredPosition * scaleFactor;
                    jointSpheresRight[i].SetActive(true);

                    // Adjust size based on joint type
                    float baseSize = 0.5f;
                    // Make fingertips a bit smaller
                    if (i % 4 == 0 && i > 0) // Fingertips are at indices 4, 8, 12, 16, 20
                    {
                        baseSize = 0.3f;
                    }
                    // Make wrist and palm joints larger
                    else if (i == 0 || i == 1 || i == 2 || i == 5 || i == 9 || i == 13 || i == 17)
                    {
                        baseSize = 0.7f;
                    }

                    jointSpheresRight[i].transform.localScale = Vector3.one * baseSize;
                }
            }
            else
            {
                leftHandKeypoints[i] = filteredPosition;
                leftHandConfidence[i] = handScore;

                // Update visualization
                if (showSpheres && i < jointSpheresLeft.Count)
                {
                    jointSpheresLeft[i].transform.localPosition = filteredPosition * scaleFactor;
                    jointSpheresLeft[i].SetActive(true);

                    // Adjust size based on joint type
                    float baseSize = 0.5f;
                    // Make fingertips a bit smaller
                    if (i % 4 == 0 && i > 0) // Fingertips are at indices 4, 8, 12, 16, 20
                    {
                        baseSize = 0.3f;
                    }
                    // Make wrist and palm joints larger
                    else if (i == 0 || i == 1 || i == 2 || i == 5 || i == 9 || i == 13 || i == 17)
                    {
                        baseSize = 0.7f;
                    }

                    jointSpheresLeft[i].transform.localScale = Vector3.one * baseSize;
                }
            }
        }
    }

    /// <summary>
    /// Get specific keypoint index for a hand part
    /// </summary>
    public enum HandKeypoint
    {
        Wrist = 0,
        ThumbCMC = 1,
        ThumbMCP = 2,
        ThumbIP = 3,
        ThumbTip = 4,
        IndexMCP = 5,
        IndexPIP = 6,
        IndexDIP = 7,
        IndexTip = 8,
        MiddleMCP = 9,
        MiddlePIP = 10,
        MiddleDIP = 11,
        MiddleTip = 12,
        RingMCP = 13,
        RingPIP = 14,
        RingDIP = 15,
        RingTip = 16,
        PinkyMCP = 17,
        PinkyPIP = 18,
        PinkyDIP = 19,
        PinkyTip = 20
    }

    /// <summary>
    /// Get a specific hand keypoint position for the right hand
    /// </summary>
    public Vector3 GetRightHandKeypointPosition(HandKeypoint keypoint)
    {
        int index = (int)keypoint;
        if (rightHandDetected && index >= 0 && index < rightHandKeypoints.Length)
        {
            return rightHandKeypoints[index];
        }
        return Vector3.zero;
    }

    /// <summary>
    /// Get a specific hand keypoint position for the left hand
    /// </summary>
    public Vector3 GetLeftHandKeypointPosition(HandKeypoint keypoint)
    {
        int index = (int)keypoint;
        if (leftHandDetected && index >= 0 && index < leftHandKeypoints.Length)
        {
            return leftHandKeypoints[index];
        }
        return Vector3.zero;
    }

    /// <summary>
    /// Get the confidence value for a specific keypoint of the right hand
    /// </summary>
    public float GetRightHandKeypointConfidence(HandKeypoint keypoint)
    {
        int index = (int)keypoint;
        if (rightHandDetected && index >= 0 && index < rightHandConfidence.Length)
        {
            return rightHandConfidence[index];
        }
        return 0f;
    }

    /// <summary>
    /// Get the confidence value for a specific keypoint of the left hand
    /// </summary>
    public float GetLeftHandKeypointConfidence(HandKeypoint keypoint)
    {
        int index = (int)keypoint;
        if (leftHandDetected && index >= 0 && index < leftHandConfidence.Length)
        {
            return leftHandConfidence[index];
        }
        return 0f;
    }

    // Optional: Add visualization for debugging
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || !useROITracking)
            return;

        // Draw right hand ROI if valid
        if (rightHandROIValid)
        {
            Gizmos.color = Color.red;
            DrawROIGizmo(rightHandROI);
        }

        // Draw left hand ROI if valid
        if (leftHandROIValid)
        {
            Gizmos.color = Color.green;
            DrawROIGizmo(leftHandROI);
        }
    }

    private void DrawROIGizmo(Rect roi)
    {
        Vector3 center = new Vector3(roi.center.x, roi.center.y, 0.1f);
        Vector3 size = new Vector3(roi.width, roi.height, 0.01f);
        Gizmos.DrawWireCube(center, size);
    }

    // Add method to update filter parameters when changed in the inspector
    private void UpdateFilterParameters()
    {
        // Update One Euro filter parameters if they exist
        if (oneEuroFiltersRight != null)
        {
            for (int i = 0; i < k_NumKeypoints; i++)
            {
                oneEuroFiltersRight[i].SetParameters(minCutoff, beta, dCutoff);
            }
        }

        if (oneEuroFiltersLeft != null)
        {
            for (int i = 0; i < k_NumKeypoints; i++)
            {
                oneEuroFiltersLeft[i].SetParameters(minCutoff, beta, dCutoff);
            }
        }
    }

    // Call this method from an Update method if you want to dynamically adjust parameters during runtime
    private void Update()
    {
        // Update filter parameters if needed
        UpdateFilterParameters();
    }

    void OnDestroy()
    {
        m_DetectAwaitable.Cancel();
    }
}