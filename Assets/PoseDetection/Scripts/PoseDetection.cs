using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using TMPro;
using Unity.Mathematics;
using Unity.Sentis;
using UnityEngine;
using UnityEngine.UI;

public class PoseDetection : MonoBehaviour
{
    // Core components
    public WebcamInput webcamInput;
    public ModelAsset poseDetector;
    public ModelAsset poseLandmarker;
    public TextAsset anchorsCSV;
    public float scoreThreshold = 0.75f;
    public SegmentationRenderer segmentationRenderer;
    public PosePreview posePreview;

    // Display objects
    public GameObject avatar;
    public GameObject outputImage;
    public GameObject inputImage;
    public bool avatarAvailable;
    public bool showFPS;
    public GameObject performancePanel;

    [Header("Detection Feedback UI")]
    public bool enableDetectionFeedback = true;
    public CanvasGroup detectionFeedbackPanel;
    public TextMeshProUGUI detectionFeedbackText;
    [Tooltip("Number of frames to track for detection history")]
    public int detectionHistoryLength = 60;
    [Tooltip("Threshold score for considering a detection as failed")]
    public float detectionFailThreshold = 0.6f;
    [Tooltip("Percentage of failed detections required to show feedback (0-100%)")]
    [Range(0, 100)]
    public float failPercentageThreshold = 50f;
    public float fadeDuration = 0.25f;

    [Header("Filtering Settings")]
    [Tooltip("Enable/disable One Euro filtering")]
    public bool useOneEuroFilter = true;
    [Tooltip("Enable/disable Kalman filtering")]
    public bool useKalmanFilter = true;

    [Header("One Euro Filter Settings")]
    [Tooltip("Minimum cutoff frequency")]
    [Range(0.1f, 5.0f)]
    public float minCutoff = 1.0f;
    [Tooltip("Cutoff slope (higher = more aggressive filtering of fast movements)")]
    [Range(0, 0.1f)]
    public float beta = 0.007f;
    [Tooltip("Derivative cutoff frequency")]
    [Range(0.1f, 5.0f)]
    public float dCutoff = 1.0f;

    [Header("Kalman Filter Settings")]
    [Tooltip("Initial covariance of the filter")]
    public float kalmanInitialCovariance = 1.0f;
    [Tooltip("Process noise (higher = more responsive to changes)")]
    public float kalmanProcessNoise = 0.1f;
    [Tooltip("Measurement noise (higher = more smoothing)")]
    public float kalmanMeasurementNoise = 0.1f;

    [Header("Real-time Optimization Settings")]
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
    public float trackingScoreThreshold = 0.6f;
    [Tooltip("Maximum frames to track before forcing new detection")]
    [Range(10, 100)]
    public int maxTrackingFrames = 30;

    [Header("Debug")]
    public bool showDebugInfo = false;
    public Text debugText;

    // Constants
    private const int k_NumKeypoints = 33;
    private const int k_NumAnchors = 2254;
    private const int detectorInputSize = 224;
    private const int landmarkerInputSize = 256;

    // ML workers and tensors
    private Worker m_PoseDetectorWorker;
    private Worker m_PoseLandmarkerWorker;
    private Tensor<float> m_DetectorInput;
    private Tensor<float> m_LandmarkerInput;
    private Awaitable m_DetectAwaitable;
    private float[,] m_Anchors;

    // Detection state
    private float m_TextureWidth;
    private float m_TextureHeight;
    private Texture2D imageTexture;
    private bool isFeedbackActive = false;
    private Queue<float> detectionScoreHistory = new Queue<float>();
    private int failedDetectionsCount = 0;

    // Filters
    private OneEuroFilter[] oneEuroFilters;
    private KalmanFilter3D[] kalmanFilters;

    // Output keypoint positions
    public Vector3[] keypoints = new Vector3[k_NumKeypoints];
    private float[] keypointConfidence = new float[k_NumKeypoints];

    // Tracking state
    private bool isTracking = false;
    private int frameCounter = 0;
    private int trackingFrameCount = 0;
    private float2x3 previousTransform;
    private float previousScore = 0;
    private CancellationTokenSource cancellationTokenSource;

    // Performance tracking
    private float frameStartTime;
    private float lastFrameTime;
    private float inferenceStartTime;
    private float lastInferenceTime;
    private float fpsUpdateInterval = 0.5f;
    private float fpsAccumulator = 0f;
    private int fpsFrameCount = 0;
    private float currentFps = 0;
    public TextMeshProUGUI performanceText;

    public async void Start()
    {
        // Initialize detection history tracking
        detectionScoreHistory = new Queue<float>(detectionHistoryLength);
        failedDetectionsCount = 0;



        // Initialize CanvasGroup properties
        if (detectionFeedbackPanel != null)
        {
            // Start with panel invisible
            detectionFeedbackPanel.alpha = 0f;
            detectionFeedbackPanel.interactable = false;
            detectionFeedbackPanel.blocksRaycasts = false;
            Debug.Log("CanvasGroup initialized: " + detectionFeedbackPanel.gameObject.name);
        }
        else
        {
            Debug.LogError("Detection feedback panel is not assigned!");
        }

        if (detectionFeedbackText == null)
        {
            Debug.LogError("Detection feedback text is not assigned!");
        }

        // Initialize multi-stage filters
        InitializeFilters();

        m_Anchors = BlazeUtilsHand.LoadAnchors(anchorsCSV.text, k_NumAnchors);

        InitializeModels();

        cancellationTokenSource = new CancellationTokenSource();

        try
        {
            // Pass the token to all awaitable operations
            // Wait for webcam initialization with cancellation support
            while (webcamInput.GetFrame() == null || webcamInput.GetFeedFrameCopy() == null)
            {
                Debug.Log("Waiting for webcam initialization...");
                await Task.Delay(100, cancellationTokenSource.Token);

                // Check if we've been cancelled
                cancellationTokenSource.Token.ThrowIfCancellationRequested();
            }
        }
        catch (OperationCanceledException)
        {
            Debug.Log("Webcam initialization was cancelled");
            return;
        }
        catch (Exception e)
        {
            Debug.LogError($"Error during initialization: {e.Message}");
            return;
        }

        imageTexture = webcamInput.GetFrame();
        m_TextureWidth = imageTexture.width;
        m_TextureHeight = imageTexture.height;

        if (segmentationRenderer != null)
        {
            segmentationRenderer.Initialize(landmarkerInputSize, webcamInput.GetFeedFrameCopy());
        }

        // Detection loop
        while (true)
        {
            try
            {
                // Start timing this frame
                frameStartTime = Time.realtimeSinceStartup;

                frameCounter++;

                // Decide whether to use detection or tracking mode
                bool shouldRunDetection = ShouldRunDetection();

                // Start timing inference
                inferenceStartTime = Time.realtimeSinceStartup;

                if (shouldRunDetection)
                {
                    // Full detection mode
                    isTracking = false;
                    m_DetectAwaitable = DetectPose(imageTexture);
                    await m_DetectAwaitable;
                }
                else
                {
                    // Tracking mode - use ROI from previous detection
                    isTracking = true;
                    m_DetectAwaitable = TrackPose(imageTexture);
                    await m_DetectAwaitable;
                }

                // Calculate inference time
                lastInferenceTime = (Time.realtimeSinceStartup - inferenceStartTime) * 1000f; // ms

                // Update FPS tracking
                UpdatePerformanceMetrics();

                // Update debug info
                if (showDebugInfo && debugText != null)
                {
                    UpdateDebugInfo();
                }

                // Calculate frame time
                lastFrameTime = Time.realtimeSinceStartup - frameStartTime;
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }

        CleanupResources();
    }


    private void UpdatePerformanceMetrics()
    {   
        if(!showFPS)
        {
            performancePanel.SetActive(false);
        }
        else performancePanel.SetActive(true);

        // Update FPS calculation

        fpsFrameCount++;
        fpsAccumulator += lastFrameTime;

        if (fpsAccumulator >= fpsUpdateInterval)
        {
            currentFps = fpsFrameCount / fpsAccumulator;
            fpsFrameCount = 0;
            fpsAccumulator = 0;

            // Update the performance text
            if (performanceText != null)
            {
                performanceText.text = string.Format("FPS: {0:F1}\nInference: {1:F1}ms",
                    currentFps, lastInferenceTime);
            }
        }
    }

    /// <summary>
    /// Determines whether to run full detection or use tracking
    /// </summary>
    private bool ShouldRunDetection()
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

        // If previous detection had low confidence, try detection again
        if (previousScore < scoreThreshold)
            return true;

        // Otherwise use tracking
        return false;
    }

    /// <summary>
    /// Updates debug information text
    /// </summary>
    private void UpdateDebugInfo()
    {
        string mode = isTracking ? "TRACKING" : "DETECTION";
        debugText.text = $"Mode: {mode}\n" +
                         $"Score: {previousScore:F2}\n" +
                         $"Frame: {frameCounter}\n" +
                         $"Track frames: {trackingFrameCount}";
    }

    /// <summary>
    /// Initialize all filtering stages
    /// </summary>
    private void InitializeFilters()
    {
        // Initialize One Euro filters (first stage)
        oneEuroFilters = new OneEuroFilter[k_NumKeypoints];
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            oneEuroFilters[i] = new OneEuroFilter(minCutoff, beta, dCutoff);
        }

        // Initialize Kalman filters (second stage)
        kalmanFilters = new KalmanFilter3D[k_NumKeypoints];
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            kalmanFilters[i] = new KalmanFilter3D(Vector3.zero, kalmanInitialCovariance, kalmanProcessNoise, kalmanMeasurementNoise);
        }

        // Initialize confidence array
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            keypointConfidence[i] = 0f;
        }
    }

    /// <summary>
    /// Update filter parameters from inspector values
    /// </summary>
    private void UpdateFilterParameters()
    {
        // Update One Euro filter parameters
        if (oneEuroFilters != null)
        {
            for (int i = 0; i < k_NumKeypoints; i++)
            {
                oneEuroFilters[i].SetParameters(minCutoff, beta, dCutoff);
            }
        }
        // Kalman filter parameters are updated automatically when the properties change
    }

    private void InitializeModels()
    {
        var poseDetectorModel = ModelLoader.Load(poseDetector);
        // Post-process the detector model if needed
        var graph = new FunctionalGraph();
        var input = graph.AddInput(poseDetectorModel, 0);
        var outputs = Functional.Forward(poseDetectorModel, input);
        var boxes = outputs[0]; // (1, 2254, 12)
        var scores = outputs[1]; // (1, 2254, 1)
        var idx_scores_boxes = BlazeUtilsHand.ArgMaxFiltering(boxes, scores);
        poseDetectorModel = graph.Compile(idx_scores_boxes.Item1, idx_scores_boxes.Item2, idx_scores_boxes.Item3);
        m_PoseDetectorWorker = new Worker(poseDetectorModel, BackendType.GPUCompute);

        var poseLandmarkerModel = ModelLoader.Load(poseLandmarker);
        m_PoseLandmarkerWorker = new Worker(poseLandmarkerModel, BackendType.GPUCompute);

        m_DetectorInput = new Tensor<float>(new TensorShape(1, detectorInputSize, detectorInputSize, 3));
        m_LandmarkerInput = new Tensor<float>(new TensorShape(1, landmarkerInputSize, landmarkerInputSize, 3));

        foreach (var outputName in poseDetectorModel.outputs)
        {
            Debug.Log(outputName.name);
        }
    }

    private void CleanupResources()
    {
        m_PoseDetectorWorker.Dispose();
        m_PoseLandmarkerWorker.Dispose();
        m_DetectorInput.Dispose();
        m_LandmarkerInput.Dispose();
    }

    Vector3 ImageToWorld(Vector2 position)
    {
        return (position - 0.5f * new Vector2(m_TextureWidth, m_TextureHeight)) / m_TextureHeight;
    }

    Matrix4x4 CalculateTransformMatrix(float2x3 M2, int landmarkerInputSize)
    {
        float a = M2.c0.x;
        float c = M2.c0.y;
        float b = M2.c1.x;
        float d = M2.c1.y;
        float tx = M2.c2.x;
        float ty = M2.c2.y;

        float det = a * d - b * c;
        float invDet = 1.0f / det;

        Matrix4x4 invM4 = new Matrix4x4();
        invM4.SetColumn(0, new Vector4(d * invDet, -c * invDet, 0f, 0f));
        invM4.SetColumn(1, new Vector4(-b * invDet, a * invDet, 0f, 0f));
        invM4.SetColumn(2, new Vector4(0f, 0f, 1f, 0f));
        invM4.SetColumn(3, new Vector4((b * ty - d * tx) * invDet, (c * tx - a * ty) * invDet, 0f, 1f));

        // M4 is the inverse of M2, maps from webcam pixel coordinates to segmentation pixel coordinates
        // Adjust to work in UV space (0-1) instead of pixel space
        float webcamWidth = (float)m_TextureWidth;
        float webcamHeight = (float)m_TextureHeight;
        float segSize = (float)landmarkerInputSize;

        // Create matrices to convert between pixel space and UV space
        Matrix4x4 webcamUVtoPixels = Matrix4x4.identity;
        webcamUVtoPixels.m00 = webcamWidth;
        webcamUVtoPixels.m11 = webcamHeight;

        Matrix4x4 segPixelsToUV = Matrix4x4.identity;
        segPixelsToUV.m00 = 1.0f / segSize;
        segPixelsToUV.m11 = 1.0f / segSize;

        Matrix4x4 flipMatrix = Matrix4x4.identity;
        flipMatrix.m11 = -1.0f;
        flipMatrix.m13 = 1.0f;

        // Create transformation that maps from webcam UV to segmentation UV
        // Order: webcam UV -> webcam pixels -> segmentation pixels -> segmentation UV
        return flipMatrix * segPixelsToUV * invM4 * webcamUVtoPixels;
    }

    /// <summary>
    /// Full pose detection mode
    /// </summary>
    async Awaitable DetectPose(Texture texture)
    {
        m_TextureWidth = texture.width;
        m_TextureHeight = texture.height;

        var size = Mathf.Max(texture.width, texture.height);
        var scale = size / (float)detectorInputSize;
        var M = BlazeUtilsHand.mul(
            BlazeUtilsHand.TranslationMatrix(0.5f * (new float2(texture.width, texture.height) + new float2(-size, size))),
            BlazeUtilsHand.ScaleMatrix(new float2(scale, -scale))
        );
        BlazeUtilsHand.SampleImageAffine(texture, m_DetectorInput, M);

        m_PoseDetectorWorker.Schedule(m_DetectorInput);

        var outputIdxAwaitable = (m_PoseDetectorWorker.PeekOutput(0) as Tensor<int>).ReadbackAndCloneAsync();
        var outputScoreAwaitable = (m_PoseDetectorWorker.PeekOutput(1) as Tensor<float>).ReadbackAndCloneAsync();
        var outputBoxAwaitable = (m_PoseDetectorWorker.PeekOutput(2) as Tensor<float>).ReadbackAndCloneAsync();

        using var outputIdx = await outputIdxAwaitable;
        using var outputScore = await outputScoreAwaitable;
        using var outputBox = await outputBoxAwaitable;

        var currentScore = outputScore[0];
        previousScore = currentScore;
        var scorePassesThreshold = currentScore >= scoreThreshold;

        // Update detection history
        UpdateDetectionHistory(currentScore);

        // Reset tracking counter when running detection
        trackingFrameCount = 0;

        // Handle UI feedback based on detection history
        if (ShouldShowFeedback())
        {
            avatar.SetActive(false);
            outputImage.SetActive(false);
            inputImage.SetActive(true);
            ShowDetectionFeedback(true);
            posePreview.SetActive(false);
            return;
        }
        else
        {
            avatar.SetActive(avatarAvailable);
            outputImage.SetActive(true);
            inputImage.SetActive(false);
            posePreview.SetActive(!avatarAvailable);
            // Hide feedback when detection is successful
            if (isFeedbackActive && ShouldHideFeedback())
            {
                ShowDetectionFeedback(false);
            }
        }

        var idx = outputIdx[0];
        var anchorPosition = detectorInputSize * new float2(m_Anchors[idx, 0], m_Anchors[idx, 1]);

        var face_ImageSpace = BlazeUtilsHand.mul(M, anchorPosition + new float2(outputBox[0, 0, 0], outputBox[0, 0, 1]));
        var faceTopRight_ImageSpace = BlazeUtilsHand.mul(M, anchorPosition + new float2(outputBox[0, 0, 0] + 0.5f * outputBox[0, 0, 2], outputBox[0, 0, 1] + 0.5f * outputBox[0, 0, 3]));

        var kp1_ImageSpace = BlazeUtilsHand.mul(M, anchorPosition + new float2(outputBox[0, 0, 4 + 2 * 0 + 0], outputBox[0, 0, 4 + 2 * 0 + 1]));
        var kp2_ImageSpace = BlazeUtilsHand.mul(M, anchorPosition + new float2(outputBox[0, 0, 4 + 2 * 1 + 0], outputBox[0, 0, 4 + 2 * 1 + 1]));
        var delta_ImageSpace = kp2_ImageSpace - kp1_ImageSpace;
        var dscale = 1.25f;
        var radius = dscale * math.length(delta_ImageSpace);
        var theta = math.atan2(delta_ImageSpace.y, delta_ImageSpace.x);
        var origin2 = new float2(0.5f * landmarkerInputSize, 0.5f * landmarkerInputSize);
        var scale2 = radius / (0.5f * landmarkerInputSize);
        var M2 = BlazeUtilsHand.mul(
            BlazeUtilsHand.mul(
                BlazeUtilsHand.mul(
                    BlazeUtilsHand.TranslationMatrix(kp1_ImageSpace),
                    BlazeUtilsHand.ScaleMatrix(new float2(scale2, -scale2))
                ),
                BlazeUtilsHand.RotationMatrix(0.5f * Mathf.PI - theta)
            ),
            BlazeUtilsHand.TranslationMatrix(-origin2)
        );

        // Store transform for tracking
        previousTransform = M2;

        BlazeUtilsHand.SampleImageAffine(texture, m_LandmarkerInput, M2);

        // Calculate transform matrix for segmentation
        Matrix4x4 finalTransform = CalculateTransformMatrix(M2, landmarkerInputSize);

        m_PoseLandmarkerWorker.Schedule(m_LandmarkerInput);

        var landmarksAwaitable = (m_PoseLandmarkerWorker.PeekOutput("Identity") as Tensor<float>).ReadbackAndCloneAsync();
        using var landmarks = await landmarksAwaitable; // (1,195)

        var segDataAwaitable = (m_PoseLandmarkerWorker.PeekOutput("Identity_2") as Tensor<float>).ReadbackAndCloneAsync();
        using var segData = await segDataAwaitable; // This returns a Tensor<float> with shape [1, 256, 256, 1]

        // Process segmentation in the separate renderer if available
        if (segmentationRenderer != null)
        {
            segmentationRenderer.UpdateSegmentation(segData, webcamInput.GetFeedFrameCopy(), finalTransform);
        }

        // Update keypoints with multi-stage filtering
        UpdateKeypoints(landmarks, M2);

        // Update filter parameters from inspector
        UpdateFilterParameters();
    }

    /// <summary>
    /// Tracking mode - uses ROI from previous detection for faster processing
    /// </summary>
    async Awaitable TrackPose(Texture texture)
    {
        // Increment tracking frame counter
        trackingFrameCount++;

        try
        {
            // Use the transform from previous detection
            var M2 = previousTransform;

            // Sample the image using the previous transform
            BlazeUtilsHand.SampleImageAffine(texture, m_LandmarkerInput, M2);

            // Calculate transform matrix for segmentation
            Matrix4x4 finalTransform = CalculateTransformMatrix(M2, landmarkerInputSize);

            m_PoseLandmarkerWorker.Schedule(m_LandmarkerInput);

            var landmarksAwaitable = (m_PoseLandmarkerWorker.PeekOutput("Identity") as Tensor<float>).ReadbackAndCloneAsync();
            using var landmarks = await landmarksAwaitable; // (1,195)

            var segDataAwaitable = (m_PoseLandmarkerWorker.PeekOutput("Identity_2") as Tensor<float>).ReadbackAndCloneAsync();
            using var segData = await segDataAwaitable; // This returns a Tensor<float> with shape [1, 256, 256, 1]

            // In tracking mode, we use a different threshold
            float trackingConfidence = CalculateTrackingConfidence(landmarks);
            previousScore = trackingConfidence;

            UpdateDetectionHistory(trackingConfidence);

            // If confidence is too low, we'll trigger a full detection on the next frame
            if (trackingConfidence < trackingScoreThreshold)
            {
                Debug.Log("Full track");
                // Check if we should show feedback based on detection history
                bool shouldShowFeedback = ShouldShowFeedback();

                if (enableDetectionFeedback && shouldShowFeedback && !isFeedbackActive)
                {
                    Debug.Log($"Showing feedback (tracking) - Failed detections: {failedDetectionsCount}/{detectionScoreHistory.Count} ({(failedDetectionsCount * 100f / detectionScoreHistory.Count):F1}%)");
                    avatar.SetActive(false);
                    outputImage.SetActive(false);
                    inputImage.SetActive(true);
                    ShowDetectionFeedback(true);
                }

                trackingFrameCount = maxTrackingFrames; // Force detection next frame

                // Update the background when confidence is low - it may mean the person left the frame
                if (segmentationRenderer != null)
                {
                    segmentationRenderer.UpdateEntireBackground(webcamInput.GetFeedFrameCopy());
                }
                return;
            }
            else
            {
                avatar.SetActive(avatarAvailable);
                outputImage.SetActive(true);
                inputImage.SetActive(false);

                // Hide feedback when tracking is successful
                if (isFeedbackActive && ShouldHideFeedback())
                {
                    ShowDetectionFeedback(false);
                }
            }

            // Update keypoints with multi-stage filtering
            UpdateKeypoints(landmarks, M2);

            // Update filter parameters from inspector
            UpdateFilterParameters();
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Error in TrackPose: {e.Message}");

            // Ensure webcam feed continues even on error
            if (segmentationRenderer != null)
            {
                segmentationRenderer.UpdateRawWebcamFeed(webcamInput.GetFeedFrameCopy());
            }

            // Force detection on next frame
            trackingFrameCount = maxTrackingFrames;
        }
    }

    /// <summary>
    /// Calculate overall confidence for tracking mode
    /// </summary>
    private float CalculateTrackingConfidence(Tensor<float> landmarks)
    {
        float totalConfidence = 0f;
        int count = 0;

        // We focus on key points that matter most for tracking
        int[] keyIndices = { 0, 11, 12, 23, 24 }; // Nose, shoulders, hips

        foreach (int i in keyIndices)
        {
            if (i < k_NumKeypoints)
            {
                float visibility = landmarks[5 * i + 3];
                float presence = landmarks[5 * i + 4];

                // Use sigmoid activation for confidence
                float rawScore = (visibility + presence) * 0.5f;
                float keyConfidence = 1f / (1f + Mathf.Exp(-6f * (rawScore - 0.5f)));

                totalConfidence += keyConfidence;
                count++;
            }
        }

        return count > 0 ? totalConfidence / count : 0f;
    }

    // Add these new methods to handle detection history
    private void UpdateDetectionHistory(float score)
    {
        // Add the new score to the history queue
        if (detectionScoreHistory.Count >= detectionHistoryLength)
        {
            // Remove the oldest score and update the failed count if needed
            float oldestScore = detectionScoreHistory.Dequeue();
            if (oldestScore < detectionFailThreshold)
            {
                failedDetectionsCount--;
            }
        }

        // Add the new score to the queue
        detectionScoreHistory.Enqueue(score);

        // Update the failed count if needed
        if (score < detectionFailThreshold)
        {
            failedDetectionsCount++;
        }

        // Log detection history stats if debug enabled
        if (showDebugInfo)
        {
            float failPercentage = detectionScoreHistory.Count > 0 ?
                (failedDetectionsCount * 100f / detectionScoreHistory.Count) : 0;
        }
    }

    private bool ShouldShowFeedback()
    {
        // Don't show feedback if we don't have enough history
        if (detectionScoreHistory.Count < detectionHistoryLength * 0.5f)
        {
            return false;
        }

        // Calculate the percentage of failed detections
        float failPercentage = (failedDetectionsCount * 100f) / detectionScoreHistory.Count;

        // Show feedback if the fail percentage exceeds the threshold
        return failPercentage >= failPercentageThreshold;
    }

    private bool ShouldHideFeedback()
    {
        // Calculate the percentage of failed detections
        float failPercentage = detectionScoreHistory.Count > 0 ?
            (failedDetectionsCount * 100f / detectionScoreHistory.Count) : 0;

        // Hide feedback if the fail percentage drops below the threshold with some hysteresis
        return failPercentage < (failPercentageThreshold - 10f);
    }

    private void UpdateKeypoints(Tensor<float> landmarks, float2x3 M2)
    {
        // For each keypoint, apply multi-stage filtering
        for (var i = 0; i < k_NumKeypoints; i++)
        {
            // Each landmark is represented by 5 numbers: x, y, z, visibility, presence
            var position_ImageSpace = BlazeUtilsHand.mul(M2, new float2(landmarks[5 * i + 0], landmarks[5 * i + 1]));
            var visibility = landmarks[5 * i + 3];
            var presence = landmarks[5 * i + 4];

            // Update confidence value using sigmoid activation
            float rawScore = (visibility + presence) * 0.5f;
            keypointConfidence[i] = 1f / (1f + Mathf.Exp(-6f * (rawScore - 0.5f)));

            // z is in a unit cube centered on hips. Convert it to world space
            Vector3 position_WorldSpace = ImageToWorld(position_ImageSpace) + new Vector3(0, 0, landmarks[5 * i + 2] / m_TextureHeight);

            position_WorldSpace.x *= -1;
            position_WorldSpace.z *= -1;

            // Skip filtering for very low confidence points (just use previous value)
            float confidenceThreshold = 0.2f;
            if (keypointConfidence[i] < confidenceThreshold)
            {
                // Keep the existing filtered position
                continue;
            }

            // Apply multi-stage filtering
            Vector3 filteredPosition = position_WorldSpace;

            // Stage 1: Apply One Euro filter (if enabled)
            if (useOneEuroFilter)
            {
                filteredPosition = oneEuroFilters[i].Filter(filteredPosition);
            }

            // Stage 2: Apply Kalman filter (if enabled)
            if (useKalmanFilter)
            {
                filteredPosition = kalmanFilters[i].Update(filteredPosition);
            }

            keypoints[i] = filteredPosition;
            posePreview.SetKeypoint(i, keypointConfidence[i] > confidenceThreshold, filteredPosition);
        }
    }

    // Add this method to the PoseDetection class
    private void ShowDetectionFeedback(bool show)
    {
        if (detectionFeedbackPanel == null || detectionFeedbackText == null)
            return;

        // Set the feedback state
        isFeedbackActive = show;
        // Ensure panel is visible and in front
        detectionFeedbackPanel.gameObject.SetActive(true);
        detectionFeedbackPanel.transform.SetAsLastSibling();
        // Set feedback text
        if (show)
        {
            detectionFeedbackText.text = "Human not detected\nMake sure your whole body is captured";
        }

        // Start coroutine to animate the panel fade
        StartCoroutine(AnimateFeedbackPanel(show));
    }

    // Add this coroutine to animate the fade
    private IEnumerator AnimateFeedbackPanel(bool fadeIn)
    {
        float startAlpha = detectionFeedbackPanel.alpha;
        float targetAlpha = fadeIn ? 0.8f : 0f; // Dim to 80% opacity when showing
        float elapsedTime = 0f;

        while (elapsedTime < fadeDuration)
        {
            elapsedTime += Time.deltaTime;
            float t = Mathf.Clamp01(elapsedTime / fadeDuration);
            detectionFeedbackPanel.alpha = Mathf.Lerp(startAlpha, targetAlpha, t);
            yield return null;
        }

        detectionFeedbackPanel.alpha = targetAlpha;

        // Enable/disable interaction based on visibility
        detectionFeedbackPanel.interactable = fadeIn;
        detectionFeedbackPanel.blocksRaycasts = fadeIn;
    }

    void OnDestroy()
    {
        // Cancel any pending operations
        if (cancellationTokenSource != null)
        {
            cancellationTokenSource.Cancel();
            cancellationTokenSource.Dispose();
            cancellationTokenSource = null;
        }

        // Clean up other resources
        if (m_DetectAwaitable != null)
        {
            m_DetectAwaitable.Cancel();
        }
        if (m_PoseDetectorWorker != null)
        {
            m_PoseDetectorWorker.Dispose();
        }
        if (m_PoseLandmarkerWorker != null)
        {
            m_PoseLandmarkerWorker.Dispose();
        }
    }
}