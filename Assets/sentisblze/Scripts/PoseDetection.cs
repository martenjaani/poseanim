using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Mathematics;
using Unity.Sentis;
using UnityEngine;
using UnityEngine.UI;

public class PoseDetection : MonoBehaviour
{
    public WebcamInput webcamInput;
    public ModelAsset poseDetector;
    public ModelAsset poseLandmarker;
    public TextAsset anchorsCSV;
    public float scoreThreshold = 0.75f;
    public SegmentationRenderer segmentationRenderer;

    // Spheres to visualize each keypoint
    public GameObject spherePrefab;
    private const int k_NumKeypoints = 33;
    private readonly List<GameObject> jointSpheres = new List<GameObject>();

    const int k_NumAnchors = 2254;
    float[,] m_Anchors;

    const int detectorInputSize = 224;
    const int landmarkerInputSize = 256;

    Worker m_PoseDetectorWorker;
    Worker m_PoseLandmarkerWorker;
    Tensor<float> m_DetectorInput;
    Tensor<float> m_LandmarkerInput;
    Awaitable m_DetectAwaitable;

    float m_TextureWidth;
    float m_TextureHeight;
    private Texture2D imageTexture;

    // Multi-stage filtering settings
    [Header("Filtering Settings")]
    [Tooltip("Enable/disable velocity threshold filtering")]
    public bool useVelocityFilter = true;
    [Tooltip("Enable/disable One Euro filtering")]
    public bool useOneEuroFilter = true;
    [Tooltip("Enable/disable Kalman filtering")]
    public bool useKalmanFilter = true;

    // Velocity filter settings
    [Header("Velocity Filter Settings")]
    [Tooltip("Maximum velocity allowed for keypoints (units/sec)")]
    public float maxVelocity = 5.0f;
    [Tooltip("Smoothing factor for velocity changes")]
    [Range(0, 0.99f)]
    public float velocitySmoothing = 0.5f;

    // One Euro filter settings
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

    // Kalman Filter Settings
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

    [Header("Output Settings")]
    [Tooltip("Scale factor for displayed keypoints")]
    public float scaleFactor = 1.0f;

    // Filters
    private VelocityThresholdFilter[] velocityFilters;
    private OneEuroFilter[] oneEuroFilters;
    private KalmanFilter3D[] kalmanFilters;

    // Output keypoint positions
    public Vector3[] keypoints = new Vector3[k_NumKeypoints];

    // Confidence values for each keypoint
    private float[] keypointConfidence = new float[k_NumKeypoints];

    // Tracking state
    private bool isTracking = false;
    private int frameCounter = 0;
    private int trackingFrameCount = 0;
    private Rect previousROI;
    private float2x3 previousTransform;
    private float previousScore = 0;

    // Debug info
    [Header("Debug")]
    public bool showDebugInfo = false;
    public Text debugText;

    public async void Start()
    {
        // Create a sphere for each keypoint
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            GameObject sphere = Instantiate(spherePrefab, this.transform);
            sphere.name = "JointSphere_" + i;
            jointSpheres.Add(sphere);
        }

        // Initialize multi-stage filters
        InitializeFilters();

        m_Anchors = BlazeUtils.LoadAnchors(anchorsCSV.text, k_NumAnchors);

        InitializeModels();

        while (webcamInput.GetFrame() == null)
        {
            Debug.Log("Waiting for webcam initialization...");
            await Task.Delay(100);
        }

        imageTexture = webcamInput.GetFrame();
        m_TextureWidth = imageTexture.width;
        m_TextureHeight = imageTexture.height;

        while (webcamInput.GetFeedFrameCopy() == null)
        {
            Debug.Log("Waiting for webcam initialization...");
            await Task.Delay(100);
        }

        if (segmentationRenderer != null)
        {
            segmentationRenderer.Initialize(landmarkerInputSize, webcamInput.GetFeedFrameCopy());
        }

        // Detection loop
        while (true)
        {
            try
            {
                frameCounter++;

                // Decide whether to use detection or tracking mode
                bool shouldRunDetection = ShouldRunDetection();

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

                // Update debug info
                if (showDebugInfo && debugText != null)
                {
                    UpdateDebugInfo();
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }

        CleanupResources();
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
        // Initialize velocity filters (first stage)
        velocityFilters = new VelocityThresholdFilter[k_NumKeypoints];
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            velocityFilters[i] = new VelocityThresholdFilter(maxVelocity, velocitySmoothing);
        }

        // Initialize One Euro filters (second stage)
        oneEuroFilters = new OneEuroFilter[k_NumKeypoints];
        for (int i = 0; i < k_NumKeypoints; i++)
        {
            oneEuroFilters[i] = new OneEuroFilter(minCutoff, beta, dCutoff);
        }

        // Initialize Kalman filters (third stage)
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
        // Update velocity filter parameters
        if (velocityFilters != null)
        {
            for (int i = 0; i < k_NumKeypoints; i++)
            {
                velocityFilters[i].SetMaxVelocity(maxVelocity);
                velocityFilters[i].SetVelocitySmoothing(velocitySmoothing);
            }
        }

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
        var idx_scores_boxes = BlazeUtils.ArgMaxFiltering(boxes, scores);
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
        var M = BlazeUtils.mul(
            BlazeUtils.TranslationMatrix(0.5f * (new float2(texture.width, texture.height) + new float2(-size, size))),
            BlazeUtils.ScaleMatrix(new float2(scale, -scale))
        );
        BlazeUtils.SampleImageAffine(texture, m_DetectorInput, M);

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

        // Reset tracking counter when running detection
        trackingFrameCount = 0;

        if (!scorePassesThreshold)
            return;

        var idx = outputIdx[0];
        var anchorPosition = detectorInputSize * new float2(m_Anchors[idx, 0], m_Anchors[idx, 1]);

        var face_ImageSpace = BlazeUtils.mul(M, anchorPosition + new float2(outputBox[0, 0, 0], outputBox[0, 0, 1]));
        var faceTopRight_ImageSpace = BlazeUtils.mul(M, anchorPosition + new float2(outputBox[0, 0, 0] + 0.5f * outputBox[0, 0, 2], outputBox[0, 0, 1] + 0.5f * outputBox[0, 0, 3]));

        // Store ROI for tracking mode
        float roiWidth = Mathf.Abs(faceTopRight_ImageSpace.x - face_ImageSpace.x) * roiExpansionFactor;
        float roiHeight = Mathf.Abs(faceTopRight_ImageSpace.y - face_ImageSpace.y) * roiExpansionFactor;
        previousROI = new Rect(
            Mathf.Max(0, face_ImageSpace.x - roiWidth * 0.5f),
            Mathf.Max(0, face_ImageSpace.y - roiHeight * 0.5f),
            Mathf.Min(m_TextureWidth, roiWidth),
            Mathf.Min(m_TextureHeight, roiHeight)
        );

        var kp1_ImageSpace = BlazeUtils.mul(M, anchorPosition + new float2(outputBox[0, 0, 4 + 2 * 0 + 0], outputBox[0, 0, 4 + 2 * 0 + 1]));
        var kp2_ImageSpace = BlazeUtils.mul(M, anchorPosition + new float2(outputBox[0, 0, 4 + 2 * 1 + 0], outputBox[0, 0, 4 + 2 * 1 + 1]));
        var delta_ImageSpace = kp2_ImageSpace - kp1_ImageSpace;
        var dscale = 1.25f;
        var radius = dscale * math.length(delta_ImageSpace);
        var theta = math.atan2(delta_ImageSpace.y, delta_ImageSpace.x);
        var origin2 = new float2(0.5f * landmarkerInputSize, 0.5f * landmarkerInputSize);
        var scale2 = radius / (0.5f * landmarkerInputSize);
        var M2 = BlazeUtils.mul(
            BlazeUtils.mul(
                BlazeUtils.mul(
                    BlazeUtils.TranslationMatrix(kp1_ImageSpace),
                    BlazeUtils.ScaleMatrix(new float2(scale2, -scale2))
                ),
                BlazeUtils.RotationMatrix(0.5f * Mathf.PI - theta)
            ),
            BlazeUtils.TranslationMatrix(-origin2)
        );

        // Store transform for tracking
        previousTransform = M2;

        BlazeUtils.SampleImageAffine(texture, m_LandmarkerInput, M2);

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
            BlazeUtils.SampleImageAffine(texture, m_LandmarkerInput, M2);

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

            // If confidence is too low, we'll trigger a full detection on the next frame
            if (trackingConfidence < trackingScoreThreshold)
            {
                trackingFrameCount = maxTrackingFrames; // Force detection next frame

                // Update the background when confidence is low - it may mean the person left the frame
                if (segmentationRenderer != null)
                {
                    segmentationRenderer.UpdateEntireBackground(webcamInput.GetFeedFrameCopy());
                }
                return;
            }

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

    private void UpdateKeypoints(Tensor<float> landmarks, float2x3 M2)
    {
        // For each keypoint, apply multi-stage filtering
        for (var i = 0; i < k_NumKeypoints; i++)
        {
            // Each landmark is represented by 5 numbers: x, y, z, visibility, presence
            var position_ImageSpace = BlazeUtils.mul(M2, new float2(landmarks[5 * i + 0], landmarks[5 * i + 1]));
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

            // Stage 1: Apply velocity filter (if enabled)
            if (useVelocityFilter)
            {
                filteredPosition = velocityFilters[i].Filter(filteredPosition);
            }

            // Stage 2: Apply One Euro filter (if enabled)
            if (useOneEuroFilter)
            {
                filteredPosition = oneEuroFilters[i].Filter(filteredPosition);
            }

            // Stage 3: Apply Kalman filter (if enabled)
            if (useKalmanFilter)
            {
                filteredPosition = kalmanFilters[i].Update(filteredPosition);
            }

            // Set the sphere's position to the filtered result
            jointSpheres[i].transform.localPosition = filteredPosition * scaleFactor;
            keypoints[i] = filteredPosition;

            // Adjust sphere size based on confidence
            float minSize = 0.01f;
            float maxSize = 0.1f;
            float normalizedConfidence = Mathf.Clamp01(keypointConfidence[i]);
            float adjustedSize = Mathf.Lerp(minSize, maxSize, normalizedConfidence);
            jointSpheres[i].transform.localScale = Vector3.one * adjustedSize;
        }
    }

    void OnDestroy()
    {
        m_DetectAwaitable.Cancel();
    }

    // Optional: Add a method to visualize the ROI for debugging
    void OnDrawGizmos()
    {
        if (!Application.isPlaying || !isTracking)
            return;

        // Draw ROI rectangle in scene view
        Gizmos.color = Color.green;
        Vector3 center = new Vector3(previousROI.center.x, previousROI.center.y, 0);
        Vector3 size = new Vector3(previousROI.width, previousROI.height, 0.01f);
        Gizmos.DrawWireCube(center, size);
    }
}