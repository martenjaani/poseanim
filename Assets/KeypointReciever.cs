using NativeWebSocket;
using Newtonsoft.Json.Linq;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

public class KeypointReceiver : MonoBehaviour
{
    public GameObject spherePrefab;
    private List<GameObject> jointSpheres = new List<GameObject>();
    private WebSocket websocket;
    public float scaleFactor = 1.0f;
    public Vector3 offset = new Vector3(0f, 0f, 5f);
    private const int NUM_JOINTS = 133;
    public Vector3[] keypoints = new Vector3[NUM_JOINTS];

    private VelocityThresholdFilter[] velocityFilters;
    private OneEuroFilter[] oneEuroFilters;
    private KalmanFilter3D[] kalmanFilters;

    public bool sphereVisible = false;

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

    async void Start()
    {
        InitializeFilters();


        // Instantiate spheres for the joints.
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            GameObject sphere = Instantiate(spherePrefab, this.transform);
            sphere.name = "JointSphere_" + i;
            jointSpheres.Add(sphere);
        }

        // Set up the WebSocket.
        websocket = new WebSocket("ws://127.0.0.1:8765");

        // Log when the connection opens.
        websocket.OnOpen += () =>
        {
            Debug.Log("WebSocket connection open!");
        };

        // Handle incoming messages.
        websocket.OnMessage += (bytes) =>
        {
            string msg = Encoding.UTF8.GetString(bytes);
            JObject jsonObj = JObject.Parse(msg);
            JArray persons = (JArray)jsonObj["keypoints_3d"];

            if (persons != null && persons.Count > 0)
            {
                JArray joints = (JArray)persons;
                int count = Mathf.Min(joints.Count, NUM_JOINTS);

                Debug.Log(joints);

                for (int i = 0; i < count; i++)
                {
                    JArray joint = (JArray)joints[i];
                    float x = -joint[0].Value<float>();
                    float y = -joint[1].Value<float>();
                    float z = -joint[2].Value<float>();

                    Vector3 pos = new Vector3(x * scaleFactor, y * scaleFactor, z * scaleFactor);

                    // Apply multi-stage filtering
                    Vector3 filteredPosition = pos;

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

                    if (!sphereVisible)
                    {
                        jointSpheres[i].SetActive(false);
                    }
                    else jointSpheres[i].SetActive(true);

                    keypoints[i] = filteredPosition;

                    jointSpheres[i].transform.localPosition = filteredPosition;
                }
            }
            else
            {
                Debug.Log("No keypoints received for this frame.");
            }
        };

        // Connect the WebSocket.
        await websocket.Connect();
    }

    private void InitializeFilters()
    {
        // Initialize velocity filters (first stage)
        velocityFilters = new VelocityThresholdFilter[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            velocityFilters[i] = new VelocityThresholdFilter(maxVelocity, velocitySmoothing);
        }

        // Initialize One Euro filters (second stage)
        oneEuroFilters = new OneEuroFilter[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            oneEuroFilters[i] = new OneEuroFilter(minCutoff, beta, dCutoff);
        }

        // Initialize Kalman filters (third stage)
        kalmanFilters = new KalmanFilter3D[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            kalmanFilters[i] = new KalmanFilter3D(Vector3.zero, kalmanInitialCovariance, kalmanProcessNoise, kalmanMeasurementNoise);
        }


    }

    void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        if (websocket != null)
        {
            websocket.DispatchMessageQueue();
        }
#endif
    }

    private async void OnApplicationQuit()
    {
        if (websocket != null)
        {
            await websocket.Close();
        }
    }


}
