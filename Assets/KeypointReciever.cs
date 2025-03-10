using NativeWebSocket;
using System;
using System.Collections.Generic;
using UnityEngine;

public class KeypointReceiver : MonoBehaviour
{
    // WebSocket configuration
    private WebSocket websocket;
    private string serverUrl = "ws://127.0.0.1:8765";

    // Rendering configuration
    [Header("Rendering")]
    public Mesh sphereMesh;
    public Material sphereMaterial;
    public float sphereSize = 0.05f;
    public Color sphereColor = Color.blue;

    // Coordinate transformation
    [Header("Transform")]
    public float scaleFactor = 1.0f;
    public Vector3 offset = new Vector3(0f, 0f, 5f);

    // Debug
    [Header("Debug")]
    public bool showDebugInfo = true;

    // Keypoint data
    private const int NUM_JOINTS = 133;
    private Vector3[] keypointPositions = new Vector3[NUM_JOINTS];
    private Matrix4x4[] matrices = new Matrix4x4[NUM_JOINTS];
    private MaterialPropertyBlock propertyBlock;

    // Performance metrics
    private float lastFrameTime;
    private int framesReceived;
    private float updateRate;
    private int lastFrameId;

    // Connection status
    private bool isConnected = false;

    async void Start()
    {
        // Initialize material property block for instancing
        propertyBlock = new MaterialPropertyBlock();
        propertyBlock.SetColor("_Color", sphereColor);

        // Initialize matrices with default transform
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            matrices[i] = Matrix4x4.TRS(
                offset,
                Quaternion.identity,
                Vector3.one * sphereSize
            );
        }

        // Connect to WebSocket server
        Debug.Log($"Connecting to WebSocket server at {serverUrl}...");
        websocket = new WebSocket(serverUrl);

        websocket.OnOpen += () => {
            Debug.Log("WebSocket connection established");
            isConnected = true;
        };

        websocket.OnClose += (e) => {
            Debug.Log($"WebSocket connection closed: {e}");
            isConnected = false;
        };

        websocket.OnError += (e) => {
            Debug.LogError($"WebSocket error: {e}");
        };

        websocket.OnMessage += (bytes) => {
            ProcessKeypoints(bytes);
        };

        await websocket.Connect();
    }

    void ProcessKeypoints(byte[] bytes)
    {
        try
        {
            // Ensure we have enough data for the header
            if (bytes.Length < 8)
            {
                Debug.LogError($"Message too short: {bytes.Length} bytes");
                return;
            }

            // Parse the header
            int frameId = BitConverter.ToInt32(bytes, 0);
            int numJoints = BitConverter.ToInt32(bytes, 4);

            // Validate expected data size
            int expectedSize = 8 + (numJoints * 12); // Header + (joints * 3 floats * 4 bytes)
            if (bytes.Length != expectedSize)
            {
                Debug.LogError($"Data size mismatch: got {bytes.Length}, expected {expectedSize}");
                return;
            }

            // Track frame rate
            framesReceived++;
            if (Time.time - lastFrameTime > 1.0f)
            {
                updateRate = framesReceived / (Time.time - lastFrameTime);
                int frameGap = frameId - lastFrameId;

                if (showDebugInfo)
                {
                    Debug.Log($"FPS: {updateRate:F1}, Frame: {frameId}, Gap: {frameGap}");
                }

                framesReceived = 0;
                lastFrameTime = Time.time;
                lastFrameId = frameId;
            }

            // Process all joints
            for (int i = 0; i < numJoints; i++)
            {
                int dataOffset = 8 + (i * 12);

                // Read the 3D position
                float x = BitConverter.ToSingle(bytes, dataOffset);
                float y = BitConverter.ToSingle(bytes, dataOffset + 4);
                float z = BitConverter.ToSingle(bytes, dataOffset + 8);

                // Convert to Unity coordinate system
                Vector3 position = new Vector3(
                    -x * scaleFactor,
                    -y * scaleFactor,
                    -z * scaleFactor
                ) + offset;

                // Store the position
                keypointPositions[i] = position;

                // Update the transformation matrix for instanced rendering
                matrices[i] = Matrix4x4.TRS(
                    position,
                    Quaternion.identity,
                    Vector3.one * sphereSize
                );
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error processing keypoints: {e}");
        }
    }

    void Update()
    {
        // Process WebSocket messages
        if (websocket != null)
        {
#if !UNITY_WEBGL || UNITY_EDITOR
            websocket.DispatchMessageQueue();
#endif
        }

        // Render all keypoints using GPU instancing
        if (sphereMesh != null && sphereMaterial != null)
        {
            Graphics.DrawMeshInstanced(
                sphereMesh,
                0,
                sphereMaterial,
                matrices,
                NUM_JOINTS,
                propertyBlock
            );
        }
    }

    void OnGUI()
    {
        if (showDebugInfo)
        {
            GUI.Label(new Rect(10, 10, 300, 20), $"Connection: {(isConnected ? "Connected" : "Disconnected")}");
            GUI.Label(new Rect(10, 30, 300, 20), $"Update rate: {updateRate:F1} FPS");
            GUI.Label(new Rect(10, 50, 300, 20), $"Keypoints: {NUM_JOINTS}");
        }
    }

    async void OnApplicationQuit()
    {
        if (websocket != null && websocket.State == WebSocketState.Open)
        {
            await websocket.Close();
        }
    }

    // Helper function to reconnect if the connection is lost
    public async void Reconnect()
    {
        if (websocket != null && websocket.State != WebSocketState.Open)
        {
            await websocket.Connect();
        }
    }
}