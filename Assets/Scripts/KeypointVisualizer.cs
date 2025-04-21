using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class KeypointVisualizer : MonoBehaviour
{
    [Header("References")]
    public PoseDetection poseDetection;
    public RawImage outputImage;

    [Header("Appearance")]
    public Color pointColor = new Color(0f, 1f, 0f, 1f);
    public Color lineColor = new Color(0f, 1f, 0f, 0.7f);
    public float pointSize = 8f;
    public float lineWidth = 4f;

    // Texture for drawing
    private Texture2D drawTexture;
    private bool textureInitialized = false;

    // Connection definitions - MediaPipe pose connections
    private readonly List<(int, int)> connections = new List<(int, int)>
    {
        // Face
        (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8),
        
        // Torso
        (9, 10), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
        
        // Left arm
        (11, 23), (23, 25), (25, 27), (27, 29), (27, 31),
        
        // Right arm  
        (12, 24), (24, 26), (26, 28), (28, 30), (28, 32),
        
        // Left leg
        (23, 24), (24, 26), (26, 28), (28, 30), (28, 32),
        
        // Right leg
        (11, 13), (13, 15), (15, 17), (15, 19), (15, 21),
        (12, 14), (14, 16), (16, 18), (16, 20), (16, 22)
    };

    void Start()
    {
        InitializeTexture();
    }

    async void InitializeTexture()
    {
        //if (outputImage == null || outputImage.texture == null)
        //{
        //    Debug.LogError("Output image or texture is null!");
        //    await Task.Delay(1000);
        //}

        // Create drawing texture with same dimensions as output image
        int width = 1920;
        int height = 1080;

        drawTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
        drawTexture.filterMode = FilterMode.Bilinear;

        // Set initial texture to transparent
        Color[] pixels = new Color[width * height];
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i] = Color.clear;
        }
        drawTexture.SetPixels(pixels);
        drawTexture.Apply();

        // Assign the texture to the output image
        outputImage.texture = drawTexture;
        textureInitialized = true;
    }

    void LateUpdate()
    {
        if (!textureInitialized || poseDetection == null)
            return;

        // Clear the texture
        ClearTexture();

        // Draw the pose visualization
        DrawPoseVisualization();

        // Apply changes to texture
        drawTexture.Apply();
    }

    void ClearTexture()
    {
        // Clear to transparent
        Color[] pixels = new Color[drawTexture.width * drawTexture.height];
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i] = Color.clear;
        }
        drawTexture.SetPixels(pixels);
    }

    void DrawPoseVisualization()
    {
        // Check if keypoints are available
        if (poseDetection.keypoints == null || poseDetection.keypoints.Length == 0)
            return;

        // Transform the 3D keypoints to 2D screen positions
        Vector2[] screenPoints = new Vector2[poseDetection.keypoints.Length];
        for (int i = 0; i < poseDetection.keypoints.Length; i++)
        {
            Vector3 worldPoint = poseDetection.keypoints[i];

            // Convert from world to screen coordinates
            // Note: Ignore Z, just use X and Y scaled to texture dimensions
            screenPoints[i] = new Vector2(
                (worldPoint.x * -1 + 0.5f) * drawTexture.width,
                (worldPoint.y * -1 + 0.5f) * drawTexture.height
            );
        }

        // Draw connections first (so points are on top)
        foreach (var connection in connections)
        {
            int from = connection.Item1;
            int to = connection.Item2;

            // Check if points are valid
            if (from < screenPoints.Length && to < screenPoints.Length)
            {
                DrawLine(screenPoints[from], screenPoints[to], lineColor, lineWidth);
            }
        }

        // Draw points
        for (int i = 0; i < screenPoints.Length; i++)
        {
            DrawPoint(screenPoints[i], pointColor, pointSize);
        }
    }

    void DrawPoint(Vector2 position, Color color, float size)
    {
        int radius = Mathf.CeilToInt(size / 2);
        for (int y = -radius; y <= radius; y++)
        {
            for (int x = -radius; x <= radius; x++)
            {
                if (x * x + y * y <= radius * radius)
                {
                    int pixelX = Mathf.RoundToInt(position.x + x);
                    int pixelY = Mathf.RoundToInt(position.y + y);

                    // Check if the pixel is within texture bounds
                    if (pixelX >= 0 && pixelX < drawTexture.width && pixelY >= 0 && pixelY < drawTexture.height)
                    {
                        drawTexture.SetPixel(pixelX, pixelY, color);
                    }
                }
            }
        }
    }

    void DrawLine(Vector2 start, Vector2 end, Color color, float width)
    {
        int w = Mathf.CeilToInt(width);
        int halfW = Mathf.FloorToInt(w / 2f);

        // Calculate direction and length
        Vector2 direction = (end - start).normalized;
        float length = Vector2.Distance(start, end);

        // Calculate perpendicular direction for line thickness
        Vector2 perpendicular = new Vector2(-direction.y, direction.x);

        // Draw the line
        for (int d = 0; d < Mathf.CeilToInt(length); d++)
        {
            Vector2 point = start + direction * d;

            for (int p = -halfW; p <= halfW; p++)
            {
                Vector2 offset = perpendicular * p;
                Vector2 pixelPos = point + offset;

                int x = Mathf.RoundToInt(pixelPos.x);
                int y = Mathf.RoundToInt(pixelPos.y);

                // Check if within texture bounds
                if (x >= 0 && x < drawTexture.width && y >= 0 && y < drawTexture.height)
                {
                    drawTexture.SetPixel(x, y, color);
                }
            }
        }
    }
}