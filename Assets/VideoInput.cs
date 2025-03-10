using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Video;

public class VideoInput : MonoBehaviour
{
    public RawImage rawImage; // Assign the RawImage in the Inspector
    public VideoPlayer videoPlayer; // Assign the VideoPlayer here
    public RawImage keypointsImage; // Assign the RawImage for the keypoints overlay

    public int targetWidth = 384;  // Desired width for the model
    public int targetHeight = 288; // Desired height for the model

    private Texture2D resizedTexture;
    private Texture2D keypointsTexture;


    void Start()
    {
        if (rawImage == null)
        {
            Debug.LogError("RawImage is not assigned in WebcamCapture script.");
            return;
        }

        if (videoPlayer == null)
        {
            Debug.LogError("VideoPlayer is not assigned.");
            return;
        }

        // Initialize the resized texture
        resizedTexture = new Texture2D(targetWidth, targetHeight, TextureFormat.RGB24, false);

        // Assign the video texture to the RawImage
        rawImage.texture = videoPlayer.targetTexture;
        rawImage.material.mainTexture = videoPlayer.targetTexture;

        // Initialize the keypoints texture
        keypointsTexture = new Texture2D(targetWidth, targetHeight, TextureFormat.RGB24, false);
        ClearTexture(keypointsTexture, Color.white); // Fill with white background
        keypointsImage.texture = keypointsTexture;

        // Start playing the video
        videoPlayer.Play();
    }



    /// <summary>
    /// Draws keypoints on a white background.
    /// </summary>
    /// <param name="keypoints">Array of keypoints (x, y, z)</param>
    public void DrawKeypointsOnWhiteBackground(float[] keypoints)
    {
        if (keypoints == null || keypoints.Length < 17 * 3)
        {
            Debug.LogWarning("Keypoints are not available.");
            return;
        }

        // Clear the texture with a white background
        ClearTexture(keypointsTexture, Color.white);

        // Draw keypoints on the texture
        for (int i = 0; i < 17; i++)
        {
            int x = (int)(keypoints[i * 3] * keypointsTexture.width);
            int y = (int)((1 - keypoints[i * 3 + 1]) * keypointsTexture.height); // Flip Y coordinate
            DrawCircle(keypointsTexture, x, y, 5, Color.red); // Draw a red circle at each keypoint
        }

        // Apply changes to the texture
        keypointsTexture.Apply();

        // Assign the modified texture to the Keypoints RawImage
        keypointsImage.texture = keypointsTexture;
    }

    /// <summary>
    /// Clears a texture with a specific color.
    /// </summary>
    /// <param name="texture">Texture to clear</param>
    /// <param name="color">Color to fill the texture with</param>
    private void ClearTexture(Texture2D texture, Color color)
    {
        Color[] pixels = new Color[texture.width * texture.height];
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i] = color;
        }
        texture.SetPixels(pixels);
        texture.Apply();
    }
    /// <summary>
    /// Draws a circle on a texture at the specified position.
    /// </summary>
    /// <param name="texture">Texture to draw on</param>
    /// <param name="x">X position</param>
    /// <param name="y">Y position</param>
    /// <param name="radius">Radius of the circle</param>
    /// <param name="color">Color of the circle</param>
    private void DrawCircle(Texture2D texture, int x, int y, int radius, Color color)
    {
        for (int i = -radius; i <= radius; i++)
        {
            for (int j = -radius; j <= radius; j++)
            {
                if (i * i + j * j <= radius * radius)
                {
                    int px = x + i;
                    int py = y + j;

                    if (px >= 0 && px < texture.width && py >= 0 && py < texture.height)
                    {
                        texture.SetPixel(px, py, color);
                    }
                }
            }
        }
    }
    /// <summary>
    /// Captures the current frame, resizes it, and returns as Texture2D.
    /// </summary>
    /// <returns>Resized Texture2D frame</returns>
    public Texture2D GetResizedFrame()
    {
        if (videoPlayer == null || !videoPlayer.isPlaying)
        {
            Debug.LogWarning("VideoPlayer is not playing.");
            return null;
        }

        // Capture current frame from the Render Texture
        RenderTexture.active = videoPlayer.targetTexture;
        Texture2D currentFrame = new Texture2D(videoPlayer.targetTexture.width, videoPlayer.targetTexture.height, TextureFormat.RGB24, false);
        currentFrame.ReadPixels(new Rect(0, 0, videoPlayer.targetTexture.width, videoPlayer.targetTexture.height), 0, 0);
        currentFrame.Apply();
        RenderTexture.active = null;

        // Resize the frame
        ResizeTexture(currentFrame, resizedTexture);
        Destroy(currentFrame); // Clean up temporary texture

        return resizedTexture;
    }

    /// <summary>
    /// Resizes a source texture into a destination texture using bilinear filtering.
    /// </summary>
    /// <param name="source">Source Texture2D</param>
    /// <param name="dest">Destination Texture2D</param>
    private void ResizeTexture(Texture2D source, Texture2D dest)
    {
        RenderTexture rt = new RenderTexture(dest.width, dest.height, 24);
        RenderTexture.active = rt;
        Graphics.Blit(source, rt);
        dest.ReadPixels(new Rect(0, 0, dest.width, dest.height), 0, 0);
        dest.Apply();
        RenderTexture.active = null;
        rt.Release();
    }


    void OnDisable()
    {
        if (videoPlayer != null)
        {
            videoPlayer.Stop();
        }
    }
}