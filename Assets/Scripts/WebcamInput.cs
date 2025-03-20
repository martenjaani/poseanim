using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using static Mediapipe.VideoPreStreamCalculatorOptions.Types;

public class WebcamInput : MonoBehaviour
{
    public int targetWidth;  // Desired width for the model
    public int targetHeight; // Desired height for the model
    public RawImage rawImage;
    public int WebcamID = 0;

    private WebCamTexture webCamTexture;
    private Texture2D resizedTexture;
    private RenderTexture renderTextureCopy;
    private bool isInitialized = false;
    private bool manualInitializationPending = false;

    void Start()
    {
        // Check if there's a WebcamSelector in the scene
        if (FindObjectOfType<WebcamSelector>() == null)
        {
            // No selector present, initialize webcam directly
            InitializeWebcam();
        }
        else
        {
            // Selector will call InitializeWebcam after user selection
            manualInitializationPending = true;
        }
    }

    public void InitializeWebcam()
    {
        if (isInitialized)
        {
            // Already initialized, release resources first
            ReleaseResources();
        }

        // Get the list of available webcams
        WebCamDevice[] devices = WebCamTexture.devices;

        // Ensure WebcamID is within valid range
        if (devices.Length == 0)
        {
            Debug.LogError("No webcam devices found!");
            return;
        }

        if (WebcamID < 0 || WebcamID >= devices.Length)
        {
            Debug.LogWarning($"Invalid WebcamID: {WebcamID}. Defaulting to 0.");
            WebcamID = 0;
        }

        // Initialize the webcam with selected device
        WebCamDevice webCamDevice = devices[WebcamID];
        Debug.Log($"Initializing webcam: {webCamDevice.name} (ID: {WebcamID})");

        webCamTexture = new WebCamTexture(webCamDevice.name);
        webCamTexture.Play();

        // Assign the webcam texture to the RawImage if available
        if (rawImage != null)
        {
            rawImage.texture = webCamTexture;
            rawImage.material.mainTexture = webCamTexture;
        }

        // Set up textures for processing
        resizedTexture = new Texture2D(targetWidth, targetHeight, TextureFormat.RGB24, false);
        renderTextureCopy = new RenderTexture(targetWidth, targetHeight, 0, RenderTextureFormat.ARGB32);
        renderTextureCopy.filterMode = FilterMode.Bilinear;

        // Wait until the webcam is fully initialized
        StartCoroutine(WaitForWebcamInit());

        manualInitializationPending = false;
    }

    private IEnumerator WaitForWebcamInit()
    {
        Debug.Log("Waiting for webcam initialization...");

        // Wait until the webcam is fully initialized with valid dimensions
        yield return new WaitUntil(() =>
            webCamTexture != null &&
            webCamTexture.width > 16 &&
            webCamTexture.GetNativeTexturePtr() != IntPtr.Zero
        );

        IntPtr pointer = webCamTexture.GetNativeTexturePtr();
        if (pointer == IntPtr.Zero)
        {
            Debug.LogError("Native texture pointer is zero even after initialization.");
        }
        else
        {
            // Bind the Texture2D to the native texture only once
            resizedTexture.UpdateExternalTexture(pointer);
            isInitialized = true;
            Debug.Log($"WebcamInput initialized: {webCamTexture.width}x{webCamTexture.height}");
        }
    }

    void Update()
    {
        if (isInitialized && webCamTexture != null && webCamTexture.isPlaying && rawImage != null)
        {
            // Ensure the RawImage displays the webcam correctly (optional)
            float ratio = (float)webCamTexture.width / webCamTexture.height;
            rawImage.rectTransform.sizeDelta = new Vector2(
                ratio * rawImage.rectTransform.sizeDelta.y,
                rawImage.rectTransform.sizeDelta.y
            );
        }
    }

    /// <summary>
    /// Returns the current resized texture.
    /// If the webcam isn't fully initialized, returns null.
    /// </summary>
    public Texture2D GetFrame()
    {
        if (!isInitialized || webCamTexture == null || !webCamTexture.isPlaying)
            return null;
        return resizedTexture;
    }

    public Texture GetFeedFrameCopy()
    {
        if (webCamTexture == null)
            return null;
        Graphics.Blit(webCamTexture, renderTextureCopy);
        return renderTextureCopy;
    }

    private void ReleaseResources()
    {
        if (webCamTexture != null)
        {
            webCamTexture.Stop();
            webCamTexture = null;
        }

        isInitialized = false;
    }

    void OnDisable()
    {
        ReleaseResources();
    }
}