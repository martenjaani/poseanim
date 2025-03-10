using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using static Mediapipe.VideoPreStreamCalculatorOptions.Types;

public class WebcamInput : MonoBehaviour
{
    public RawImage rawImage; // Assign the RawImage in the Inspector
    public int targetWidth;  // Desired width for the model
    public int targetHeight; // Desired height for the model

    public int WebcamID = 1;

    private WebCamTexture webCamTexture;
    private Texture2D resizedTexture;

    private RenderTexture renderTextureCopy;

    private bool isInitialized = false;

    void Start()
    {
        if (rawImage == null)
        {
            Debug.LogError("RawImage is not assigned in WebcamCapture script.");
            return;
        }

        // Initialize the webcam
        webCamTexture = new WebCamTexture();
        var webCamDevice = WebCamTexture.devices[WebcamID];
        webCamTexture = new WebCamTexture(webCamDevice.name);
        webCamTexture.Play();

        // Assign the webcam texture to the RawImage
        rawImage.texture = webCamTexture;
        rawImage.material.mainTexture = webCamTexture;



        // webcam texture into 2dTexture
        resizedTexture = new Texture2D(targetWidth, targetHeight, TextureFormat.RGB24, false);

        renderTextureCopy = new RenderTexture(targetWidth, targetHeight, 0, RenderTextureFormat.ARGB32);
        renderTextureCopy.filterMode = FilterMode.Bilinear; // Adjust as needed



        // Wait until the webcam is initialized.
        StartCoroutine(WaitForWebcamInit());


    }

    private IEnumerator WaitForWebcamInit()
    {
        // Wait until the webcam is fully initialized.
        yield return new WaitUntil(() => webCamTexture.width > 16 && webCamTexture.GetNativeTexturePtr() != System.IntPtr.Zero);

        IntPtr pointer = webCamTexture.GetNativeTexturePtr();
        if (pointer == System.IntPtr.Zero)
        {
            Debug.LogError("Native texture pointer is zero even after initialization.");
        }
        else
        {
            // Bind the Texture2D to the native texture only once.
            resizedTexture.UpdateExternalTexture(pointer);
            isInitialized = true;
            Debug.Log($"WebcamInput initialized: {webCamTexture.width}x{webCamTexture.height}");
        }
    }

    // Add this to WebcamInput to ensure consistent texture size
    void Update()
    {
        if (isInitialized && webCamTexture.isPlaying)
        {
            // Ensure the RawImage displays the webcam correctly
            float ratio = (float)webCamTexture.width / webCamTexture.height;
            rawImage.rectTransform.sizeDelta = new Vector2(ratio * rawImage.rectTransform.sizeDelta.y,
                                                          rawImage.rectTransform.sizeDelta.y);
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
        return renderTextureCopy; // Return renderTextureCopy, not resizedTexture
    }



    void OnDisable()
    {
        if (webCamTexture != null)
        {
            webCamTexture.Stop();
        }
    }



}
