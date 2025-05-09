/*
 *  This code handles the segmentation and background replacement aspects of the system through these key approaches:

    Compute Shader Processing: Uses GPU-accelerated compute shaders to process segmentation data and maintain a dynamic background model.
    Dynamic Background Learning: Gradually updates a background model over time by learning from non-human areas of each frame.
    Segmentation Threshold: Applies a threshold to the segmentation data to create a binary mask that identifies human/non-human pixels.
    Texture Pipeline: Manages a series of render textures for processing (segmentation mask, background model, final output).
    Inpainting: Can replace detected human region with the corresponding background pixels - "removing" the person from the scene. 
 */

using Unity.Sentis;
using UnityEngine;
using UnityEngine.UI;

public class SegmentationRenderer : MonoBehaviour
{
    public RawImage outputDisplay;          // The RawImage to display the final result
    public ComputeShader segmentationComputeShader;  // Compute shader to process segmentation data
    public ComputeShader backgroundModelComputeShader; // Compute shader for background model
    public float segmentationThreshold = 0.5f;   // Threshold for segmentation mask
    public Color fallbackColor = Color.black;  // Fallback color for areas not yet captured in background


    public Color segColor = Color.black;
    public float humanOpacity = 0.98f;
    public bool dontUseInPaint = true;


    private RenderTexture segmentationRT;       // RenderTexture for the segmentation mask
    private RenderTexture backgroundModelRT;     // RenderTexture storing the dynamic background model
    private RenderTexture outputRT;              // RenderTexture for the final output
    private ComputeBuffer segBuffer;             // ComputeBuffer to hold segmentation data
    private bool isInitialized = false;



    // Learning rate parameters
    public float baseLearningRate = 0.01f;
    public float farDistanceFactor = 3.0f;

    [Range(0f, 0.5f)]
    public float dialationAmmount = 0.015f;


    // Add these new fields to the SegmentationRenderer class
    [Header("Person-Only Output")]
    public bool generatePersonOnlyTexture = true;
    public RawImage personOnlyDisplay;  // Optional UI display for person-only texture
    public Material personOnlyMaterial;  // Material with PersonOnly shader
    private RenderTexture personOnlyRT;  // Render texture for the person-only output
    // Kernel handles for compute shaders
    private int segmentationKernel;
    private int updateBackgroundKernel;
    private int applyBackgroundKernel;

    /// <summary>
    /// Initialize resources for segmentation rendering and background modeling
    /// </summary>
    /// <param name="size">Size of the segmentation texture (usually 256)</param>
    /// <param name="webcamTexture">Reference to the webcam texture</param>
    public void Initialize(int size, Texture webcamTexture)
    {
        // Get webcam dimensions
        int webcamWidth = webcamTexture.width;
        int webcamHeight = webcamTexture.height;

        // Segmentation mask sRGB RT
        var segDesc = new RenderTextureDescriptor(size, size, RenderTextureFormat.ARGB32)
        {
            enableRandomWrite = true,
            sRGB = false
        };
        segmentationRT = new RenderTexture(segDesc);
        segmentationRT.Create();

        // Background model sRGB RT (webcam resolution)
        var bgDesc = new RenderTextureDescriptor(webcamWidth, webcamHeight, RenderTextureFormat.ARGB32)
        {
            enableRandomWrite = true,
            sRGB = false
        };
        backgroundModelRT = new RenderTexture(bgDesc);
        backgroundModelRT.Create();

        // Final output sRGB RT (webcam resolution)
        var outDesc = new RenderTextureDescriptor(webcamWidth, webcamHeight, RenderTextureFormat.ARGB32)
        {
            enableRandomWrite = true,
            sRGB = false
        };
        outputRT = new RenderTexture(outDesc);
        outputRT.Create();

        // Get kernel handles
        segmentationKernel = segmentationComputeShader.FindKernel("CSMain");

        // Initialize background model compute shader kernels
        if (backgroundModelComputeShader != null)
        {
            updateBackgroundKernel = backgroundModelComputeShader.FindKernel("UpdateBackground");
            applyBackgroundKernel = backgroundModelComputeShader.FindKernel("ApplyBackground");

            // Initialize the background model with the first webcam frame
            // This gives us a better starting point than black
            Graphics.Blit(webcamTexture, backgroundModelRT);

            // Don't try to dispatch compute shader in Initialize - 
            // just use Graphics.Blit for the initial frame
            Debug.Log("Background model initialized with webcam frame");
        }
        else
        {
            Debug.LogError("Background model compute shader not assigned!");
        }

        // Initialize person-only render texture if enabled
        if (generatePersonOnlyTexture)
        {
            personOnlyRT = new RenderTexture(webcamWidth, webcamHeight, 0, RenderTextureFormat.ARGB32);
            personOnlyRT.enableRandomWrite = true;
            personOnlyRT.Create();

            // If a UI element is provided, assign the texture to it
            if (personOnlyDisplay != null)
            {
                personOnlyDisplay.texture = personOnlyRT;
            }
        }

        // Assign the output to the display UI element
        outputDisplay.texture = outputRT;

        isInitialized = true;
    }

    /// <summary>
    /// Update the segmentation data, background model, and render the final output
    /// </summary>
    /// <param name="segData">Segmentation data tensor from model</param>
    /// <param name="webcamTexture">Current webcam texture</param>
    /// <param name="transformMatrix">Transform matrix for mapping webcam to segmentation space</param>
    public void UpdateSegmentation(Tensor<float> segData, Texture webcamTexture, Matrix4x4 transformMatrix)
    {
        if (!isInitialized)
        {
            Debug.LogWarning("SegmentationRenderer not initialized. Call Initialize() first.");
            return;
        }

        // Update the segmentation texture using the compute shader
        UpdateSegmentationTexture(segData);

        try
        {
            // Update the background model
            UpdateBackgroundModel(webcamTexture, transformMatrix);

            // If we're using the SegPaint shader, update it with our textures and transform
            if (outputDisplay.material != null && outputDisplay.material.shader.name == "SegPaint")
            {
                Material mat = outputDisplay.material;
                mat.SetTexture("_MainTex", webcamTexture);
                mat.SetTexture("_MaskTex", segmentationRT);
                mat.SetTexture("_BgTex", backgroundModelRT);
                mat.SetMatrix("_InvTransform", transformMatrix);
                mat.SetColor("_BackgroundColor", segColor);

                if (dontUseInPaint)
                {
                    mat.SetInt("_UseDynamicBg", 0);
                    mat.SetFloat("_DilationAmount", 0);
                    mat.SetFloat("_HumanOpacity", humanOpacity);

                }
                else
                {
                    mat.SetInt("_UseDynamicBg", 1);
                    mat.SetFloat("_DilationAmount", dialationAmmount);
                    mat.SetFloat("_HumanOpacity", 0f);


                }

            }

            // Generate the person-only texture if enabled
            if (generatePersonOnlyTexture && personOnlyMaterial != null)
            {
                // Set the material properties
                personOnlyMaterial.SetTexture("_MainTex", webcamTexture);
                personOnlyMaterial.SetTexture("_MaskTex", segmentationRT);
                personOnlyMaterial.SetMatrix("_InvTransform", transformMatrix);

                // Render to the person-only render texture
                Graphics.Blit(webcamTexture, personOnlyRT, personOnlyMaterial);
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error in UpdateBackgroundModel: " + e.Message);
            // Fallback - just show the webcam feed if there's an error
            Graphics.Blit(webcamTexture, outputRT);
        }
    }

    /// <summary>
    /// Process segmentation data using compute shader
    /// </summary>
    /// <param name="segData">Segmentation data tensor from model</param>
    private void UpdateSegmentationTexture(Tensor<float> segData)
    {
        int width = segmentationRT.width;
        int height = segmentationRT.height;
        int pixelCount = width * height;

        // Create or update the ComputeBuffer if needed
        if (segBuffer == null || segBuffer.count != pixelCount)
        {
            if (segBuffer != null) segBuffer.Release();
            segBuffer = new ComputeBuffer(pixelCount, sizeof(float));
        }

        // Extract data from tensor
        float[] segArray = segData.DownloadToArray();

        // Upload segmentation data to the ComputeBuffer
        segBuffer.SetData(segArray);

        // Set shader parameters
        segmentationComputeShader.SetInt("_Width", width);
        segmentationComputeShader.SetInt("_Height", height);
        segmentationComputeShader.SetBuffer(segmentationKernel, "segData", segBuffer);
        segmentationComputeShader.SetTexture(segmentationKernel, "Result", segmentationRT);

        // Dispatch the compute shader
        int threadGroupsX = Mathf.CeilToInt(width / 8.0f);
        int threadGroupsY = Mathf.CeilToInt(height / 8.0f);
        segmentationComputeShader.Dispatch(segmentationKernel, threadGroupsX, threadGroupsY, 1);
    }

    /// <summary>
    /// Update the background model with current frame and apply it to masked areas
    /// </summary>
    /// <param name="webcamTex">Current webcam texture</param>
    /// <param name="transformMatrix">Transform matrix for mapping webcam to segmentation space</param>
    private void UpdateBackgroundModel(Texture webcamTex, Matrix4x4 transformMatrix)
    {
        int webcamWidth = webcamTex.width;
        int webcamHeight = webcamTex.height;

        // 1. Update the background model with the current frame (only in non-human areas)
        try
        {
            backgroundModelComputeShader.SetTexture(updateBackgroundKernel, "InputTexture", webcamTex);
            backgroundModelComputeShader.SetTexture(updateBackgroundKernel, "SegmentationMask", segmentationRT);
            backgroundModelComputeShader.SetTexture(updateBackgroundKernel, "BackgroundModel", backgroundModelRT);
            backgroundModelComputeShader.SetMatrix("_TransformMatrix", transformMatrix);
            backgroundModelComputeShader.SetFloat("_SegmentationThreshold", segmentationThreshold);
            backgroundModelComputeShader.SetFloat("_LearningRate", baseLearningRate); // Base learning rate
            backgroundModelComputeShader.SetFloat("_FarDistanceFactor", farDistanceFactor); // Multiplier for far pixels
            backgroundModelComputeShader.SetFloat("_MinDecayRate", 0.001f); // ghost cleaner

            int threadGroupsX = Mathf.CeilToInt(webcamWidth / 8.0f);
            int threadGroupsY = Mathf.CeilToInt(webcamHeight / 8.0f);
            backgroundModelComputeShader.Dispatch(updateBackgroundKernel, threadGroupsX, threadGroupsY, 1);

            // 2. Apply the background model to the current frame (replace human areas with background)
            backgroundModelComputeShader.SetTexture(applyBackgroundKernel, "InputTexture", webcamTex);
            backgroundModelComputeShader.SetTexture(applyBackgroundKernel, "SegmentationMask", segmentationRT);
            backgroundModelComputeShader.SetTexture(applyBackgroundKernel, "BackgroundModel", backgroundModelRT);
            backgroundModelComputeShader.SetTexture(applyBackgroundKernel, "OutputTexture", outputRT);
            backgroundModelComputeShader.SetMatrix("_TransformMatrix", transformMatrix);
            backgroundModelComputeShader.SetFloat("_SegmentationThreshold", segmentationThreshold);
            backgroundModelComputeShader.SetVector("_FallbackColor", new Vector4(fallbackColor.r, fallbackColor.g, fallbackColor.b, 1.0f));

            backgroundModelComputeShader.Dispatch(applyBackgroundKernel, threadGroupsX, threadGroupsY, 1);

            // Debug logging
            //Debug.Log("Background model updated successfully");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error in compute shader dispatch: {e.Message}\nStack: {e.StackTrace}");

            // Fallback - just blit the webcam texture if shader fails
            Graphics.Blit(webcamTex, outputRT);
        }
    }


    /// <summary>
    /// Updates the display with just the raw webcam feed (no segmentation)
    /// Used when detection or tracking fails to ensure the feed continues
    /// </summary>
    /// <param name="webcamTexture">Current webcam texture</param>
    public void UpdateRawWebcamFeed(Texture webcamTexture)
    {
        if (!isInitialized)
        {
            Debug.LogWarning("SegmentationRenderer not initialized. Call Initialize() first.");
            return;
        }

        try
        {
            // Just display the webcam feed directly
            Graphics.Blit(webcamTexture, outputRT);

            // If we're using the SegPaint shader, update it with webcam texture
            if (outputDisplay.material != null && outputDisplay.material.shader.name == "SegPaint")
            {
                Material mat = outputDisplay.material;
                mat.SetTexture("_MainTex", webcamTexture);
                mat.SetInt("_DebugMode", 0); // Ensure normal display mode
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error in UpdateRawWebcamFeed: " + e.Message);
        }
    }

    /// <summary>
    /// Gets the render texture containing only the person from the webcam feed.
    /// This can be used for hand detection to ensure it's processing only the detected person.
    /// </summary>
    /// <returns>RenderTexture with the person isolated or null if not available</returns>
    public RenderTexture GetPersonOnlyTexture()
    {
        if (!isInitialized || !generatePersonOnlyTexture)
        {
            return null;
        }

        return personOnlyRT;
    }

    /// <summary>
    /// Clean up resources when the component is destroyed
    /// </summary>
    private void OnDestroy()
    {
        if (segBuffer != null)
        {
            segBuffer.Release();
            segBuffer = null;
        }

        if (segmentationRT != null)
        {
            segmentationRT.Release();
            segmentationRT = null;
        }

        if (backgroundModelRT != null)
        {
            backgroundModelRT.Release();
            backgroundModelRT = null;
        }

        if (outputRT != null)
        {
            outputRT.Release();
            outputRT = null;
        }
        if (personOnlyRT != null)
        {
            personOnlyRT.Release();
            personOnlyRT = null;
        }

    }

    /// <summary>
    /// Updates the entire background model with the current frame
    /// Used when no human is detected in the frame
    /// </summary>
    /// <param name="webcamTexture">Current webcam texture</param>
    public void UpdateEntireBackground(Texture webcamTexture)
    {
        if (!isInitialized)
        {
            Debug.LogWarning("SegmentationRenderer not initialized. Call Initialize() first.");
            return;
        }

        try
        {
            // When no human is detected, we can update the entire background model
            // with the current frame at a faster rate than normal
            float entireFrameUpdateRate = 0.1f; // 10% blend per frame, much faster than normal updates

            // Update the background model by blending the current frame
            // We'll create a simple compute shader operation or use a material to do this
            if (backgroundModelComputeShader != null)
            {
                int updateFullBgKernel = backgroundModelComputeShader.FindKernel("UpdateFullBackground");
                if (updateFullBgKernel >= 0) // Kernel exists
                {
                    backgroundModelComputeShader.SetTexture(updateFullBgKernel, "InputTexture", webcamTexture);
                    backgroundModelComputeShader.SetTexture(updateFullBgKernel, "BackgroundModel", backgroundModelRT);
                    backgroundModelComputeShader.SetFloat("_LearningRate", entireFrameUpdateRate);

                    int width, height;
                    GetDimensions(webcamTexture, out width, out height);

                    int threadGroupsX = Mathf.CeilToInt(width / 8.0f);
                    int threadGroupsY = Mathf.CeilToInt(height / 8.0f);
                    backgroundModelComputeShader.Dispatch(updateFullBgKernel, threadGroupsX, threadGroupsY, 1);
                }
                else
                {
                    // Fallback if kernel doesn't exist - use Graphics.Blit with a lerp
                    Graphics.Blit(webcamTexture, backgroundModelRT);
                }
            }
            else
            {
                // Fallback if no compute shader - direct update
                Graphics.Blit(webcamTexture, backgroundModelRT);
            }

            // Display the raw webcam feed (no segmentation)
            Graphics.Blit(webcamTexture, outputRT);

            // Update the output display
            outputDisplay.texture = outputRT;
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error in UpdateEntireBackground: " + e.Message);

            // Fallback - just show the webcam feed
            Graphics.Blit(webcamTexture, outputRT);
        }
    }

    // Helper method to get texture dimensions
    private void GetDimensions(Texture texture, out int width, out int height)
    {
        width = texture.width;
        height = texture.height;
    }

    /// <summary>
    /// Set the segmentation threshold at runtime
    /// </summary>
    /// <param name="threshold">New threshold value (0-1)</param>
    public void SetThreshold(float threshold)
    {
        segmentationThreshold = Mathf.Clamp01(threshold);
    }

    /// <summary>
    /// Set the fallback color at runtime
    /// </summary>
    /// <param name="color">New fallback color</param>
    public void SetFallbackColor(Color color)
    {
        fallbackColor = color;
    }

    /// <summary>
    /// Set the learning rate parameters for background updates
    /// </summary>
    /// <param name="baseRate">Base learning rate (0-1)</param>
    /// <param name="farFactor">Multiplier for pixels far from humans</param>
    public void SetLearningRateParameters(float baseRate, float farFactor)
    {
        baseLearningRate = Mathf.Clamp01(baseRate);
        farDistanceFactor = Mathf.Max(1.0f, farFactor);
    }

    /// <summary>
    /// Get a temporary debug view of the background model texture
    /// </summary>
    public void ShowBackgroundModelDebug()
    {
        if (isInitialized)
        {
            outputDisplay.texture = backgroundModelRT;
        }
    }

    /// <summary>
    /// Return to normal view (showing the output with background replacement)
    /// </summary>
    public void ShowNormalView()
    {
        if (isInitialized)
        {
            outputDisplay.texture = outputRT;
        }
    }
}