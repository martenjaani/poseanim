using UnityEngine;
using UnityEngine.UI;

public class BackgroundUpdater : MonoBehaviour
{
    public RawImage rawImage; // Reference to the RawImage with the moving black area
    public ComputeShader backgroundUpdateShader; // Reference to the compute shader

    private RenderTexture backgroundTexture;    // Stores the clean background
    private RenderTexture currentFrameTexture;    // Stores the current frame
    private RenderTexture shaderOutputTexture;    // Captures the shader output
    private RawImage copyRawImage;                // The copy RawImage displaying the final output
    private int kernelHandle;

    private bool init = false;

    private void InitCall()
    {
        // Ensure rawImage is assigned and its material is set
        if (rawImage == null)
        {
            Debug.LogError("RawImage is not assigned!");
            return;
        }
        if (rawImage.material == null)
        {
            Debug.LogError("RawImage material is not assigned!");
            return;
        }

        int width = (int)rawImage.rectTransform.rect.width;
        int height = (int)rawImage.rectTransform.rect.height;

        // Initialize RenderTextures with random write enabled
        shaderOutputTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
        shaderOutputTexture.enableRandomWrite = true;
        shaderOutputTexture.Create();

        backgroundTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
        backgroundTexture.enableRandomWrite = true;
        backgroundTexture.Create();

        currentFrameTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
        currentFrameTexture.enableRandomWrite = true;
        currentFrameTexture.Create();

        Debug.Log("RenderTextures initialized successfully!");

        // Create the copy RawImage for final display
        CreateCopyRawImage();

        // Get the kernel handle for the compute shader and assign textures
        kernelHandle = backgroundUpdateShader.FindKernel("UpdateBackground");
        backgroundUpdateShader.SetTexture(kernelHandle, "CurrentFrameTexture", currentFrameTexture);
        backgroundUpdateShader.SetTexture(kernelHandle, "BackgroundTexture", backgroundTexture);

        // Also assign the background texture to the copy's material
        copyRawImage.material.SetTexture("_BackgroundTex", backgroundTexture);
    }

    private void CreateCopyRawImage()
    {
        // Create a new GameObject to hold the copy RawImage
        GameObject copyGameObject = new GameObject("CopyRawImage");
        copyGameObject.transform.SetParent(rawImage.transform.parent, false);

        // Add a RawImage component
        copyRawImage = copyGameObject.AddComponent<RawImage>();

        // Copy RectTransform properties from the original RawImage
        copyRawImage.rectTransform.sizeDelta = rawImage.rectTransform.sizeDelta;
        copyRawImage.rectTransform.anchorMin = rawImage.rectTransform.anchorMin;
        copyRawImage.rectTransform.anchorMax = rawImage.rectTransform.anchorMax;
        copyRawImage.rectTransform.pivot = rawImage.rectTransform.pivot;
        copyRawImage.rectTransform.anchoredPosition = rawImage.rectTransform.anchoredPosition;

        // Create a material using the custom shader and assign the original texture as _MainTex
        Material material = new Material(Shader.Find("Custom/FillMaskedArea"));
        material.SetTexture("_MainTex", rawImage.texture);
        copyRawImage.texture = rawImage.texture;
        copyRawImage.material = material;
    }

    // Call this method (e.g., from another script or Update) to update the textures continuously
    public void UpdateCall()
    {
        if (!init)
        {
            InitCall();
            init = true;
        }

        // Blit the current rawImage material to capture the shader output
        Graphics.Blit(null, shaderOutputTexture, rawImage.material);

        // Copy the shader output into currentFrameTexture
        Graphics.Blit(shaderOutputTexture, currentFrameTexture);

        // Update the _MainTex property in the copy material to use the latest shader output
        copyRawImage.material.SetTexture("_MainTex", shaderOutputTexture);

        // Compute the number of thread groups needed (numthreads is 8x8 in the compute shader)
        int threadGroupsX = Mathf.CeilToInt(currentFrameTexture.width / 8.0f);
        int threadGroupsY = Mathf.CeilToInt(currentFrameTexture.height / 8.0f);
        backgroundUpdateShader.Dispatch(kernelHandle, threadGroupsX, threadGroupsY, 1);
    }

    private void OnDestroy()
    {
        // Release RenderTextures on destroy
        if (shaderOutputTexture != null) shaderOutputTexture.Release();
        if (backgroundTexture != null) backgroundTexture.Release();
        if (currentFrameTexture != null) currentFrameTexture.Release();
    }
}
