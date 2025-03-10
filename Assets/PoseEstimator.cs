using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Sentis;
using UnityEngine;
using UnityEngine.UI;

public class PoseEstimator : MonoBehaviour
{
    public WebcamInput webcamCapture; // Assign the WebcamCapture script here
    public ModelAsset modelAsset;       // Assign the RTMPose3D ONNX model here
    public AvatarController avatarController; // Assign the AvatarController here
    //public VideoInput videoInput; // Instead of WebcamInput
    public RawImage debugRaw;

    public GameObject spherePrefab;
    private List<GameObject> jointSpheres = new List<GameObject>();




    private Model runtimeModel;
    private Worker worker;
    private float[] results;

    void Start()
    {
        if (webcamCapture == null)
        {
            // Debug.LogError("WebcamCapture reference is not assigned.");
            // return;
        }

        if (modelAsset == null)
        {
            Debug.LogError("ModelAsset is not assigned.");
            return;
        }

        // Load the ONNX model
        runtimeModel = ModelLoader.Load(modelAsset);

        // Create a worker for inference using GPU
        worker = new Worker(runtimeModel, BackendType.GPUCompute);

        Debug.Log("PoseEstimator initialized.");
        // 1. Instantiate a sphere for each joint.
        for (int i = 0; i < 133; i++)
        {
            GameObject sphere = Instantiate(spherePrefab, this.transform);
            sphere.name = "JointSphere_" + i;
            jointSpheres.Add(sphere);
        }
    }

    async void Update()
    {
        // Capture the current frame
        Texture2D inputTexture = webcamCapture.GetFrame();
        // Texture2D inputTexture = videoInput.GetResizedFrame();


        if (inputTexture == null)
        {
            Debug.LogWarning("No frame captured from webcam.");
            return;
        }

        // Preprocess the frame into a tensor
        using Tensor<float> inputTensor = TextureConverter.ToTensor(inputTexture, width: 384, height: 288, channels: 3);

        //Texture2D visualTexture = TensorToTexture2D(inputTensor, 384, 288, 3);
        //debugRaw.texture = visualTexture;




        // Schedule the model execution with the input tensor
        worker.Schedule(inputTensor);

        // Wait for the inference to complete
        await Task.Yield(); // Allows other tasks to run

        // Get the output tensor
        Tensor<float> outputTensor = worker.PeekOutput() as Tensor<float>;

        if (outputTensor == null)
        {
            Debug.LogError("Failed to retrieve output tensor.");
            return;
        }

        // Download the output tensor to an array
        results = outputTensor.DownloadToArray();

        // Assuming 'results' is a float array containing 17 keypoints (each with x, y, z)
        // so the total length should be 51.
        if (results != null && results.Length >= 133 * 3)
        {
            for (int i = 0; i < 17; i++)
            {
                int baseIndex = i * 3;
                float x = results[baseIndex];
                float y = results[baseIndex + 1];
                float z = results[baseIndex + 2];
                Debug.Log(string.Format("Keypoint {0}: x = {1:F3}, y = {2:F3}, z = {3:F3}", i, x, y, z));

                Vector3 pos = new Vector3(x, y, z);
                jointSpheres[baseIndex].transform.localPosition = pos;
            }   
        }
        else
        {
            Debug.LogError("Results array is null or does not contain enough data.");
        }

        /// videoInput.DrawKeypointsOnWhiteBackground(results);






        // Pass the results to the AvatarController
        if (avatarController != null)
        {
            //avatarController.keypoints = results;
        }



    }

    Texture2D TensorToTexture2D(Tensor<float> tensor, int width, int height, int channels)
    {
        // Download the tensor data
        float[] tensorData = tensor.DownloadToArray();

        Texture2D texture = new Texture2D(width, height, TextureFormat.RGB24, false);

        // Create a Color array to hold the pixel data.
        Color[] pixels = new Color[width * height];

        // Assume channels == 3 (RGB)
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = (y * width + x) * channels;
                // If your tensor values are normalized (0-1), this works directly.
                // Otherwise, adjust accordingly.
                float r = tensorData[index];
                float g = tensorData[index + 1];
                float b = tensorData[index + 2];

                pixels[y * width + x] = new Color(r, g, b);
            }
        }

        texture.SetPixels(pixels);
        texture.Apply();
        return texture;
    }





    void OnDisable()
    {
        // Dispose of the worker to free GPU resources
        if (worker != null)
        {
            worker.Dispose();
            worker = null;
        }
    }
}
