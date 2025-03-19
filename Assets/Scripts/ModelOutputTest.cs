using Unity.Sentis;
using UnityEngine;

public class ModelOutputTest : MonoBehaviour
{
    public ModelAsset handDetector;

    void Start()
    {
        var model = ModelLoader.Load(handDetector);
        var worker = new Worker(model, BackendType.GPUCompute);

        var inputTensor = new Tensor<float>(new TensorShape(1, 192, 192, 3), new float[1 * 192 * 192 * 3]);
        worker.Schedule(inputTensor);

        var scoresTensor = worker.PeekOutput(0) as Tensor<float>;
        var boxesTensor = worker.PeekOutput(1) as Tensor<float>;

        Debug.Log($"Scores shape: {scoresTensor.shape}");
        Debug.Log($"Boxes shape: {boxesTensor.shape}");

        worker.Dispose();
        inputTensor.Dispose();
    }
}