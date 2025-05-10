using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class WebcamSelector : MonoBehaviour
{
    [Header("UI References")]
    public GameObject selectorPanel;
    public TextMeshProUGUI availableCamerasText;
    public InputField webcamIdInput;
    public InputField view1Time;
    public InputField view2Time;
    public InputField view3Time;
    public Button confirmButton;
    public Toggle fpsToggle;
    public TextMeshProUGUI statusText;
    public GameObject viewController;
    [Header("References")]
    public WebcamInput webcamInput;  // Reference to your WebcamInput component
    public PoseDetection poseDetection;

    private WebCamDevice[] availableDevices;
    private bool selectionConfirmed = false;

    private void Start()
    {
        // Ensure we have references
        if (webcamInput == null)
        {
            Debug.LogError("WebcamInput reference is not assigned! Trying to find one in the scene...");
            webcamInput = FindObjectOfType<WebcamInput>();

            if (webcamInput == null)
            {
                Debug.LogError("No WebcamInput found in the scene! WebcamSelector cannot function.");
                gameObject.SetActive(false);
                return;
            }
        }

        if (selectorPanel == null || webcamIdInput == null || confirmButton == null || availableCamerasText == null)
        {
            Debug.LogError("Missing UI references for WebcamSelector!");
            return;
        }

        // Pause the application
        Time.timeScale = 0f;

        // Get all available webcams
        availableDevices = WebCamTexture.devices;

        // Display available webcams
        DisplayAvailableWebcams();

        // Set up button listener
        confirmButton.onClick.AddListener(ConfirmWebcamSelection);
    }

    private void DisplayAvailableWebcams()
    {
        string deviceList = "Available Webcams:\n\n";

        if (availableDevices.Length == 0)
        {
            deviceList += "No webcams detected!";
            statusText.text = "Error: No webcams found";
            statusText.color = Color.red;
        }
        else
        {
            for (int i = 0; i < availableDevices.Length; i++)
            {
                deviceList += $"ID {i}: {availableDevices[i].name}\n";
            }

            // Set default ID to 0
            webcamIdInput.text = "0";
        }

        availableCamerasText.text = deviceList;
    }

    private void ConfirmWebcamSelection()
    {
        if (availableDevices.Length == 0)
        {
            statusText.text = "No webcams available to select";
            statusText.color = Color.red;
            return;
        }

        if (string.IsNullOrEmpty(webcamIdInput.text) || string.IsNullOrEmpty(view1Time.text) || string.IsNullOrEmpty(view2Time.text) || string.IsNullOrEmpty(view3Time.text))
        {
            statusText.text = "Please enter valid inputs";
            statusText.color = Color.red;
            return;
        }

        if (int.TryParse(webcamIdInput.text, out int selectedId) && float.TryParse(view1Time.text, out float view1seconds) && float.TryParse(view2Time.text, out float view2seconds) && float.TryParse(view3Time.text, out float view3seconds))
        {
            if (selectedId >= 0 && selectedId < availableDevices.Length)
            {
                if (!(view1seconds <= 0f && view2seconds <= 0f && view3seconds <= 0f))
                {
                    ViewController viewControllerComponent = viewController.GetComponent<ViewController>();
                    if (viewControllerComponent != null)
                    {
                        viewControllerComponent.view1Time = Mathf.Clamp(view1seconds, 0, 99);
                        viewControllerComponent.view2Time = Mathf.Clamp(view2seconds, 0, 99);
                        viewControllerComponent.view3Time = Mathf.Clamp(view3seconds, 0, 99);
                    }

                    // Set the webcam ID in WebcamInput
                    webcamInput.WebcamID = selectedId;

                    // Hide selector and resume application
                    selectorPanel.SetActive(false);
                    selectionConfirmed = true;
                    Time.timeScale = 1f;



                    // Initialize WebcamInput
                    webcamInput.InitializeWebcam();

                    // Log the selection
                    Debug.Log($"Selected webcam ID {selectedId}: {availableDevices[selectedId].name}");

                    poseDetection.showFPS = fpsToggle.isOn;
                    viewController.SetActive(true);


                }
                else
                {
                    statusText.text = $"All three views can't be 0 seconds";
                    statusText.color = Color.red;
                }


            }
            else
            {
                statusText.text = $"Invalid Webcam ID. Please enter a number between 0 and {availableDevices.Length - 1}";
                statusText.color = Color.red;
            }
        }
        else
        {
            statusText.text = "Please enter a valid number";
            statusText.color = Color.red;
        }
    }

    private void Update()
    {
        // If user presses Enter, confirm selection
        if (!selectionConfirmed && Input.GetKeyDown(KeyCode.Return))
        {
            ConfirmWebcamSelection();
        }

        // If user presses Escape, use default webcam (ID 0)
        if (!selectionConfirmed && Input.GetKeyDown(KeyCode.Escape) && availableDevices.Length > 0)
        {
            webcamIdInput.text = "0";
            ConfirmWebcamSelection();
        }
    }

}