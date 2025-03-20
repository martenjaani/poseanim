using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using TMPro;

public class WebcamSelector : MonoBehaviour
{
    [Header("UI References")]
    public GameObject selectorPanel;
    public TextMeshProUGUI availableCamerasText;
    public InputField webcamIdInput;
    public Button confirmButton;
    public TextMeshProUGUI statusText;

    [Header("References")]
    public WebcamInput webcamInput;  // Reference to your WebcamInput component

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

        if (string.IsNullOrEmpty(webcamIdInput.text))
        {
            statusText.text = "Please enter a valid webcam ID";
            statusText.color = Color.red;
            return;
        }

        if (int.TryParse(webcamIdInput.text, out int selectedId))
        {
            if (selectedId >= 0 && selectedId < availableDevices.Length)
            {
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
            }
            else
            {
                statusText.text = $"Invalid ID. Please enter a number between 0 and {availableDevices.Length - 1}";
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