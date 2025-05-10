/*
 * This code handles the cycling of different views
 * View 1 - segmentation mask is applied on the human
 * View 2 - segmentation mask and keypoint visualisation are applied
 * View 3 - Human is replaced with the avatar
 */

using System.Collections;
using UnityEngine;

public class ViewController : MonoBehaviour
{
    [Range(0, 2)]
    public int viewMode = 0;
    public bool cycleViewModes = true;

    // Individual linger times for each view
    public float view1Time = 3f;
    public float view2Time = 5f;
    public float view3Time = 4f;

    // Input keys for manual view switching
    public KeyCode nextViewKey = KeyCode.RightArrow;
    public KeyCode previousViewKey = KeyCode.LeftArrow;

    public SegmentationRenderer segmentationRenderer;
    public PoseDetection poseDetection;

    private Coroutine cycleCoroutine;
    private int previousViewMode = -1;

    // Show segmentation mask and keypoints on top of the human
    private void activateView1()
    {
        poseDetection.avatarAvailable = false;
        segmentationRenderer.dontUseInPaint = true;
        Debug.Log("View 1 activated");
    }

    // Human removed, only keypoints
    private void activateView2()
    {
        segmentationRenderer.dontUseInPaint = false;
        poseDetection.avatarAvailable = false;
        Debug.Log("View 2 activated");
    }

    // No human or keypoints, only avatar
    private void activateView3()
    {
        poseDetection.avatarAvailable = true;
        segmentationRenderer.dontUseInPaint = false;
        Debug.Log("View 3 activated");
    }

    private void Start()
    {
        // Apply initial view mode immediately
        ApplyViewMode();
        previousViewMode = viewMode;

        // Start cycling if enabled
        if (cycleViewModes)
        {
            StartCycling();
        }
    }

    private void Update()
    {
        // Toggle cycling on/off
        if (cycleViewModes && cycleCoroutine == null)
        {
            StartCycling();
        }
        else if (!cycleViewModes && cycleCoroutine != null)
        {
            StopCoroutine(cycleCoroutine);
            cycleCoroutine = null;
        }

        // Check if viewMode was changed in the inspector
        if (viewMode != previousViewMode)
        {
            ApplyViewMode();
            previousViewMode = viewMode;
        }

        // Handle manual view switching when cycling is off
        if (!cycleViewModes)
        {
            if (Input.GetKeyDown(nextViewKey))
            {
                viewMode = (viewMode + 1) % 3;
                ApplyViewMode();
                previousViewMode = viewMode;
            }
            else if (Input.GetKeyDown(previousViewKey))
            {
                viewMode = (viewMode - 1 + 3) % 3; // +3 to avoid negative numbers
                ApplyViewMode();
                previousViewMode = viewMode;
            }
        }
    }

    private void StartCycling()
    {
        if (cycleCoroutine != null)
        {
            StopCoroutine(cycleCoroutine);
        }
        cycleCoroutine = StartCoroutine(CycleViewModes());
    }

    private IEnumerator CycleViewModes()
    {
        while (true)
        {
            // Skip views with zero duration
            if (GetCurrentViewTime() <= 0)
            {
                viewMode = (viewMode + 1) % 3;
                previousViewMode = viewMode;
                continue;  // Skip to next iteration without waiting
            }

            // Apply the view immediately
            ApplyViewMode();

            // Wait for the appropriate time
            yield return new WaitForSeconds(GetCurrentViewTime());

            // Find next valid view mode
            int nextView = (viewMode + 1) % 3;
            while (GetViewTime(nextView) <= 0)
            {
                nextView = (nextView + 1) % 3;

            }

            viewMode = nextView;
            previousViewMode = viewMode;
        }
    }

    // Helper to get time for a specific view
    private float GetViewTime(int viewIndex)
    {
        switch (viewIndex)
        {
            case 0: return view1Time;
            case 1: return view2Time;
            case 2: return view3Time;
            default: return 0f;
        }
    }

    private float GetCurrentViewTime()
    {
        switch (viewMode)
        {
            case 0:
                return view1Time;
            case 1:
                return view2Time;
            case 2:
                return view3Time;
            default:
                return view1Time;
        }
    }

    private void ApplyViewMode()
    {
        // Ensure we're within valid range
        viewMode = Mathf.Clamp(viewMode, 0, 2);

        switch (viewMode)
        {
            case 0:
                activateView1();
                break;
            case 1:
                activateView2();
                break;
            case 2:
                activateView3();
                break;
            default:
                activateView1();
                break;
        }
    }

    // Public method to manually change view mode with immediate effect
    public void SetViewMode(int mode)
    {
        // Stop current cycling if active
        if (cycleCoroutine != null)
        {
            StopCoroutine(cycleCoroutine);
            cycleCoroutine = null;
        }

        viewMode = Mathf.Clamp(mode, 0, 2);
        ApplyViewMode();
        previousViewMode = viewMode;

        // Restart cycling if enabled
        if (cycleViewModes)
        {
            StartCycling();
        }
    }

    // Optional: Add these public methods for UI buttons
    public void NextView()
    {
        SetViewMode((viewMode + 1) % 3);
    }

    public void PreviousView()
    {
        SetViewMode((viewMode - 1 + 3) % 3);
    }

    public void ToggleCycling()
    {
        cycleViewModes = !cycleViewModes;
    }
}