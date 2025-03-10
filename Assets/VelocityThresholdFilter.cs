using UnityEngine;

/// <summary>
/// A simple velocity-based low-pass filter to prevent sudden large changes in keypoint positions
/// </summary>
public class VelocityThresholdFilter
{
    private Vector3 m_LastValidPosition;
    private Vector3 m_LastVelocity;
    private bool m_Initialized = false;

    // Maximum velocity allowed between frames (in units per second)
    private float m_MaxVelocity;

    // Smoothing factor for velocity changes (higher = more smoothing)
    private float m_VelocitySmoothing;

    // Time tracking for velocity calculation
    private float m_LastUpdateTime;

    /// <summary>
    /// Create a new velocity threshold filter
    /// </summary>
    /// <param name="maxVelocity">Maximum allowed velocity (units per second)</param>
    /// <param name="velocitySmoothing">Smoothing factor for velocity changes (0-1)</param>
    public VelocityThresholdFilter(float maxVelocity = 5.0f, float velocitySmoothing = 0.5f)
    {
        m_MaxVelocity = maxVelocity;
        m_VelocitySmoothing = Mathf.Clamp01(velocitySmoothing);
        m_LastUpdateTime = Time.time;
    }

    /// <summary>
    /// Filter a new position value based on velocity constraints
    /// </summary>
    /// <param name="newPosition">The raw new position</param>
    /// <returns>The filtered position</returns>
    public Vector3 Filter(Vector3 newPosition)
    {
        // Initialize with the first valid position
        if (!m_Initialized)
        {
            m_LastValidPosition = newPosition;
            m_LastVelocity = Vector3.zero;
            m_Initialized = true;
            m_LastUpdateTime = Time.time;
            return newPosition;
        }

        float deltaTime = Time.time - m_LastUpdateTime;
        m_LastUpdateTime = Time.time;

        // Avoid division by zero or very small deltaTime
        if (deltaTime < 0.001f)
            deltaTime = 0.001f;

        // Calculate velocity
        Vector3 currentVelocity = (newPosition - m_LastValidPosition) / deltaTime;

        // Apply smoothing to velocity
        Vector3 smoothedVelocity = Vector3.Lerp(m_LastVelocity, currentVelocity, 1.0f - m_VelocitySmoothing);

        // Check if velocity is within threshold
        float velocityMagnitude = smoothedVelocity.magnitude;

        if (velocityMagnitude > m_MaxVelocity)
        {
            // Cap velocity to maximum
            smoothedVelocity = smoothedVelocity.normalized * m_MaxVelocity;

            // Calculate new position based on capped velocity
            newPosition = m_LastValidPosition + (smoothedVelocity * deltaTime);
        }

        // Update state
        m_LastValidPosition = newPosition;
        m_LastVelocity = smoothedVelocity;

        return newPosition;
    }

    /// <summary>
    /// Reset the filter with a known good position
    /// </summary>
    /// <param name="position">Position to initialize with</param>
    public void Reset(Vector3 position)
    {
        m_LastValidPosition = position;
        m_LastVelocity = Vector3.zero;
        m_Initialized = true;
        m_LastUpdateTime = Time.time;
    }

    /// <summary>
    /// Set the maximum velocity threshold
    /// </summary>
    /// <param name="maxVelocity">Maximum velocity in units per second</param>
    public void SetMaxVelocity(float maxVelocity)
    {
        m_MaxVelocity = Mathf.Max(0.1f, maxVelocity);
    }

    /// <summary>
    /// Set the velocity smoothing factor
    /// </summary>
    /// <param name="smoothing">Smoothing factor (0-1)</param>
    public void SetVelocitySmoothing(float smoothing)
    {
        m_VelocitySmoothing = Mathf.Clamp01(smoothing);
    }
}