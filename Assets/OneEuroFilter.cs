using UnityEngine;

/// <summary>
/// Implementation of the 1€ Filter for smoothing noisy signals
/// Based on the paper: "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive Systems"
/// </summary>
public class OneEuroFilter
{
    // Filter parameters
    private float m_MinCutoff; // Minimum cutoff frequency
    private float m_Beta;      // Cutoff slope (higher values = more aggressive filtering of fast movements)
    private float m_DCutoff;   // Derivative cutoff frequency

    // Filter state
    private Vector3 m_XFilter; // Position filter
    private Vector3 m_DxFilter; // Derivative filter
    private float m_LastTime;   // Last update time
    private bool m_Initialized;

    /// <summary>
    /// Creates a new 1€ Filter instance
    /// </summary>
    /// <param name="minCutoff">Minimum cutoff frequency (default = 1.0)</param>
    /// <param name="beta">Cutoff slope (default = 0.007)</param>
    /// <param name="dCutoff">Derivative cutoff frequency (default = 1.0)</param>
    public OneEuroFilter(float minCutoff = 1.0f, float beta = 0.007f, float dCutoff = 1.0f)
    {
        m_MinCutoff = minCutoff;
        m_Beta = beta;
        m_DCutoff = dCutoff;
        m_Initialized = false;
    }

    /// <summary>
    /// Filter a new position value
    /// </summary>
    /// <param name="value">The raw position value</param>
    /// <returns>The filtered position</returns>
    public Vector3 Filter(Vector3 value)
    {
        float currentTime = Time.time;

        // Initialize with first value
        if (!m_Initialized)
        {
            m_XFilter = value;
            m_DxFilter = Vector3.zero;
            m_Initialized = true;
            m_LastTime = currentTime;
            return value;
        }

        // Compute delta time
        float dt = currentTime - m_LastTime;
        if (dt <= 0 || float.IsNaN(dt))
        {
            dt = 0.01f; // Fallback value if time is invalid
        }

        // Sanity check for dt
        dt = Mathf.Clamp(dt, 0.0001f, 0.5f);
        m_LastTime = currentTime;

        // Compute derivative
        Vector3 dx = (value - m_XFilter) / dt;

        // Filter derivative
        Vector3 edx = ApplyLowPassFilter(dx, m_DxFilter, Alpha(dt, m_DCutoff));
        m_DxFilter = edx;

        // Use derivative to adjust cutoff frequency
        float cutoff = m_MinCutoff + m_Beta * edx.magnitude;

        // Filter position
        Vector3 filtered = ApplyLowPassFilter(value, m_XFilter, Alpha(dt, cutoff));
        m_XFilter = filtered;

        return filtered;
    }

    // Compute alpha parameter for low-pass filter based on cutoff frequency and delta time
    private float Alpha(float deltaTime, float cutoff)
    {
        float tau = 1.0f / (2.0f * Mathf.PI * cutoff);
        return 1.0f / (1.0f + tau / deltaTime);
    }

    // Apply low-pass filter to a vector
    private Vector3 ApplyLowPassFilter(Vector3 value, Vector3 prevValue, float alpha)
    {
        return Vector3.Lerp(prevValue, value, alpha);
    }

    /// <summary>
    /// Reset the filter with a known good value
    /// </summary>
    /// <param name="value">Value to initialize with</param>
    public void Reset(Vector3 value)
    {
        m_XFilter = value;
        m_DxFilter = Vector3.zero;
        m_Initialized = true;
        m_LastTime = Time.time;
    }

    /// <summary>
    /// Set filter parameters
    /// </summary>
    /// <param name="minCutoff">Minimum cutoff frequency</param>
    /// <param name="beta">Cutoff slope</param>
    /// <param name="dCutoff">Derivative cutoff frequency</param>
    public void SetParameters(float minCutoff, float beta, float dCutoff)
    {
        m_MinCutoff = Mathf.Max(0.01f, minCutoff);
        m_Beta = Mathf.Max(0.0f, beta);
        m_DCutoff = Mathf.Max(0.01f, dCutoff);
    }
}