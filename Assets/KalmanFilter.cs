using UnityEngine;

/// <summary>
/// A simple 1D Kalman filter for smoothing scalar values.
/// </summary>
public class KalmanFilter1D
{
    public float Estimate { get; private set; }
    public float ErrorCovariance { get; private set; }
    public float Q { get; set; } // Process noise
    public float R { get; set; } // Measurement noise

    public KalmanFilter1D(float initialEstimate, float initialCovariance, float processNoise, float measurementNoise)
    {
        Estimate = initialEstimate;
        ErrorCovariance = initialCovariance;
        Q = processNoise;
        R = measurementNoise;
    }

    /// <summary>
    /// Updates the filter with a new measurement
    /// </summary>
    /// <param name="measurement">The new measurement to incorporate</param>
    /// <returns>The filtered estimate</returns>
    public float Update(float measurement)
    {
        // Predict
        float predictedEstimate = Estimate;
        float predictedCovariance = ErrorCovariance + Q;

        // Update
        float K = predictedCovariance / (predictedCovariance + R);
        Estimate = predictedEstimate + K * (measurement - predictedEstimate);
        ErrorCovariance = (1 - K) * predictedCovariance;

        return Estimate;
    }

    /// <summary>
    /// Resets the filter to initial values
    /// </summary>
    /// <param name="initialEstimate">Initial state estimate</param>
    /// <param name="initialCovariance">Initial error covariance</param>
    public void Reset(float initialEstimate, float initialCovariance)
    {
        Estimate = initialEstimate;
        ErrorCovariance = initialCovariance;
    }
}

/// <summary>
/// A simple 3D Kalman filter that runs independent 1D filters for x, y, and z.
/// </summary>
public class KalmanFilter3D
{
    public KalmanFilter1D FilterX { get; private set; }
    public KalmanFilter1D FilterY { get; private set; }
    public KalmanFilter1D FilterZ { get; private set; }

    public KalmanFilter3D(Vector3 initialEstimate, float initialCovariance, float processNoise, float measurementNoise)
    {
        FilterX = new KalmanFilter1D(initialEstimate.x, initialCovariance, processNoise, measurementNoise);
        FilterY = new KalmanFilter1D(initialEstimate.y, initialCovariance, processNoise, measurementNoise);
        FilterZ = new KalmanFilter1D(initialEstimate.z, initialCovariance, processNoise, measurementNoise);
    }

    /// <summary>
    /// Updates the filter with a new 3D measurement
    /// </summary>
    /// <param name="measurement">The new 3D measurement to incorporate</param>
    /// <returns>The filtered 3D estimate</returns>
    public Vector3 Update(Vector3 measurement)
    {
        float x = FilterX.Update(measurement.x);
        float y = FilterY.Update(measurement.y);
        float z = FilterZ.Update(measurement.z);
        return new Vector3(x, y, z);
    }

    /// <summary>
    /// Sets new noise parameters for all dimensions
    /// </summary>
    /// <param name="processNoise">New process noise value</param>
    /// <param name="measurementNoise">New measurement noise value</param>
    public void SetNoiseParameters(float processNoise, float measurementNoise)
    {
        FilterX.Q = processNoise;
        FilterX.R = measurementNoise;

        FilterY.Q = processNoise;
        FilterY.R = measurementNoise;

        FilterZ.Q = processNoise;
        FilterZ.R = measurementNoise;
    }

    /// <summary>
    /// Resets the filter to initial values
    /// </summary>
    /// <param name="initialEstimate">Initial state estimate vector</param>
    /// <param name="initialCovariance">Initial error covariance</param>
    public void Reset(Vector3 initialEstimate, float initialCovariance)
    {
        FilterX.Reset(initialEstimate.x, initialCovariance);
        FilterY.Reset(initialEstimate.y, initialCovariance);
        FilterZ.Reset(initialEstimate.z, initialCovariance);
    }
}