using UnityEngine;
using UnityEngine.UI;

public class ManualArmRotationControl : MonoBehaviour
{
    public Transform leftArmBone;   // Drag your "mixamorig:LeftArm" bone here in Inspector
    public Transform leftForeArmBone; // Drag your "mixamorig:LeftForeArm" bone here in Inspector

    [Header("Left Arm Rotation Sliders")]
    public Slider leftArmRotXSlider;
    public Slider leftArmRotYSlider;
    public Slider leftArmRotZSlider;

    [Header("Left Forearm Rotation Sliders")]
    public Slider leftForeArmRotXSlider;
    public Slider leftForeArmRotYSlider;
    public Slider leftForeArmRotZSlider;

    void Update()
    {
        if (leftArmBone != null)
        {
            leftArmBone.localRotation = Quaternion.Euler(leftArmRotXSlider.value, leftArmRotYSlider.value, leftArmRotZSlider.value);
        }

        if (leftForeArmBone != null)
        {
            leftForeArmBone.localRotation = Quaternion.Euler(leftForeArmRotXSlider.value, leftForeArmRotYSlider.value, leftForeArmRotZSlider.value);
        }
    }
}