using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using System.Linq;

public class AvatarController : MonoBehaviour
{
    // Public KeypointReceiver
    // public KeypointReceiver keypointReceiver;
    public PoseDetection poseDetection;

    [Range(0f, 1f)]
    public float smoothing = 0.7f; // Smoothing factor for rotations

    // Bone name constants (Mixamo) - Updated to include SPINE1, SPINE2 and corrected arm/shoulder names
    private const string HIPS = "mixamorig:Hips";
    private const string SPINE = "mixamorig:Spine";
    private const string SPINE1 = "mixamorig:Spine1"; // Added SPINE1
    private const string SPINE2 = "mixamorig:Spine2"; // Added SPINE2
    private const string NECK = "mixamorig:Neck";
    private const string HEAD = "mixamorig:Head";
    private const string NOSE = "mixamorig:Nose";
    private const string LEFT_SHOULDER = "mixamorig:LeftShoulder"; // Mixamo Shoulder is like Clavicle/Traps
    private const string RIGHT_SHOULDER = "mixamorig:RightShoulder"; // Mixamo Shoulder is like Clavicle/Traps
    private const string LEFT_ARM = "mixamorig:LeftArm"; // Mixamo Arm is Upper Arm
    private const string RIGHT_ARM = "mixamorig:RightArm"; // Mixamo Arm is Upper Arm
    private const string LEFT_FOREARM = "mixamorig:LeftForeArm";
    private const string RIGHT_FOREARM = "mixamorig:RightForeArm";
    private const string LEFT_HAND = "mixamorig:LeftHand";
    private const string RIGHT_HAND = "mixamorig:RightHand";
    private const string LEFT_UPLEG = "mixamorig:LeftUpLeg";
    private const string RIGHT_UPLEG = "mixamorig:RightUpLeg";
    private const string LEFT_LEG = "mixamorig:LeftLeg";
    private const string RIGHT_LEG = "mixamorig:RightLeg";
    private const string LEFT_FOOT = "mixamorig:LeftFoot";
    private const string RIGHT_FOOT = "mixamorig:RightFoot";
    private const string LEFT_PINKY_BASE = "mixamorig:LeftHandPinky1";
    private const string LEFT_RING_BASE = "mixamorig:LeftHandRing1";
    private const string LEFT_MIDDLE_BASE = "mixamorig:LeftHandMiddle1";
    private const string LEFT_INDEX_BASE = "mixamorig:LeftHandIndex1";
    private const string LEFT_THUMB_BASE = "mixamorig:LeftHandThumb1";
    private const string RIGHT_PINKY_BASE = "mixamorig:RightHandPinky1";
    private const string RIGHT_RING_BASE = "mixamorig:RightHandRing1";
    private const string RIGHT_MIDDLE_BASE = "mixamorig:RightHandMiddle1";
    private const string RIGHT_INDEX_BASE = "mixamorig:RightHandIndex1";
    private const string RIGHT_THUMB_BASE = "mixamorig:RightHandThumb1";
    private const string LEFT_TOEBASE = "mixamorig:LeftToe_End";
    private const string RIGHT_TOEBASE = "mixamorig:RightToe_End";

    // COCO indices
    //private const int NOSE_INDEX = 0;
    //private const int LEFT_EYE_INDEX = 1;
    //private const int RIGHT_EYE_INDEX = 2;
    //private const int LEFT_EAR_INDEX = 3;
    //private const int RIGHT_EAR_INDEX = 4;
    //private const int LEFT_SHOULDER_INDEX = 5;
    //private const int LEFT_ELBOW_INDEX = 7;
    //private const int LEFT_WRIST_INDEX = 9;
    //private const int LEFT_HIP_INDEX = 11;
    //private const int RIGHT_HIP_INDEX = 12;
    //private const int RIGHT_SHOULDER_INDEX = 6;
    //private const int RIGHT_ELBOW_INDEX = 8;
    //private const int RIGHT_WRIST_INDEX = 10;
    //private const int RIGHT_UPLEG_INDEX = 12;
    //private const int LEFT_UPLEG_INDEX = 11;
    //private const int RIGHT_KNEE_INDEX = 14;
    //private const int LEFT_KNEE_INDEX = 13;
    //private const int RIGHT_ANKLE_INDEX = 16;
    //private const int LEFT_ANKLE_INDEX = 15;
    //private const int LEFT_BIG_TOE_INDEX = 17;
    //private const int RIGHT_BIG_TOE_INDEX = 20;
    //private const int LEFT_SMALL_TOE_INDEX = 18;
    //private const int RIGHT_ANKLE_BOTTOM = 22;
    //private const int LEFT_ANKLE_BOTTOM = 19;
    //private const int RIGHT_SMALL_TOE_INDEX = 21;
    //private const int RIGHT_PINKY_BASE_INDEX = 129;
    //private const int RIGHT_RING_BASE_INDEX = 125;
    //private const int RIGHT_MIDDLE_BASE_INDEX = 121;
    //private const int RIGHT_INDEX_BASE_INDEX = 117;
    //private const int RIGHT_THUMB_BASE_INDEX = 114;
    //private const int LEFT_PINKY_BASE_INDEX = 108;
    //private const int LEFT_RING_BASE_INDEX = 104;
    //private const int LEFT_MIDDLE_BASE_INDEX = 100;
    //private const int LEFT_INDEX_BASE_INDEX = 96;
    //private const int LEFT_THUMB_BASE_INDEX = 93;


    // BlazePose keypoint indices
    private const int NOSE_INDEX = 0;
    private const int LEFT_EYE_INNER_INDEX = 1;
    private const int LEFT_EYE_INDEX = 2;
    private const int LEFT_EYE_OUTER_INDEX = 3;
    private const int RIGHT_EYE_INNER_INDEX = 4;
    private const int RIGHT_EYE_INDEX = 5;
    private const int RIGHT_EYE_OUTER_INDEX = 6;
    private const int LEFT_EAR_INDEX = 7;
    private const int RIGHT_EAR_INDEX = 8;
    private const int MOUTH_LEFT_INDEX = 9;
    private const int MOUTH_RIGHT_INDEX = 10;
    private const int LEFT_SHOULDER_INDEX = 11;
    private const int RIGHT_SHOULDER_INDEX = 12;
    private const int LEFT_ELBOW_INDEX = 13;
    private const int RIGHT_ELBOW_INDEX = 14;
    private const int LEFT_WRIST_INDEX = 15;
    private const int RIGHT_WRIST_INDEX = 16;
    private const int LEFT_PINKY_BASE_INDEX = 17;
    private const int RIGHT_PINKY_BASE_INDEX = 18;
    private const int LEFT_INDEX_BASE_INDEX = 19;
    private const int RIGHT_INDEX_BASE_INDEX = 20;
    private const int LEFT_THUMB_BASE_INDEX = 21;
    private const int RIGHT_THUMB_BASE_INDEX = 22;
    private const int LEFT_HIP_INDEX = 23;
    private const int RIGHT_HIP_INDEX = 24;
    private const int LEFT_KNEE_INDEX = 25;
    private const int RIGHT_KNEE_INDEX = 26;
    private const int LEFT_ANKLE_INDEX = 27;
    private const int RIGHT_ANKLE_INDEX = 28;
    private const int LEFT_HEEL_INDEX = 29;
    private const int RIGHT_HEEL_INDEX = 30;
    private const int LEFT_FOOT_INDEX = 31;
    private const int RIGHT_FOOT_INDEX = 32;


    public Dictionary<string, Bone> Bones => bones;


    private List<string> allBoneNames = new List<string> { HIPS, SPINE, 
        //SPINE1, SPINE2, 
        NECK, HEAD,
            LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_ARM, RIGHT_ARM, LEFT_FOREARM, RIGHT_FOREARM, LEFT_HAND,
            RIGHT_HAND, LEFT_UPLEG, RIGHT_UPLEG, LEFT_LEG, RIGHT_LEG, LEFT_FOOT, RIGHT_FOOT,
            LEFT_PINKY_BASE, LEFT_RING_BASE, LEFT_MIDDLE_BASE, LEFT_INDEX_BASE, LEFT_THUMB_BASE,
            RIGHT_PINKY_BASE, RIGHT_RING_BASE, RIGHT_MIDDLE_BASE, RIGHT_INDEX_BASE, RIGHT_THUMB_BASE, LEFT_TOEBASE, RIGHT_TOEBASE, NOSE
            };
    // Bones Class
    public class Bone
    {
        public Transform Transform = null;
        public Vector3 KeypointPosition = Vector3.zero;
        public Bone Child = null;
        public Bone Parent = null;
        public Quaternion InitRotation;
        public Quaternion InverseRotation;

    }
    // Move in z direction
    private float centerTall = 200 * 0.75f;
    private float tall = 200 * 0.75f;
    private float prevTall = 200 * 0.75f;
    public float ZScaleOffset = -0.3f;
    public float ZTrueScale = 0.47f; //hea kui 0.1 - 0.5



    private Dictionary<string, Bone> bones = new Dictionary<string, Bone>();
    private Vector3 initPosition;
    private bool areBonesCached = false;

    void Start()
    {
        StartCoroutine(InitializeAvatar());
    }

    IEnumerator InitializeAvatar()
    {
        // Wait until keypointReceiver.latestKeypoints3D is populated and has at least 133 keypoints (or your desired condition)
        while (poseDetection.keypoints == null || poseDetection.keypoints.Length < 33) // Adjust the count if needed
        {
            Debug.Log("Waiting for Keypoints to be populated...");
            yield return null; // Wait for the next frame
        }

        Debug.Log("Keypoints Populated! Proceeding with FindAndCacheBones.");
        FindAndCacheBones();
    }
    private void FindAndCacheBones()
    {
        bones = new Dictionary<string, Bone>();

        // Find all transforms in the hierarchy
        Transform[] allTransforms = GetComponentsInChildren<Transform>();

        // Helper function to find a transform by name (case-insensitive)
        Transform FindTransform(string boneName)
        {
            foreach (var transform in allTransforms)
            {
                if (transform.name.Equals(boneName, System.StringComparison.OrdinalIgnoreCase))
                {
                    return transform;
                }
            }
            Debug.LogWarning("Bone not found: " + boneName);
            return null;
        }



        for (int i = 0; i < allBoneNames.Count; i++)
        {
            var boneName = allBoneNames[i];
            bones.Add(boneName, new Bone());
            bones[boneName].Transform = FindTransform(boneName);
        }

        //List<Vector3> keypoints3D = keypointReceiver.latestKeypoints3D; // Correct type: List<Vector3>
        // --- 1. Update Bone Keypoint Positions ---
        //UpdateBoneKeypointPositions(keypoints3D);

        // Set Child-Parent Relationships - Updated for SPINE1, SPINE2 and full spine chain + arm/leg hierarchy
        // Spine and Head Chain - Full Spine Hierarchy (Hips -> Spine -> Spine1 -> Spine2 -> Neck -> Head)
        // SetBoneChildren(HIPS, SPINE);
        // SetBoneChildren(SPINE, SPINE1);
        // SetBoneChildren(SPINE1, SPINE2);
        // SetBoneChildren(SPINE2, NECK);
        // SetBoneChildren(NECK, HEAD);

        SetBoneChildren(SPINE, NECK); //wierd
        SetBoneChildren(NECK, HEAD); //wierd

        // Right Arm - Corrected hierarchy and bone names
        SetBoneChildren(RIGHT_SHOULDER, RIGHT_ARM);     // Shoulder (Clavicle) -> Arm (Upper Arm)
        SetBoneChildren(RIGHT_ARM, RIGHT_FOREARM);    // Arm (Upper Arm) -> Forearm (Lower Arm)
        SetBoneChildren(RIGHT_FOREARM, RIGHT_HAND);   // Forearm (Lower Arm) -> Hand

        // Left Arm - Corrected hierarchy and bone names
        SetBoneChildren(LEFT_SHOULDER, LEFT_ARM);      // Shoulder (Clavicle) -> Arm (Upper Arm)
        SetBoneChildren(LEFT_ARM, LEFT_FOREARM);     // Arm (Upper Arm) -> Forearm (Lower Arm)
        SetBoneChildren(LEFT_FOREARM, LEFT_HAND);    // Forearm (Lower Arm) -> Hand

        // Right Leg
        SetBoneChildren(RIGHT_UPLEG, RIGHT_LEG);
        SetBoneChildren(RIGHT_LEG, RIGHT_FOOT);
        SetBoneChildren(RIGHT_FOOT, RIGHT_TOEBASE);

        // Left Leg
        SetBoneChildren(LEFT_UPLEG, LEFT_LEG);
        SetBoneChildren(LEFT_LEG, LEFT_FOOT);
        SetBoneChildren(LEFT_FOOT, LEFT_TOEBASE);


        // Helper function to set child-parent relationships (same as before)
        void SetBoneChildren(string parentName, string childName)
        {
            if (bones.ContainsKey(parentName) && bones.ContainsKey(childName))
            {
                bones[parentName].Child = bones[childName];
                bones[childName].Parent = bones[parentName];
            }
            else
            {
                Debug.LogWarning("Could not set child-parent relationship for " + parentName + " and " + childName + ". Bone(s) might be missing.");
            }
        }


        // Set Inverse Rotations (same as before)
        var forward = TriangleNormal(bones[HIPS].Transform.position, bones[LEFT_UPLEG].Transform.position, bones[RIGHT_UPLEG].Transform.position);
        foreach (var boneEntry in bones)
        {
            Bone bone = boneEntry.Value;
            if (bone.Transform != null)
            {
                bone.InitRotation = bone.Transform.rotation;
            }

            if (bone.Child != null)
            {
                bone.InverseRotation = GetInverse(bone, bone.Child, forward) * bone.InitRotation;
            }
        }

        // Hip Specific Inverse Setup (same as before)
        if (bones.ContainsKey(HIPS))
        {
            Bone hipBone = bones[HIPS];
            initPosition = hipBone.Transform.position;
            hipBone.InverseRotation = Quaternion.Inverse(Quaternion.LookRotation(forward)) * hipBone.InitRotation;
        }

        if (bones.ContainsKey(HEAD))
        {
            Bone headBone = bones[HEAD];
            headBone.InitRotation = headBone.Transform.rotation;
            var gaze = bones[NOSE].Transform.position - headBone.Transform.position;
            headBone.InverseRotation = Quaternion.Inverse(Quaternion.LookRotation(gaze)) * headBone.InitRotation;

        }

        // Hand Specific Inverse Setup (same as before - with warnings)
        if (bones.ContainsKey(LEFT_HAND))
        {
            Bone lHandBone = bones[LEFT_HAND];
            Bone lIindex = bones[LEFT_INDEX_BASE];
            Bone lMiddle = bones[LEFT_MIDDLE_BASE];
            //Debug.Log($"{lHandBone.KeypointPosition}, {lThumb.KeypointPosition}, {lMiddle.KeypointPosition}");
            Vector3 lf = TriangleNormal(lHandBone.Transform.position, lMiddle.Transform.position, lIindex.Transform.position);

            Debug.Log(lf);
            Debug.Log(lHandBone.Transform.position);
            Debug.DrawRay(lHandBone.Transform.position, lf*100f, Color.green );
            

            lHandBone.InitRotation = lHandBone.Transform.rotation;
            lHandBone.InverseRotation = Quaternion.Inverse(Quaternion.LookRotation(lf, lMiddle.KeypointPosition - lHandBone.KeypointPosition)) * lHandBone.InitRotation;
        }

        if (bones.ContainsKey(RIGHT_HAND))
        {
            Bone rHandBone = bones[RIGHT_HAND];
            Bone rIndex = bones[RIGHT_INDEX_BASE];
            Bone rMiddle = bones[RIGHT_MIDDLE_BASE];
            //Debug.Log($"{rHandBone.KeypointPosition}, {rThumb.KeypointPosition}, {rMiddle.KeypointPosition}");

            Vector3 rf = TriangleNormal(rHandBone.Transform.position, rMiddle.Transform.position, rIndex.Transform.position);

            Debug.DrawRay(rHandBone.Transform.position, rf*20f, Color.green);

            rHandBone.InitRotation = rHandBone.Transform.rotation;
            rHandBone.InverseRotation = Quaternion.Inverse(Quaternion.LookRotation(rf, rMiddle.KeypointPosition - rHandBone.KeypointPosition)) * rHandBone.InitRotation;
        }




        areBonesCached = true;
        Debug.Log("Bone Caching and Inverse Setup Complete.");
    }


    // Helper Functions (same as before)
    Vector3 TriangleNormal(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 d1 = a - b;
        Vector3 d2 = a - c;
        Vector3 dd = Vector3.Cross(d1, d2);
        dd.Normalize();
        return dd;
    }

    private Quaternion GetInverse(Bone p1, Bone p2, Vector3 forward)
    {
        return Quaternion.Inverse(Quaternion.LookRotation(p1.Transform.position - p2.Transform.position, forward));
    }

    void Update()
    {
        if (poseDetection.keypoints.Length == 0 || areBonesCached == false)
        {
            Debug.LogWarning("No keypoint data received. Make sure KeypointReceiver is working and providing data.");
            return; // Exit Update if no keypoint data
        }

        Vector3[] keypoints3D = poseDetection.keypoints; // Correct type: List<Vector3>
        // --- 1. Update Bone Keypoint Positions ---
        UpdateBoneKeypointPositions(keypoints3D);

        var dist1 = Vector3.Distance(bones[HEAD].KeypointPosition, bones[NECK].KeypointPosition);
        var dist2 = Vector3.Distance(bones[NECK].KeypointPosition, bones[HIPS].KeypointPosition);
        var avgKneePos = (bones[LEFT_LEG].KeypointPosition + bones[RIGHT_LEG].KeypointPosition) / 2f;
        var dist3 = Vector3.Distance(bones[HIPS].KeypointPosition, avgKneePos);
        var dist4 = Vector3.Distance(avgKneePos, (bones[LEFT_FOOT].KeypointPosition + bones[RIGHT_FOOT].KeypointPosition) / 2f);
        var height = dist1 + dist2 + dist3 + dist4;

        //Debug.Log(height);

        // Low pass filter in z direction
        tall = height * 0.7f + prevTall * 0.3f;
        prevTall = tall;

        if (tall == 0)
        {
            tall = centerTall;
        }
        var zMovement =  tall * (1/ZTrueScale) + ZScaleOffset;
        //Debug.Log(zMovement);

        // --- 2. Hip Movement and Rotation ---
        Vector3 forward = TriangleNormal(bones[HIPS].KeypointPosition, bones[LEFT_UPLEG].KeypointPosition, bones[RIGHT_UPLEG].KeypointPosition); // Use KeypointPositions
        if (bones[HIPS].Transform != null && forward != Vector3.zero)
        {
            bones[HIPS].Transform.position = bones[HIPS].KeypointPosition * 2.5f + new Vector3(initPosition.x, initPosition.y, initPosition.z + zMovement); // Scaling factor 0.005f and initPosition - adjust as needed
            bones[HIPS].Transform.rotation = Quaternion.LookRotation(forward) * bones[HIPS].InverseRotation;
        }

        //Debug.Log(bones[HIPS].Transform.position.z);
        //Debug.Log(this.gameObject.transform.position.z);

        // --- 3. Bone Rotations (Loop) ---
        foreach (var boneEntry in allBoneNames)
        {
            Bone bone = bones[boneEntry];
            if (bone.Parent != null && bone.Child != null && bone.Parent.KeypointPosition != Vector3.zero && bone.Child.KeypointPosition != Vector3.zero && bone.Transform != null)
            {
                Vector3 fv = bone.Parent.KeypointPosition - bone.KeypointPosition; // fv: direction from current bone to parent bone
                Quaternion targetRotation = Quaternion.LookRotation(bone.KeypointPosition - bone.Child.KeypointPosition, fv) * bone.InverseRotation; // targetRotation based on keypoints

                if (!float.IsNaN(targetRotation.x) && !float.IsNaN(targetRotation.y) && !float.IsNaN(targetRotation.z) && !float.IsNaN(targetRotation.w)) // NaN check for rotation
                {
                    bone.Transform.rotation = Quaternion.Slerp(bone.Transform.rotation, targetRotation, smoothing); // Apply smoothing
                }
                else
                {
                    Debug.LogWarning($"Invalid rotation (NaN) detected for bone: {bone}. Skipping rotation update.");
                }
            }
            else if (bone.Child != null && bones[HIPS].KeypointPosition != Vector3.zero && bone.KeypointPosition != Vector3.zero && bone.Child.KeypointPosition != Vector3.zero && bone.Transform != null) // Handle bones without parent (e.g., Spine base in VNect example) using hip forward
            {
                Quaternion targetRotation = Quaternion.LookRotation(bone.KeypointPosition - bone.Child.KeypointPosition, forward) * bone.InverseRotation;
                if (!float.IsNaN(targetRotation.x) && !float.IsNaN(targetRotation.y) && !float.IsNaN(targetRotation.z) && !float.IsNaN(targetRotation.w)) // NaN check
                {
                    bone.Transform.rotation = Quaternion.Slerp(bone.Transform.rotation, targetRotation, smoothing); // Apply smoothing
                }
                else
                {
                    Debug.LogWarning($"Invalid rotation (NaN) detected for bone: {boneEntry} (Child only case). Skipping rotation update.");
                }
            }
        }
        var gaze = bones[NOSE].KeypointPosition - bones[HEAD].KeypointPosition;
        var f = TriangleNormal(bones[HEAD].KeypointPosition, GetKeypointOrZero(keypoints3D, LEFT_EAR_INDEX), GetKeypointOrZero(keypoints3D, RIGHT_EAR_INDEX));
        bones[HEAD].Transform.rotation = Quaternion.LookRotation(gaze, f) * bones[HEAD].InverseRotation;


        UpdateHandRotation(LEFT_HAND, LEFT_INDEX_BASE, LEFT_PINKY_BASE);
        UpdateHandRotation(RIGHT_HAND, RIGHT_INDEX_BASE, RIGHT_PINKY_BASE);


    }


    // --- Helper Function to Update Bone Keypoint Positions based on MediaPipe Keypoints ---
    void UpdateBoneKeypointPositions(Vector3[] keypoints3D)
    {
        var MidLeg = GetKeypointOrZero(keypoints3D, LEFT_HIP_INDEX, RIGHT_HIP_INDEX);
        var ChestKeypont = GetKeypointOrZero(keypoints3D, LEFT_SHOULDER_INDEX, RIGHT_SHOULDER_INDEX);
        var Spine1 = (MidLeg + ChestKeypont) / 2f; //spine1
        var Spine = (MidLeg + Spine1) / 2f; //spine
        var Hip = (MidLeg + Spine) / 2; //hip
        var Spine2 = (ChestKeypont + Spine1) /2; //spine2

        var HeadKeypoint = GetKeypointOrZero(keypoints3D, LEFT_EAR_INDEX, RIGHT_EAR_INDEX);
        var NeckKeypoint = (ChestKeypont + HeadKeypoint) / 2f;
        var LeftTrap = ((ChestKeypont + NeckKeypoint) / 2f + GetKeypointOrZero(keypoints3D, LEFT_SHOULDER_INDEX)) / 2f;
        var RightTrap = ((ChestKeypont + NeckKeypoint) / 2f + GetKeypointOrZero(keypoints3D, RIGHT_SHOULDER_INDEX)) / 2f;


        //var rToeMid = GetKeypointOrZero(keypoints3D, RIGHT_BIG_TOE_INDEX, RIGHT_BIG_TOE_INDEX);
        //var rAnkleBot = (GetKeypointOrZero(keypoints3D, RIGHT_ANKLE_BOTTOM) + rToeMid) / 2f;
        //
        //var lToeMid = GetKeypointOrZero(keypoints3D, LEFT_BIG_TOE_INDEX, LEFT_BIG_TOE_INDEX);
        //var lAnkleBot = (GetKeypointOrZero(keypoints3D, LEFT_ANKLE_BOTTOM) + lToeMid) / 2f;




        if (bones.ContainsKey(HIPS)) bones[HIPS].KeypointPosition = Hip;
        if (bones.ContainsKey(SPINE)) bones[SPINE].KeypointPosition = Spine;
        if (bones.ContainsKey(SPINE1)) bones[SPINE1].KeypointPosition = Spine1;
        if (bones.ContainsKey(SPINE2)) bones[SPINE2].KeypointPosition = Spine2;
        if (bones.ContainsKey(NECK)) bones[NECK].KeypointPosition = NeckKeypoint;
        if (bones.ContainsKey(HEAD)) bones[HEAD].KeypointPosition = HeadKeypoint;
        if (bones.ContainsKey(NOSE)) bones[NOSE].KeypointPosition = GetKeypointOrZero(keypoints3D, NOSE_INDEX);
        if (bones.ContainsKey(LEFT_SHOULDER)) bones[LEFT_SHOULDER].KeypointPosition = LeftTrap;
        if (bones.ContainsKey(RIGHT_SHOULDER)) bones[RIGHT_SHOULDER].KeypointPosition = RightTrap;
        if (bones.ContainsKey(LEFT_ARM)) bones[LEFT_ARM].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_SHOULDER_INDEX);
        if (bones.ContainsKey(RIGHT_ARM)) bones[RIGHT_ARM].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_SHOULDER_INDEX);
        if (bones.ContainsKey(LEFT_FOREARM)) bones[LEFT_FOREARM].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_ELBOW_INDEX);
        if (bones.ContainsKey(RIGHT_FOREARM)) bones[RIGHT_FOREARM].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_ELBOW_INDEX);
        if (bones.ContainsKey(LEFT_HAND)) bones[LEFT_HAND].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_WRIST_INDEX); 
        if (bones.ContainsKey(RIGHT_HAND)) bones[RIGHT_HAND].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_WRIST_INDEX); 
        if (bones.ContainsKey(LEFT_UPLEG)) bones[LEFT_UPLEG].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_HIP_INDEX);  
        if (bones.ContainsKey(RIGHT_UPLEG)) bones[RIGHT_UPLEG].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_HIP_INDEX); 
        if (bones.ContainsKey(LEFT_LEG)) bones[LEFT_LEG].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_KNEE_INDEX);
        if (bones.ContainsKey(RIGHT_LEG)) bones[RIGHT_LEG].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_KNEE_INDEX);
        if (bones.ContainsKey(LEFT_FOOT)) bones[LEFT_FOOT].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_ANKLE_INDEX);
        if (bones.ContainsKey(RIGHT_FOOT)) bones[RIGHT_FOOT].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_ANKLE_INDEX);
        if (bones.ContainsKey(LEFT_TOEBASE)) bones[LEFT_TOEBASE].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_FOOT_INDEX);
        if (bones.ContainsKey(RIGHT_TOEBASE)) bones[RIGHT_TOEBASE].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_FOOT_INDEX);
        if (bones.ContainsKey(LEFT_THUMB_BASE)) bones[LEFT_THUMB_BASE].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_THUMB_BASE_INDEX);
        if (bones.ContainsKey(RIGHT_THUMB_BASE)) bones[RIGHT_THUMB_BASE].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_THUMB_BASE_INDEX);
       // if (bones.ContainsKey(LEFT_MIDDLE_BASE)) bones[LEFT_MIDDLE_BASE].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_MIDDLE_BASE_INDEX);
       // if (bones.ContainsKey(RIGHT_MIDDLE_BASE)) bones[RIGHT_MIDDLE_BASE].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_MIDDLE_BASE_INDEX);
        if (bones.ContainsKey(LEFT_INDEX_BASE)) bones[LEFT_INDEX_BASE].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_INDEX_BASE_INDEX);
        if (bones.ContainsKey(RIGHT_INDEX_BASE)) bones[RIGHT_INDEX_BASE].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_INDEX_BASE_INDEX);
        if (bones.ContainsKey(LEFT_PINKY_BASE)) bones[LEFT_PINKY_BASE].KeypointPosition = GetKeypointOrZero(keypoints3D, LEFT_PINKY_BASE_INDEX);
        if (bones.ContainsKey(RIGHT_PINKY_BASE)) bones[RIGHT_PINKY_BASE].KeypointPosition = GetKeypointOrZero(keypoints3D, RIGHT_PINKY_BASE_INDEX);


    }

    // Helper function to safely get keypoint or return Vector3.zero if index is out of bounds or keypoint is null
    Vector3 GetKeypointOrZero(Vector3[] keypoints, int index)
    {
        if (keypoints != null && index >= 0 && index < keypoints.Length && keypoints[index] != null)
        {
            return keypoints[index];
        }
        return Vector3.zero; // Return zero if keypoint is not available
    }

    // Helper function to get average keypoint position for cases like Hip (averaging left and right hip keypoints if available)
    Vector3 GetKeypointOrZero(Vector3[] keypoints, int index1, int index2)
    {
        Vector3 pos1 = GetKeypointOrZero(keypoints, index1);
        Vector3 pos2 = GetKeypointOrZero(keypoints, index2);
        if (pos1 != Vector3.zero && pos2 != Vector3.zero)
        {
            return (pos1 + pos2) * 0.5f;
        }
        else if (pos1 != Vector3.zero)
        {
            return pos1;
        }
        else if (pos2 != Vector3.zero)
        {
            return pos2;
        }
        return Vector3.zero;
    }
    // --- 5. Hand Rotation Helper Function ---
    void UpdateHandRotation(string handBoneName, string indexBaseBoneName, string middleBaseBoneName)
    {
        if (bones.ContainsKey(handBoneName) && bones.ContainsKey(indexBaseBoneName) && bones.ContainsKey(middleBaseBoneName))
        {
            Bone handBone = bones[handBoneName];
            Bone indexFingerBone = bones[indexBaseBoneName];
            Bone middleFingerBone = bones[middleBaseBoneName];

            if (handBone.Transform != null && indexFingerBone.KeypointPosition != Vector3.zero && middleFingerBone.KeypointPosition != Vector3.zero)
            {
                Vector3 handForward = Vector3.zero;
                if (indexFingerBone.Transform.name.Contains("Left"))
                {
                    handForward = TriangleNormal(handBone.KeypointPosition, indexFingerBone.KeypointPosition, middleFingerBone.KeypointPosition); // Hand forward using keypoints

                }
                else
                {
                    handForward = TriangleNormal(handBone.KeypointPosition, middleFingerBone.KeypointPosition, indexFingerBone.KeypointPosition); // Hand forward using keypoints

                }
                Quaternion targetHandRotation = Quaternion.LookRotation(handForward, middleFingerBone.KeypointPosition - handBone.KeypointPosition); // no inverse currectnly
               
                if (!float.IsNaN(targetHandRotation.x) && !float.IsNaN(targetHandRotation.y) && !float.IsNaN(targetHandRotation.z) && !float.IsNaN(targetHandRotation.w)) // NaN check
                {
                    handBone.Transform.rotation = Quaternion.Slerp(handBone.Transform.rotation, targetHandRotation, smoothing); // Apply smoothing
                }
                else
                {
                    Debug.LogWarning($"Invalid rotation (NaN) detected for bone: {handBoneName} (Hand Rotation). Skipping rotation update.");
                }

                //Debug.DrawRay(handBone.Transform.position, handForward, Color.yellow);

            }
        }
    }



}