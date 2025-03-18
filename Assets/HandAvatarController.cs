using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Handles finger animation based on hand tracking data, compatible with AvatarController bones
/// </summary>
public class FingerAnimatorLite : MonoBehaviour
{
    [Header("References")]
    public HandDetection handDetection;
    public AvatarController avatarController;

    [Header("Animation Settings")]
    [Range(0f, 1f)]
    public float smoothing = 0.5f;

    [Header("Debug")]
    public bool showDebugGizmos = false;
    public bool logDebugInfo = false;

    // Finger bone references
    private Dictionary<string, FingerChain> fingerChains = new Dictionary<string, FingerChain>();

    // Class to store finger joint transforms and inverse rotations
    [System.Serializable]
    public class FingerChain
    {
        public Transform proximal;      // first joint (e.g. Index1)
        public Transform intermediate;  // second joint (e.g. Index2)
        public Transform distal;        // third joint (e.g. Index3)
        public Transform tip;       // fourth joint (e.g. Index4)

        public Quaternion proximalInverse;
        public Quaternion intermediateInverse;
        public Quaternion distalInverse;
        public Quaternion tipInverse;

        public Quaternion proximalInit;
        public Quaternion intermediateInit;
        public Quaternion distalInit;
        public Quaternion tipInit;

    }

    private void Start()
    {
        if (avatarController == null || handDetection == null)
        {
            Debug.LogError("Missing required references!");
            return;
        }

        StartCoroutine(InitializeWhenReady());
    }

    private IEnumerator InitializeWhenReady()
    {
        // Wait until AvatarController has initialized its bones
        while (avatarController.Bones == null || avatarController.Bones.Count == 0)
        {
            yield return null;
        }

        // Find and cache all finger joints
        FindFingerJoints();

        // Set up inverse rotations
        SetupInverseRotations();

        if (logDebugInfo)
        {
            // Log the found finger joints
            foreach (var entry in fingerChains)
            {
                Debug.Log($"Finger: {entry.Key}, " +
                          $"Proximal: {(entry.Value.proximal != null ? entry.Value.proximal.name : "Missing")}, " +
                          $"Intermediate: {(entry.Value.intermediate != null ? entry.Value.intermediate.name : "Missing")}, " +
                          $"Distal: {(entry.Value.distal != null ? entry.Value.distal.name : "Missing")}");
            }
        }
    }

    private void FindFingerJoints()
    {
        var bones = avatarController.Bones;

        // Initialize finger chains for each finger
        string[] fingers = new[] {
            "RightThumb", "RightIndex", "RightMiddle", "RightRing", "RightPinky",
            "LeftThumb", "LeftIndex", "LeftMiddle", "LeftRing", "LeftPinky"
        };

        foreach (string finger in fingers)
        {
            fingerChains[finger] = new FingerChain();
        }

        // Find the base finger joints in AvatarController's bones
        if (bones.TryGetValue("mixamorig:RightHandThumb1", out var bone)) fingerChains["RightThumb"].proximal = bone.Transform;
        if (bones.TryGetValue("mixamorig:RightHandIndex1", out bone)) fingerChains["RightIndex"].proximal = bone.Transform;
        if (bones.TryGetValue("mixamorig:RightHandMiddle1", out bone)) fingerChains["RightMiddle"].proximal = bone.Transform;
        if (bones.TryGetValue("mixamorig:RightHandRing1", out bone)) fingerChains["RightRing"].proximal = bone.Transform;
        if (bones.TryGetValue("mixamorig:RightHandPinky1", out bone)) fingerChains["RightPinky"].proximal = bone.Transform;

        if (bones.TryGetValue("mixamorig:LeftHandThumb1", out bone)) fingerChains["LeftThumb"].proximal = bone.Transform;
        if (bones.TryGetValue("mixamorig:LeftHandIndex1", out bone)) fingerChains["LeftIndex"].proximal = bone.Transform;
        if (bones.TryGetValue("mixamorig:LeftHandMiddle1", out bone)) fingerChains["LeftMiddle"].proximal = bone.Transform;
        if (bones.TryGetValue("mixamorig:LeftHandRing1", out bone)) fingerChains["LeftRing"].proximal = bone.Transform;
        if (bones.TryGetValue("mixamorig:LeftHandPinky1", out bone)) fingerChains["LeftPinky"].proximal = bone.Transform;

        // Find intermediate and distal joints by searching the hierarchy
        foreach (string finger in fingers)
        {
            FingerChain chain = fingerChains[finger];
            if (chain.proximal != null)
            {
                // Find intermediate joint (should be first child of proximal)
                for (int i = 0; i < chain.proximal.childCount; i++)
                {
                    Transform child = chain.proximal.GetChild(i);
                    if (child.name.Contains("2") || child.name.Contains("Intermediate"))
                    {
                        chain.intermediate = child;
                        break;
                    }
                }

                if (chain.intermediate != null)
                {
                    for (int i = 0; i < chain.intermediate.childCount; i++)
                    {
                        Transform child = chain.intermediate.GetChild(i);
                        if (child.name.Contains("3") || child.name.Contains("Distal"))
                        {
                            chain.distal = child;

                            // Find tip joint (should be first child of distal)
                            if (chain.distal != null)
                            {
                                for (int j = 0; j < chain.distal.childCount; j++)
                                {
                                    Transform tipChild = chain.distal.GetChild(j);
                                    if (tipChild.name.Contains("4") || tipChild.name.Contains("Tip"))
                                    {
                                        chain.tip = tipChild;
                                        break;
                                    }
                                }
                            }

                            break;
                        }
                    }
                }
            }
        }

        // Store initial rotations
        foreach (var chain in fingerChains.Values)
        {
            if (chain.proximal != null) chain.proximalInit = chain.proximal.rotation;
            if (chain.intermediate != null) chain.intermediateInit = chain.intermediate.rotation;
            if (chain.distal != null) chain.distalInit = chain.distal.rotation;
            if (chain.tip != null) chain.tipInit = chain.tip.rotation;
        }
    }

    private void SetupInverseRotations()
    {
        // Get wrist and palm normal reference
        Transform rightWrist = null, leftWrist = null;
        if (avatarController.Bones.TryGetValue("mixamorig:RightHand", out var bone)) rightWrist = bone.Transform;
        if (avatarController.Bones.TryGetValue("mixamorig:LeftHand", out bone)) leftWrist = bone.Transform;

        // For each finger, calculate inverse rotations
        foreach (var entry in fingerChains)
        {
            string fingerName = entry.Key;
            FingerChain chain = entry.Value;

            // Skip if missing important transforms
            if (chain.proximal == null || chain.intermediate == null) continue;

            // Select appropriate wrist
            Transform wrist = fingerName.StartsWith("Right") ? rightWrist : leftWrist;
            if (wrist == null) continue;

            // Get reference normal (palm normal)
            Vector3 palmNormal;
            if (fingerName.StartsWith("Right"))
            {
                // For right hand: use normal from wrist, index, pinky
                palmNormal = TriangleNormal(
                    wrist.position,
                    fingerChains["RightIndex"].proximal.position,
                    fingerChains["RightPinky"].proximal.position
                );
            }
            else
            {
                // For left hand: use normal from wrist, pinky, index
                palmNormal = TriangleNormal(
                    wrist.position,
                    fingerChains["LeftPinky"].proximal.position,
                    fingerChains["LeftIndex"].proximal.position
                );
            }

            // Calculate joint directions in rest pose
            Vector3 proxToInter = chain.proximal.position - chain.intermediate.position;

            // Set inverse rotations to maintain original pose
            chain.proximalInverse = Quaternion.Inverse(
                Quaternion.LookRotation(proxToInter, palmNormal)
            ) * chain.proximalInit;

            if (chain.distal != null && chain.tip != null)
            {
                Vector3 interToDist = chain.intermediate.position - chain.distal.position;
                chain.intermediateInverse = Quaternion.Inverse(
                    Quaternion.LookRotation(interToDist, palmNormal)
                ) * chain.intermediateInit;

                // For distal (last joint) we have to estimate a direction
                Vector3 distToEnd = interToDist.normalized; // Assume same direction as previous segment
                chain.distalInverse = Quaternion.Inverse(
                    Quaternion.LookRotation(distToEnd, palmNormal)
                ) * chain.distalInit;

                Vector3 tipDir = interToDist.normalized;
                chain.tipInverse = Quaternion.Inverse(
                    Quaternion.LookRotation(tipDir, palmNormal)
                ) * chain.tipInit;

            }
        }
    }

    private void Update()
    {
        if (handDetection == null) return;

        // Update right hand fingers if detected
        if (handDetection.rightHandDetected)
        {
            UpdateFingerRotations(true);
        }

        // Update left hand fingers if detected
        if (handDetection.leftHandDetected)
        {
            UpdateFingerRotations(false);
        }

        // Visual debug
        if (showDebugGizmos)
        {
            VisualizeFingerBones();
        }
    }

    private void UpdateFingerRotations(bool isRightHand)
    {
        // Common palm normal reference
        Vector3 wristPos = isRightHand ?
            handDetection.GetRightHandKeypointPosition(HandDetection.HandKeypoint.Wrist) :
            handDetection.GetLeftHandKeypointPosition(HandDetection.HandKeypoint.Wrist);

        Vector3 indexMcpPos = isRightHand ?
            handDetection.GetRightHandKeypointPosition(HandDetection.HandKeypoint.IndexMCP) :
            handDetection.GetLeftHandKeypointPosition(HandDetection.HandKeypoint.IndexMCP);

        Vector3 pinkyMcpPos = isRightHand ?
            handDetection.GetRightHandKeypointPosition(HandDetection.HandKeypoint.PinkyMCP) :
            handDetection.GetLeftHandKeypointPosition(HandDetection.HandKeypoint.PinkyMCP);

        if (wristPos == Vector3.zero || indexMcpPos == Vector3.zero || pinkyMcpPos == Vector3.zero)
            return;

        // Calculate palm normal based on hand
        Vector3 palmNormal;
        if (isRightHand)
        {
            palmNormal = TriangleNormal(wristPos, indexMcpPos, pinkyMcpPos);
        }
        else
        {
            palmNormal = TriangleNormal(wristPos, pinkyMcpPos, indexMcpPos);
        }

        // Process each finger
        string[] fingers = isRightHand ?
            new[] { "RightThumb", "RightIndex", "RightMiddle", "RightRing", "RightPinky" } :
            new[] { "LeftThumb", "LeftIndex", "LeftMiddle", "LeftRing", "LeftPinky" };

        foreach (string fingerName in fingers)
        {
            FingerChain chain = fingerChains[fingerName];
            if (chain.proximal == null || chain.intermediate == null) continue;

            // Map finger name to keypoints
            HandDetection.HandKeypoint mcpKeypoint, pipKeypoint, dipKeypoint, tipKeypoint;

            switch (fingerName)
            {
                case "RightThumb":
                case "LeftThumb":
                    mcpKeypoint = HandDetection.HandKeypoint.ThumbCMC;
                    pipKeypoint = HandDetection.HandKeypoint.ThumbMCP;
                    dipKeypoint = HandDetection.HandKeypoint.ThumbIP;
                    tipKeypoint = HandDetection.HandKeypoint.ThumbTip;
                    break;

                case "RightIndex":
                case "LeftIndex":
                    mcpKeypoint = HandDetection.HandKeypoint.IndexMCP;
                    pipKeypoint = HandDetection.HandKeypoint.IndexPIP;
                    dipKeypoint = HandDetection.HandKeypoint.IndexDIP;
                    tipKeypoint = HandDetection.HandKeypoint.IndexTip;
                    break;

                case "RightMiddle":
                case "LeftMiddle":
                    mcpKeypoint = HandDetection.HandKeypoint.MiddleMCP;
                    pipKeypoint = HandDetection.HandKeypoint.MiddlePIP;
                    dipKeypoint = HandDetection.HandKeypoint.MiddleDIP;
                    tipKeypoint = HandDetection.HandKeypoint.MiddleTip;
                    break;

                case "RightRing":
                case "LeftRing":
                    mcpKeypoint = HandDetection.HandKeypoint.RingMCP;
                    pipKeypoint = HandDetection.HandKeypoint.RingPIP;
                    dipKeypoint = HandDetection.HandKeypoint.RingDIP;
                    tipKeypoint = HandDetection.HandKeypoint.RingTip;
                    break;

                case "RightPinky":
                case "LeftPinky":
                    mcpKeypoint = HandDetection.HandKeypoint.PinkyMCP;
                    pipKeypoint = HandDetection.HandKeypoint.PinkyPIP;
                    dipKeypoint = HandDetection.HandKeypoint.PinkyDIP;
                    tipKeypoint = HandDetection.HandKeypoint.PinkyTip;
                    break;

                default:
                    continue;
            }

            // Get keypoint positions
            Vector3 mcpPos, pipPos, dipPos, tipPos;

            if (isRightHand)
            {
                mcpPos = handDetection.GetRightHandKeypointPosition(mcpKeypoint);
                pipPos = handDetection.GetRightHandKeypointPosition(pipKeypoint);
                dipPos = handDetection.GetRightHandKeypointPosition(dipKeypoint);
                tipPos = handDetection.GetRightHandKeypointPosition(tipKeypoint);
            }
            else
            {
                mcpPos = handDetection.GetLeftHandKeypointPosition(mcpKeypoint);
                pipPos = handDetection.GetLeftHandKeypointPosition(pipKeypoint);
                dipPos = handDetection.GetLeftHandKeypointPosition(dipKeypoint);
                tipPos = handDetection.GetLeftHandKeypointPosition(tipKeypoint);
            }

            // Skip if missing keypoints
            if (mcpPos == Vector3.zero || pipPos == Vector3.zero || dipPos == Vector3.zero || tipPos == Vector3.zero)
            {
                if (logDebugInfo)
                {
                    Debug.Log($"Missing keypoints for {fingerName}: MCP={mcpPos != Vector3.zero}, PIP={pipPos != Vector3.zero}, DIP={dipPos != Vector3.zero}, TIP={tipPos != Vector3.zero}");
                }
                continue;
            }

            if (logDebugInfo)
            {
                Debug.Log($"Applying rotations to {fingerName}");
            }

            // Apply rotation to proximal joint (MCP)
            Vector3 mcpToPip = (mcpPos - pipPos).normalized;
            Quaternion proximalTarget = Quaternion.LookRotation(mcpToPip, palmNormal) * chain.proximalInverse;
            chain.proximal.rotation = Quaternion.Slerp(chain.proximal.rotation, proximalTarget, smoothing);

            // Apply rotation to intermediate joint (PIP)
            if (chain.intermediate != null)
            {
                Vector3 pipToDip = (pipPos - dipPos).normalized;
                Quaternion intermediateTarget = Quaternion.LookRotation(pipToDip, palmNormal) * chain.intermediateInverse;
                chain.intermediate.rotation = Quaternion.Slerp(chain.intermediate.rotation, intermediateTarget, smoothing);
            }

            // Apply rotation to distal joint (DIP)
            if (chain.distal != null)
            {
                Vector3 dipToTip = (dipPos - tipPos).normalized;
                Quaternion distalTarget = Quaternion.LookRotation(dipToTip, palmNormal) * chain.distalInverse;
                chain.distal.rotation = Quaternion.Slerp(chain.distal.rotation, distalTarget, smoothing);
            }
            if (chain.tip != null)
            {
                // The tip usually just follows the direction of the distal joint
                // We can use the same direction or a slight variation
                Vector3 dipToTip = (dipPos - tipPos).normalized;

                Quaternion tipTarget = Quaternion.LookRotation(dipToTip, palmNormal) * chain.tipInverse;
                chain.tip.rotation = Quaternion.Slerp(chain.tip.rotation, tipTarget, smoothing);
            }
        }
    }

    private Vector3 TriangleNormal(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 d1 = a - b;
        Vector3 d2 = a - c;
        Vector3 normal = Vector3.Cross(d1, d2);
        normal.Normalize();
        return normal;
    }

    private void VisualizeFingerBones()
    {
        foreach (var entry in fingerChains)
        {
            FingerChain chain = entry.Value;

            if (chain.proximal != null)
            {
                Debug.DrawRay(chain.proximal.position, chain.proximal.forward * 0.03f, Color.blue);

                if (chain.intermediate != null)
                {
                    Debug.DrawLine(chain.proximal.position, chain.intermediate.position, Color.yellow);
                    Debug.DrawRay(chain.intermediate.position, chain.intermediate.forward * 0.02f, Color.blue);

                    if (chain.distal != null)
                    {
                        Debug.DrawLine(chain.intermediate.position, chain.distal.position, Color.yellow);
                        Debug.DrawRay(chain.distal.position, chain.distal.forward * 0.01f, Color.blue);
                    }
                }
            }
        }
    }
}