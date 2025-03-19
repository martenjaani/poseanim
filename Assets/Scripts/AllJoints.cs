using System.IO;
using System.Text;
using UnityEngine;

public class AllJoints : MonoBehaviour
{
    StringBuilder jointList = new StringBuilder();

    void Start()
    {
        jointList.AppendLine("Joints for: " + gameObject.name);
        AppendChildren(transform, "");
        // Specify a path relative to the project or absolute
        string filePath = Path.Combine(Application.dataPath, "JointList.txt");
        File.WriteAllText(filePath, jointList.ToString());
        Debug.Log("Joint list saved to " + filePath);
    }

    void AppendChildren(Transform parent, string indent)
    {
        jointList.AppendLine(indent + parent.name);
        foreach (Transform child in parent)
        {
            AppendChildren(child, indent + "  ");
        }
    }
}
