using Newtonsoft.Json;
using UnityEngine;

public class bendiosc : MonoBehaviour
{
    public string OscTargetIp = "127.0.0.1";
    public int OscTargetPort = 9000;
    public Transform[] Trackers = new Transform[2]; // Assuming 2 trackers
    private OscCore.OscClient oscClient;

    private int currentIndex = 0;

    void Start()
    {
        oscClient = new OscCore.OscClient(OscTargetIp, OscTargetPort);
    }

    void Update()
    {
        if (oscClient == null)
            return;

        // Read and parse JSON file for leftjoint
        string leftJointFilePath = Application.dataPath + "/leftjoint.json";
        string leftJointJsonString = System.IO.File.ReadAllText(leftJointFilePath);
        JointData leftJointData = JsonConvert.DeserializeObject<JointData>(leftJointJsonString);

        // Read and parse JSON file for leftpose
        string leftPoseFilePath = Application.dataPath + "/leftpose.json";
        string leftPoseJsonString = System.IO.File.ReadAllText(leftPoseFilePath);
        PoseData leftPoseData = JsonConvert.DeserializeObject<PoseData>(leftPoseJsonString);

        // Read and parse JSON file for rightjoint
        string rightJointFilePath = Application.dataPath + "/rightjoint.json";
        string rightJointJsonString = System.IO.File.ReadAllText(rightJointFilePath);
        JointData rightJointData = JsonConvert.DeserializeObject<JointData>(rightJointJsonString);

        // Read and parse JSON file for rightpose
        string rightPoseFilePath = Application.dataPath + "/rightpose.json";
        string rightPoseJsonString = System.IO.File.ReadAllText(rightPoseFilePath);
        PoseData rightPoseData = JsonConvert.DeserializeObject<PoseData>(rightPoseJsonString);

        // Send rotation data over OSC for both trackers
        if (leftJointData != null && leftJointData.Data != null && rightJointData != null && rightJointData.Data != null &&
            currentIndex < leftJointData.Data.Length && currentIndex < rightJointData.Data.Length)
        {
            for (int i = 0; i < Trackers.Length; i++)
            {
                var jointRotation = (i == 0) ? leftJointData.Data[currentIndex] : rightJointData.Data[currentIndex];
                if (jointRotation.Length == 3)
                {
                    // Sending joint rotation data over OSC as a Vector3
                    oscClient.Send($"/tracking/trackers/{i + 1}/rotation", new Vector3(jointRotation[0], jointRotation[1], jointRotation[2]));
                }
            }
        }

        // Send position data over OSC for both trackers
        if (leftPoseData != null && leftPoseData.Data != null && rightPoseData != null && rightPoseData.Data != null &&
            currentIndex < leftPoseData.Data.Length && currentIndex < rightPoseData.Data.Length)
        {
            for (int i = 0; i < Trackers.Length; i++)
            {
                var posePosition = (i == 0) ? leftPoseData.Data[currentIndex] : rightPoseData.Data[currentIndex];
                if (posePosition.Length == 3)
                {
                    // Sending pose position data over OSC as a Vector3
                    oscClient.Send($"/tracking/trackers/{i + 1}/position", new Vector3(posePosition[0], posePosition[1], posePosition[2]));
                }
            }
        }

        // Increase index for the next frame
        currentIndex = (currentIndex + 1) % Mathf.Min(leftJointData.Data.Length, leftPoseData.Data.Length, rightJointData.Data.Length, rightPoseData.Data.Length);
    }

    // Define class to match JSON structure
    public class JointData
    {
        public float[][] Data;
    }

    public class PoseData
    {
        public float[][] Data;
    }
}
