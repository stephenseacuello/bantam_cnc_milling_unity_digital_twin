using UnityEngine;

namespace MiracleTwin.CNC
{
    [CreateAssetMenu(menuName = "MIRACLE/CNC/Machine Profile")]
    public class CNCMachineProfileSO : ScriptableObject
    {
        [Header("Identity")]
        public string machineName = "Unknown CNC";
        public string rosTopicId = "cnc1";

        [Header("Work Envelope (mm)")]
        public Vector3 minPosition = Vector3.zero;
        public Vector3 maxPosition = new(152.4f, 101.6f, 69.85f);

        [Header("Spindle")]
        public float minRPM = 0f;
        public float maxRPM = 23000f;
        public string spindleType = "BLDC";
        public string colletType = "ER-11";

        [Header("Physical")]
        public Vector3 overallDimensionsMM = new(400f, 387f, 260f);
        public float weightKg = 15.4f;

        [Header("Workpiece Defaults")]
        public Vector3 defaultWorkpieceSizeMM = new(76.2f, 50.8f, 76.2f);
        public string defaultMaterial = "6061-T6 Aluminum";
    }
}
