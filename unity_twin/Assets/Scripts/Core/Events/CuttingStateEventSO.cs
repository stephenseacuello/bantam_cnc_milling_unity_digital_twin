using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>Cutting simulation state data passed between subsystems.</summary>
    [System.Serializable]
    public struct CuttingStateData
    {
        public float spindleRPM;
        public float feedRate;
        public UnityEngine.Vector3 toolPosition;
        public UnityEngine.Vector3 toolDirection;
        public float axialDepth;
        public float radialDepth;
        public float toolDiameter;
        public bool isCutting;
        public int currentGCodeLine;

        // Force outputs
        public float forceFx, forceFy, forceFz;
        public float powerWatts;
        public float torqueNm;
        public float mrr;

        // Thermal outputs
        public float toolTemperature;
        public float interfaceTemperature;

        // Wear outputs
        public float flankWearVB;
        public float wearPercentage;

        // Chip outputs
        public float chipThicknessRatio;
        public float shearAngleDeg;
        public float chipCurlRadius;
        public float chipVelocity;

        // Surface roughness outputs
        public float surfaceRoughnessRa;  // µm
        public float surfaceRoughnessRz;  // µm
        public string surfaceGrade;       // ISO N-grade (e.g. "N4 (Fine milled)")
    }

    [CreateAssetMenu(menuName = "MIRACLE/Events/Cutting State")]
    public class CuttingStateEventSO : GameEventSO<CuttingStateData> { }
}
