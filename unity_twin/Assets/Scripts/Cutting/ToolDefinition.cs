using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// End mill tool geometry and cutting coefficient definition.
    /// Configurable for different tools used in the Bantam Explorer.
    /// </summary>
    [CreateAssetMenu(menuName = "MIRACLE/Tool Definition")]
    public class ToolDefinition : ScriptableObject
    {
        [Header("Identity")]
        public string toolId = "HSS_quarter_inch_2flute";
        public string description = "1/4\" 2-Flute HSS End Mill";

        [Header("Geometry")]
        public float diameter = 6.35f;         // mm (1/4")
        public int fluteCount = 2;
        public float helixAngle = 30f;         // degrees
        public float rakeAngle = 10f;          // degrees
        public float reliefAngle = 12f;        // degrees
        public float noseRadius = 0.4f;        // mm
        public float cuttingLength = 19f;      // mm
        public float overallLength = 57f;      // mm
        public float shankDiameter = 6.35f;    // mm

        [Header("Collet")]
        public string colletType = "ER-11";
        public float maxRunout = 0.01f;        // mm TIR

        [Header("Operating Limits (Bantam Explorer)")]
        public float minRPM = 10000f;
        public float maxRPM = 23000f;
        public float recommendedRPM = 16000f;
        public float maxFeedRate = 2000f;      // mm/min
        public float recommendedFeedPerTooth = 0.05f; // mm
        public float maxAxialDepth = 3.0f;     // mm
        public float maxRadialDepth = 6.35f;   // mm (full width)

        [Header("Cutting Coefficients")]
        public float Ktc = 796f;
        public float Krc = 168f;
        public float Kac = 80f;
        public float Kte = 14.5f;
        public float Kre = 10.2f;
        public float Kae = 4.8f;

        [Header("Wear Properties")]
        public float taylorC = 300f;
        public float taylorN = 0.125f;
        public float initialWearVB = 0.02f;
        public float maxWearVB = 0.30f;

        /// <summary>Recommended feed per tooth for given material.</summary>
        public float GetRecommendedFeedPerTooth(string material)
        {
            return material switch
            {
                "6061-T6" => 0.05f,
                "7075-T6" => 0.04f,
                "Ti-6Al-4V" => 0.02f,
                _ => 0.03f
            };
        }

        /// <summary>Calculate chip load for given RPM and feed rate.</summary>
        public float CalculateChipLoad(float rpm, float feedRate_mmPerMin)
        {
            if (rpm <= 0 || fluteCount <= 0) return 0;
            return feedRate_mmPerMin / (rpm * fluteCount);
        }

        /// <summary>Calculate cutting speed from RPM.</summary>
        public float CalculateCuttingSpeed(float rpm)
        {
            return Mathf.PI * diameter * rpm / 1000f; // m/min
        }
    }
}
