using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Surface roughness prediction model.
    ///
    /// Kinematic roughness: Ra = fz² / (32 · rε)
    /// where fz = feed per tooth, rε = tool nose radius.
    ///
    /// With Brammertz correction for minimum chip thickness:
    /// Ra_corrected = Ra_kinematic + rε · (1 - cos(arcsin(hmin / (2·rε))))
    ///
    /// Vibration-superimposed roughness adds amplitude from chatter detection.
    /// </summary>
    public static class SurfaceRoughnessModel
    {
        /// <summary>Tool nose radius for standard 1/4" HSS end mill (mm).</summary>
        private const float DefaultNoseRadius = 0.4f;

        /// <summary>Minimum chip thickness ratio (fraction of edge radius).</summary>
        private const float MinChipThicknessRatio = 0.3f;

        /// <summary>
        /// Calculate arithmetic mean roughness Ra.
        /// </summary>
        /// <param name="fz_mm">Feed per tooth (mm)</param>
        /// <param name="noseRadius_mm">Tool nose radius (mm)</param>
        /// <param name="vibrationAmplitude_um">Vibration amplitude (µm, from chatter)</param>
        /// <param name="flankWearVB_mm">Flank wear VB (mm) for ploughing roughness contribution</param>
        /// <returns>Ra in micrometers (µm)</returns>
        public static float CalculateRa(float fz_mm, float noseRadius_mm = DefaultNoseRadius,
            float vibrationAmplitude_um = 0f, float flankWearVB_mm = 0f)
        {
            if (fz_mm <= 0 || noseRadius_mm <= 0)
                return 0f;

            // Kinematic roughness: Ra = fz² / (32 · rε)
            // Result in mm, convert to µm (* 1000)
            float Ra_kinematic = (fz_mm * fz_mm) / (32f * noseRadius_mm) * 1000f;

            // Brammertz correction for minimum chip thickness effect
            float edgeRadius = 0.01f; // mm (typical for HSS)
            float hmin = MinChipThicknessRatio * edgeRadius;
            float sinArg = Mathf.Clamp(hmin / (2f * noseRadius_mm), 0f, 1f);
            float Ra_brammertz = noseRadius_mm * (1f - Mathf.Cos(Mathf.Asin(sinArg))) * 1000f;

            // Total roughness
            float Ra = Ra_kinematic + Ra_brammertz;

            // Add vibration contribution (superimposed)
            if (vibrationAmplitude_um > 0)
            {
                Ra += vibrationAmplitude_um * 0.5f; // Approximate
            }

            // Wear-roughness coupling: flank wear ploughing effect
            // Empirical relationship: 8 * VB^1.2 µm roughness contribution
            // Active when VB exceeds initial edge prep (> 0.02 mm)
            if (flankWearVB_mm > 0.02f)
            {
                Ra += 8f * Mathf.Pow(flankWearVB_mm, 1.2f);
            }

            return Mathf.Max(Ra, 0.01f); // Minimum physical Ra
        }

        /// <summary>Calculate Rz (peak-to-valley roughness) from Ra.</summary>
        public static float CalculateRz(float Ra_um)
        {
            // Typical ratio Rz/Ra ≈ 4-6 for milling
            return Ra_um * 5f;
        }

        /// <summary>Get surface quality grade based on Ra value.</summary>
        public static string GetSurfaceGrade(float Ra_um)
        {
            if (Ra_um <= 0.1f) return "N1 (Mirror)";
            if (Ra_um <= 0.2f) return "N2 (Fine ground)";
            if (Ra_um <= 0.4f) return "N3 (Ground)";
            if (Ra_um <= 0.8f) return "N4 (Fine milled)";
            if (Ra_um <= 1.6f) return "N5 (Milled)";
            if (Ra_um <= 3.2f) return "N6 (Rough milled)";
            return "N7+ (Coarse)";
        }
    }
}
