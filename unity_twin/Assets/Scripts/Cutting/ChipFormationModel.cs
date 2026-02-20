using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Chip formation data output.
    /// </summary>
    public struct ChipData
    {
        public float chipThicknessRatio;    // r = t1/t2 (typically 0.3-0.5 for Al)
        public float shearAngle;            // φ (degrees)
        public float chipCurlRadius;        // Rc (mm)
        public float chipVelocity;          // Vc (m/min)
        public bool isContinuous;           // true for Al at normal speeds
        public float chipThickness;         // t2 (mm)
        public float shearStrainRate;       // γ̇ (1/s)
    }

    /// <summary>
    /// Chip formation model based on Merchant's minimum energy theory.
    ///
    /// Merchant's relation: φ = π/4 - β/2 + α/2
    /// where β = arctan(μ) is the friction angle and α is the rake angle.
    ///
    /// For 6061-T6 Al with HSS: μ ≈ 0.4, α = 10° rake
    /// → φ ≈ 29.1° (shear angle)
    /// → r ≈ 0.46 (chip compression ratio)
    /// → Rc ≈ 4.8mm (chip curl radius)
    ///
    /// Reference: Cutting_Physics_Digital_Twin_Reference.pdf Section 2
    /// </summary>
    public static class ChipFormationModel
    {
        // Al/HSS friction coefficient
        private const float DefaultFrictionCoeff = 0.4f;
        // Default rake angle (degrees)
        private const float DefaultRakeAngle = 10f;

        /// <summary>
        /// Calculate chip formation parameters.
        /// </summary>
        /// <param name="V_mpm">Cutting speed (m/min)</param>
        /// <param name="fz_mm">Feed per tooth (mm)</param>
        /// <param name="D_mm">Tool diameter (mm)</param>
        /// <param name="frictionCoeff">Friction coefficient (default 0.4 for Al/HSS)</param>
        /// <param name="rakeAngleDeg">Rake angle in degrees (default 10°)</param>
        public static ChipData Calculate(float V_mpm, float fz_mm, float D_mm,
            float frictionCoeff = DefaultFrictionCoeff, float rakeAngleDeg = DefaultRakeAngle)
        {
            float alpha = rakeAngleDeg * Mathf.Deg2Rad;
            float beta = Mathf.Atan(frictionCoeff);

            // Merchant's shear angle
            float phi = Mathf.PI / 4f - beta / 2f + alpha / 2f;

            // Chip thickness ratio (compression ratio)
            float r = Mathf.Sin(phi) / Mathf.Cos(phi - alpha);

            // Uncut and actual chip thickness
            float t1 = fz_mm;              // Uncut chip thickness ≈ feed per tooth
            float t2 = t1 / Mathf.Max(r, 0.01f);  // Actual chip thickness

            // Chip curl radius (from Cutting_Physics_Digital_Twin_Reference.pdf §2.4)
            // Rc = (t2 · D) / (4 · t1) · (1 + 2·t1/D)
            float Rc = (t2 * D_mm) / (4f * t1) * (1f + 2f * t1 / D_mm);
            Rc = Mathf.Clamp(Rc, 1f, 50f); // Physical bounds

            // Chip velocity (velocity diagram)
            float Vc = V_mpm * Mathf.Sin(phi) / Mathf.Cos(phi - alpha);

            // Shear strain rate
            float shearZoneWidth = 0.025f; // mm (typical)
            float Vs = V_mpm * Mathf.Cos(alpha) / (60f * Mathf.Cos(phi - alpha)); // m/s
            float strainRate = Vs / (shearZoneWidth / 1000f);

            // Chip type: continuous for Al at normal speeds and feeds
            bool isContinuous = V_mpm > 50f && fz_mm < 0.15f;

            return new ChipData
            {
                chipThicknessRatio = r,
                shearAngle = phi * Mathf.Rad2Deg,
                chipCurlRadius = Rc,
                chipVelocity = Vc,
                isContinuous = isContinuous,
                chipThickness = t2,
                shearStrainRate = strainRate
            };
        }
    }
}
