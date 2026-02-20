using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Three-stage flank wear model with Taylor tool life prediction.
    ///
    /// Stages:
    /// 1. Break-in (rapid initial wear, ~2 min)
    /// 2. Steady-state (linear wear, main cutting life)
    /// 3. Accelerated (exponential, approaching failure)
    ///
    /// Taylor equation: V · T^n · f^a · ap^b = C
    /// For 6061-T6 Al + HSS: n=0.125, a=0.5, b=0.15, C=300
    ///
    /// Wear-force coupling: Kte_w = Kte · (1 + kwear · VB)
    /// kwear ≈ 12.5 mm⁻¹ (from Cutting_Physics_Digital_Twin_Reference.pdf §4)
    /// </summary>
    public class ToolWearModel
    {
        // Three-stage model parameters
        private const float VB0 = 0.02f;       // Initial wear after edge prep (mm)
        private const float t1 = 2.0f;         // End of break-in period (minutes)
        private const float VB1 = 0.08f;        // Wear at end of break-in (mm)
        private const float C2 = 0.004f;        // Steady-state wear rate (mm/min at V=100)
        private const float VBmax = 0.30f;       // End-of-life criterion (mm)
        private const float VBhard = 0.50f;      // Hard cap (mm)

        // Taylor equation constants (6061-T6 + HSS)
        private const float TaylorN = 0.125f;
        private const float TaylorA = 0.5f;
        private const float TaylorB = 0.15f;
        private const float TaylorC = 300f;
        private const float ReferenceSpeed = 100f; // m/min

        // State
        public float VB { get; private set; } = VB0;
        public float CuttingTime { get; private set; }
        public float WearPercentage => VB / VBmax * 100f;
        public float RemainingLifeMinutes => EstimateRemainingLife();
        public bool IsEndOfLife => VB >= VBmax;
        public int CurrentStage { get; private set; } = 1;

        public string RecommendedAction
        {
            get
            {
                if (VB < 0.10f) return "CONTINUE";
                if (VB < 0.15f) return "MONITOR";
                if (VB < 0.25f) return "PLAN_REPLACEMENT";
                return "REPLACE_NOW";
            }
        }

        /// <summary>
        /// Update wear state for one time step while cutting.
        /// </summary>
        /// <param name="V_mpm">Cutting speed (m/min)</param>
        /// <param name="fz_mm">Feed per tooth (mm)</param>
        /// <param name="ap_mm">Axial depth of cut (mm)</param>
        /// <param name="dt_seconds">Time step (seconds)</param>
        /// <param name="toolTemperature_C">Tool temperature for Arrhenius-inspired thermal coupling (°C)</param>
        public void Update(float V_mpm, float fz_mm, float ap_mm, float dt_seconds, float toolTemperature_C = 20f)
        {
            if (dt_seconds <= 0 || V_mpm <= 0) return;

            float dt_min = dt_seconds / 60f;
            CuttingTime += dt_min;

            // Speed-adjusted wear rate factor
            float speedFactor = V_mpm / ReferenceSpeed;

            // Arrhenius-inspired thermal acceleration factor
            // Linear approximation valid for HSS in aluminum (20-200°C range)
            // thermalFactor = 1 + 0.035 * max(0, T - 20), clamped to [1, 3]
            float thermalFactor = Mathf.Clamp(1f + 0.035f * Mathf.Max(0f, toolTemperature_C - 20f), 1f, 3f);

            if (CuttingTime < t1)
            {
                // Stage 1: Break-in (rapid initial wear)
                CurrentStage = 1;
                VB = VB0 + (VB1 - VB0) * Mathf.Sqrt(CuttingTime / t1);
            }
            else if (VB < 0.25f)
            {
                // Stage 2: Steady-state (linear wear) — accelerated by temperature
                CurrentStage = 2;
                float rate = C2 * speedFactor * thermalFactor;
                // Feed and depth influence
                rate *= Mathf.Pow(Mathf.Max(fz_mm, 0.01f) / 0.05f, 0.3f);
                rate *= Mathf.Pow(Mathf.Max(ap_mm, 0.1f) / 1.0f, 0.1f);
                VB += rate * dt_min;
            }
            else
            {
                // Stage 3: Accelerated (exponential) — thermal factor amplifies catastrophic wear
                CurrentStage = 3;
                float C3 = 0.1f * speedFactor * thermalFactor;
                VB += VB * C3 * dt_min;
            }

            VB = Mathf.Min(VB, VBhard);
        }

        /// <summary>Estimate remaining tool life using Taylor equation.</summary>
        private float EstimateRemainingLife()
        {
            if (IsEndOfLife) return 0f;

            // Simplified: remaining = estimated_total - elapsed
            // Taylor total life at reference speed: T = (C / V)^(1/n)
            float totalLife = Mathf.Pow(TaylorC / Mathf.Max(ReferenceSpeed, 1f),
                                        1f / TaylorN);
            return Mathf.Max(0, totalLife - CuttingTime);
        }

        /// <summary>Get Taylor tool life prediction for given conditions.</summary>
        public float TaylorLifePrediction(float V_mpm, float fz_mm, float ap_mm)
        {
            // V · T^n · f^a · ap^b = C
            // T = (C / (V · f^a · ap^b))^(1/n)
            float denominator = V_mpm *
                                Mathf.Pow(Mathf.Max(fz_mm, 0.001f), TaylorA) *
                                Mathf.Pow(Mathf.Max(ap_mm, 0.01f), TaylorB);
            if (denominator <= 0) return float.MaxValue;
            return Mathf.Pow(TaylorC / denominator, 1f / TaylorN);
        }

        /// <summary>Reset wear to initial state.</summary>
        public void Reset()
        {
            VB = VB0;
            CuttingTime = 0;
            CurrentStage = 1;
        }
    }
}
