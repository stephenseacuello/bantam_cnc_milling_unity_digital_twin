using UnityEngine;
using System;
using RosMessageTypes.Miracle;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Chatter risk levels for machining operations.
    /// </summary>
    public enum ChatterRisk
    {
        LOW,
        MEDIUM,
        HIGH
    }

    /// <summary>
    /// Analytical stability lobe predictor using Altintas zeroth-order approximation (ZOA).
    ///
    /// Computes the stability boundary: ap_lim = -1/(2·Ktc·N·Re[G(jω)])
    /// where G(jω) is the structural transfer function at chatter frequency ω.
    ///
    /// Modal parameters default to 1/4" HSS endmill in ER-11 collet.
    /// </summary>
    public class StabilityLobePredictor : MonoBehaviour
    {
        [Header("Modal Parameters (Tap-Test Calibration)")]
        [SerializeField] private float naturalFrequencyHz = 1800f;
        [SerializeField] private float dampingRatio = 0.03f;
        [SerializeField] private float stiffnessNpm = 8e6f;  // N/m

        [Header("Cutting Parameters")]
        [SerializeField] private float Ktc = 796f;   // N/mm² tangential cutting coefficient
        [SerializeField] private int fluteCount = 2;

        [Header("Risk Thresholds")]
        [Tooltip("Margin below stability limit for MEDIUM risk (fraction of ap_lim)")]
        [SerializeField] private float mediumRiskMargin = 0.8f;
        [Tooltip("Margin below stability limit for HIGH risk (fraction of ap_lim)")]
        [SerializeField] private float highRiskMargin = 0.95f;

        [Header("RPM Search")]
        [SerializeField] private float minRPM = 3000f;
        [SerializeField] private float maxRPM = 25000f;
        [SerializeField] private float rpmSearchStep = 50f;

        [Header("Tool Wear Adjustment")]
        [SerializeField] private float currentWearVB = 0f;           // Current flank wear in mm
        [SerializeField] private float wearStiffnessReduction = 0.15f; // % stiffness loss per 0.1mm wear
        [SerializeField] private float wearDampingIncrease = 0.05f;    // % damping increase per 0.1mm wear
        [SerializeField] private float maxWearVB = 0.3f;              // VBmax clamp (mm)

        // Cached lobe surface for visualization
        private float[,] lobeSurface; // [rpmIndex, depthIndex]
        private float[] lobeSurfaceRPMs;
        private float[] lobeSurfaceDepths;
        private bool lobeSurfaceDirty = true;

        /// <summary>Last evaluated chatter risk.</summary>
        public ChatterRisk LastRisk { get; private set; } = ChatterRisk.LOW;

        /// <summary>Last evaluated stability limit (mm).</summary>
        public float LastStabilityLimit { get; private set; }

        /// <summary>Current flank wear value (mm).</summary>
        public float CurrentWearVB => currentWearVB;

        /// <summary>
        /// Update tool wear and recompute stability limits.
        /// Marks the lobe surface cache as dirty so it will be recomputed on next access.
        /// </summary>
        public void UpdateToolWear(float wearVB_mm)
        {
            currentWearVB = Mathf.Max(0f, wearVB_mm);
            lobeSurfaceDirty = true;
        }

        /// <summary>
        /// Get wear-adjusted modal stiffness: k_eff = k * (1 - wearFactor * VB / 0.1).
        /// Wear reduces effective stiffness (tool becomes more compliant).
        /// Clamped so stiffness never drops below 10% of nominal.
        /// </summary>
        private float GetEffectiveStiffness(float wearVB)
        {
            float clampedVB = Mathf.Clamp(wearVB, 0f, maxWearVB);
            float factor = 1f - wearStiffnessReduction * (clampedVB / 0.1f);
            factor = Mathf.Max(factor, 0.1f); // Never below 10% of nominal
            return stiffnessNpm * factor;
        }

        /// <summary>
        /// Get wear-adjusted damping: zeta_eff = zeta * (1 + wearDampingIncrease * VB / 0.1).
        /// Wear increases effective damping slightly (contact area grows).
        /// </summary>
        private float GetEffectiveDamping(float wearVB)
        {
            float clampedVB = Mathf.Clamp(wearVB, 0f, maxWearVB);
            float factor = 1f + wearDampingIncrease * (clampedVB / 0.1f);
            return dampingRatio * factor;
        }

        /// <summary>
        /// Calculate the stability limit (ap_lim in mm) at a given RPM
        /// using the Altintas-Budak ZOA method, with wear-adjusted modal parameters.
        /// </summary>
        public float CalculateStabilityLimit(float rpm)
        {
            if (rpm <= 0) return float.MaxValue;

            float k_eff = GetEffectiveStiffness(currentWearVB);
            float zeta_eff = GetEffectiveDamping(currentWearVB);
            float omega_n = naturalFrequencyHz * 2f * Mathf.PI;
            float bestApLim = float.MaxValue;

            // Sweep chatter frequencies near natural frequency
            for (float ratio = 0.5f; ratio < 2.0f; ratio += 0.005f)
            {
                float omega_c = omega_n * ratio;
                float r = omega_c / omega_n;
                float r2 = r * r;
                float dr = 2f * zeta_eff * r;

                // Real part of FRF: Re[G(jω)]
                float denom = k_eff * ((1f - r2) * (1f - r2) + dr * dr);
                if (Mathf.Abs(denom) < 1e-12f) continue;
                float realG = (1f - r2) / denom;

                if (realG >= 0) continue; // Only unstable when Re[G] < 0

                // ap_lim (mm) = -1 / (2 * Ktc(N/mm²) * N * Re[G](mm/N))
                float realG_mmN = realG * 1000f;  // Convert m/N to mm/N
                float apLim = -1f / (2f * Ktc * fluteCount * realG_mmN);

                if (apLim > 0 && apLim < bestApLim)
                {
                    // Verify this frequency corresponds to a valid lobe
                    float toothPassFreq = rpm * fluteCount / 60f;
                    if (toothPassFreq > 0)
                    {
                        float phaseAngle = Mathf.Atan2(-dr, 1f - r2);
                        float N_lobe = (omega_c - phaseAngle) / (2f * Mathf.PI * toothPassFreq);
                        if (N_lobe > 0)
                            bestApLim = apLim;
                    }
                }
            }

            return bestApLim < float.MaxValue ? bestApLim : 100f; // 100mm = effectively unlimited
        }

        /// <summary>
        /// Compute full lobe surface grid for visualization.
        /// The surface stores stability limits at each (RPM, depth) grid point.
        /// </summary>
        public void ComputeLobeSurface(float minRPMRange, float maxRPMRange, float minDepth, float maxDepth, int resolution = 50)
        {
            lobeSurface = new float[resolution, resolution];
            lobeSurfaceRPMs = new float[resolution];
            lobeSurfaceDepths = new float[resolution];

            for (int i = 0; i < resolution; i++)
            {
                lobeSurfaceRPMs[i] = Mathf.Lerp(minRPMRange, maxRPMRange, (float)i / (resolution - 1));
                lobeSurfaceDepths[i] = Mathf.Lerp(minDepth, maxDepth, (float)i / (resolution - 1));
            }

            for (int ri = 0; ri < resolution; ri++)
            {
                float apLim = CalculateStabilityLimit(lobeSurfaceRPMs[ri]);
                for (int di = 0; di < resolution; di++)
                {
                    // Store margin: positive = stable, negative = unstable
                    lobeSurface[ri, di] = apLim - lobeSurfaceDepths[di];
                }
            }

            lobeSurfaceDirty = false;
        }

        /// <summary>
        /// Get interpolated stability limit at specific RPM from the cached surface.
        /// Returns the stability limit in mm. Falls back to direct calculation if surface not computed.
        /// </summary>
        public float GetStabilityLimitFromSurface(float rpm)
        {
            if (lobeSurface == null || lobeSurfaceRPMs == null || lobeSurfaceRPMs.Length < 2)
                return CalculateStabilityLimit(rpm);

            // Find bounding RPM indices
            int len = lobeSurfaceRPMs.Length;
            if (rpm <= lobeSurfaceRPMs[0])
                return CalculateStabilityLimit(lobeSurfaceRPMs[0]);
            if (rpm >= lobeSurfaceRPMs[len - 1])
                return CalculateStabilityLimit(lobeSurfaceRPMs[len - 1]);

            // Binary search for interval
            int lo = 0, hi = len - 1;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (lobeSurfaceRPMs[mid] <= rpm) lo = mid;
                else hi = mid;
            }

            // The stability limit at each RPM column is where margin crosses zero.
            // margin = apLim - depth, so apLim = margin + depth. At depth index 0, margin = apLim - minDepth.
            // Simpler: just interpolate the direct calculation values.
            float apLo = CalculateStabilityLimit(lobeSurfaceRPMs[lo]);
            float apHi = CalculateStabilityLimit(lobeSurfaceRPMs[hi]);
            float t = (rpm - lobeSurfaceRPMs[lo]) / (lobeSurfaceRPMs[hi] - lobeSurfaceRPMs[lo]);
            return Mathf.Lerp(apLo, apHi, t);
        }

        /// <summary>
        /// Get the full lobe surface data for visualization.
        /// Returns the surface grid, RPM axis values, and depth axis values.
        /// </summary>
        public (float[,] surface, float[] rpms, float[] depths) GetLobeSurfaceData()
        {
            return (lobeSurface, lobeSurfaceRPMs, lobeSurfaceDepths);
        }

        /// <summary>
        /// Check if operating point is in a stable pocket.
        /// A stable pocket is where the depth of cut is below the stability limit
        /// with at least the medium risk margin of headroom.
        /// </summary>
        public bool IsInStablePocket(float rpm, float depth)
        {
            float apLim = CalculateStabilityLimit(rpm);
            return depth < apLim * mediumRiskMargin;
        }

        /// <summary>
        /// Evaluate chatter risk for a given RPM and depth of cut.
        /// </summary>
        public ChatterRisk EvaluateChatterRisk(float rpm, float depthMM)
        {
            float apLim = CalculateStabilityLimit(rpm);
            LastStabilityLimit = apLim;

            float ratio = depthMM / apLim;

            if (ratio >= highRiskMargin)
            {
                LastRisk = ChatterRisk.HIGH;
            }
            else if (ratio >= mediumRiskMargin)
            {
                LastRisk = ChatterRisk.MEDIUM;
            }
            else
            {
                LastRisk = ChatterRisk.LOW;
            }

            return LastRisk;
        }

        /// <summary>
        /// Recommend a stable RPM by searching for the nearest stable lobe pocket.
        /// Returns the recommended RPM, or the current RPM if already stable.
        /// </summary>
        public float RecommendStableRPM(float currentRPM, float depthMM)
        {
            if (EvaluateChatterRisk(currentRPM, depthMM) == ChatterRisk.LOW)
                return currentRPM;

            float bestRPM = currentRPM;
            float bestMargin = 0f;

            // Search in both directions from current RPM
            for (float rpm = minRPM; rpm <= maxRPM; rpm += rpmSearchStep)
            {
                float apLim = CalculateStabilityLimit(rpm);
                float margin = apLim - depthMM;

                if (margin > bestMargin)
                {
                    bestMargin = margin;
                    bestRPM = rpm;
                }
            }

            // Prefer RPM close to current
            float closestStableRPM = bestRPM;
            float closestDist = Mathf.Abs(bestRPM - currentRPM);

            for (float rpm = minRPM; rpm <= maxRPM; rpm += rpmSearchStep)
            {
                float apLim = CalculateStabilityLimit(rpm);
                if (depthMM < apLim * mediumRiskMargin) // Must be in LOW risk zone
                {
                    float dist = Mathf.Abs(rpm - currentRPM);
                    if (dist < closestDist)
                    {
                        closestDist = dist;
                        closestStableRPM = rpm;
                    }
                }
            }

            return closestStableRPM;
        }

        /// <summary>
        /// Get the chatter risk score as a float (0=safe, 1=at limit).
        /// </summary>
        public float GetChatterRiskScore(float rpm, float depthMM)
        {
            float apLim = CalculateStabilityLimit(rpm);
            if (apLim <= 0) return 1f;
            return Mathf.Clamp01(depthMM / apLim);
        }

        /// <summary>
        /// Build a structured stability recommendation for the given operating point.
        /// Includes risk level, recommended RPM, max stable depth, stability margin,
        /// and a human-readable recommendation string.
        /// </summary>
        public StabilityRecommendationMsg GetStabilityRecommendation(string machineId, float rpm, float depthMM)
        {
            var risk = EvaluateChatterRisk(rpm, depthMM);
            float apLim = LastStabilityLimit;
            float recommendedRPM = RecommendStableRPM(rpm, depthMM);
            float margin = apLim > 0 ? Mathf.Clamp01(1f - depthMM / apLim) : 0f;

            string riskStr = risk.ToString();
            string text;

            switch (risk)
            {
                case ChatterRisk.HIGH:
                    text = $"Chatter risk HIGH. Reduce depth below {apLim:F2}mm or change RPM to {recommendedRPM:F0}. " +
                           $"Current depth {depthMM:F2}mm exceeds {highRiskMargin * 100f:F0}% of stability limit.";
                    break;
                case ChatterRisk.MEDIUM:
                    text = $"Chatter risk MEDIUM. Current depth {depthMM:F2}mm is approaching stability limit {apLim:F2}mm. " +
                           $"Consider reducing depth or shifting RPM to {recommendedRPM:F0}.";
                    break;
                default:
                    text = $"Operating within stable zone. Stability margin: {margin * 100f:F0}%.";
                    break;
            }

            return new StabilityRecommendationMsg(
                machine_id: machineId,
                current_rpm: rpm,
                recommended_rpm: recommendedRPM,
                current_depth: depthMM,
                max_stable_depth: apLim,
                risk_level: riskStr,
                stability_margin: margin,
                recommendation: text
            );
        }
    }
}
