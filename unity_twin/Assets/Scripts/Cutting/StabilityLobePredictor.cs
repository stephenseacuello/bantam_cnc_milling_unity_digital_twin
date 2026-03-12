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

        /// <summary>Last evaluated chatter risk.</summary>
        public ChatterRisk LastRisk { get; private set; } = ChatterRisk.LOW;

        /// <summary>Last evaluated stability limit (mm).</summary>
        public float LastStabilityLimit { get; private set; }

        /// <summary>
        /// Calculate the stability limit (ap_lim in mm) at a given RPM
        /// using the Altintas-Budak ZOA method.
        /// </summary>
        public float CalculateStabilityLimit(float rpm)
        {
            if (rpm <= 0) return float.MaxValue;

            float omega_n = naturalFrequencyHz * 2f * Mathf.PI;
            float bestApLim = float.MaxValue;

            // Sweep chatter frequencies near natural frequency
            for (float ratio = 0.5f; ratio < 2.0f; ratio += 0.005f)
            {
                float omega_c = omega_n * ratio;
                float r = omega_c / omega_n;
                float r2 = r * r;
                float dr = 2f * dampingRatio * r;

                // Real part of FRF: Re[G(jω)]
                float denom = stiffnessNpm * ((1f - r2) * (1f - r2) + dr * dr);
                if (Mathf.Abs(denom) < 1e-12f) continue;
                float realG = (1f - r2) / denom;

                if (realG >= 0) continue; // Only unstable when Re[G] < 0

                // ap_lim = -1 / (2 · Ktc · N · Re[G])
                // Ktc is in N/mm², Re[G] is in m/N, so convert: Ktc * 1e6 to get N/m²
                // Actually Ktc in N/mm² and ap in mm. We need consistent units.
                // ap_lim (mm) = -1 / (2 * Ktc(N/mm²) * N * Re[G](m/N))
                // = -1 / (2 * Ktc * N * Re[G])
                // But Re[G] has units m/N (compliance). We need to convert to mm/N:
                // Re[G] (mm/N) = Re[G] (m/N) * 1000
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
