using System;
using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.CNC;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Result of lookahead analysis for a single upcoming toolpath segment.
    /// </summary>
    [Serializable]
    public struct LookaheadResult
    {
        public int segmentIndex;
        public float peakForceN;
        public float powerW;
        public float tempRiseC;
        public float cumulativeWearMM;
        public float remainingLifeMin;
        public float chatterRiskScore;
        public bool collisionFlag;
    }

    /// <summary>
    /// Performs lookahead analysis on upcoming G-code segments to predict
    /// forces, power, temperature rise, cumulative wear, remaining tool life,
    /// chatter risk, and fixture collisions. Re-runs when feed/spindle overrides change.
    /// </summary>
    public class GCodeLookahead : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CNCMachineProfileSO machineProfile;
        [SerializeField] private StabilityLobePredictor stabilityPredictor;

        [Header("Settings")]
        [SerializeField] private int defaultBlockCount = 50;
        [SerializeField] private float forceWarningThresholdN = 500f;
        [SerializeField] private float powerWarningThresholdW = 5000f;
        [SerializeField] private float wearWarningThresholdMM = 0.25f;

        [Header("Fixture Collision")]
        [Tooltip("Fixture bounds for AABB collision check. Leave null to skip.")]
        [SerializeField] private BoxCollider fixtureCollider;

        // Altintas cutting coefficients (matching CuttingForceEngine)
        private const float Ktc = 796f;
        private const float Krc = 168f;
        private const float Kte = 14.5f;
        private const float Kre = 10.2f;

        // Default tool geometry
        private const float DefaultToolDiameter = 6.35f; // mm (1/4")
        private const int DefaultFlutes = 2;
        private const float DefaultAxialDepth = 1.0f; // mm
        private const float DefaultRadialDepth = 3.175f; // mm (half tool diameter)

        private List<LookaheadResult> cachedResults = new();
        private float lastFeedOverride = 1f;
        private float lastSpindleOverride = 1f;

        /// <summary>Read-only access to latest lookahead results.</summary>
        public IReadOnlyList<LookaheadResult> Results => cachedResults;

        /// <summary>Whether any result exceeds warning thresholds.</summary>
        public bool HasWarnings { get; private set; }

        /// <summary>
        /// Run lookahead from a given segment index over the next blockCount segments.
        /// </summary>
        public List<LookaheadResult> RunLookahead(
            IReadOnlyList<ToolpathSegment> segments,
            int fromIndex,
            int blockCount = -1,
            float feedOverride = 1f,
            float spindleOverride = 1f)
        {
            if (blockCount < 0) blockCount = defaultBlockCount;

            cachedResults.Clear();
            HasWarnings = false;
            lastFeedOverride = feedOverride;
            lastSpindleOverride = spindleOverride;

            if (segments == null || fromIndex >= segments.Count) return cachedResults;

            int endIndex = Mathf.Min(fromIndex + blockCount, segments.Count);
            float cumulativeWear = 0f;
            float cumulativeTime = 0f;

            for (int i = fromIndex; i < endIndex; i++)
            {
                var seg = segments[i];
                var result = AnalyzeSegment(seg, i, ref cumulativeWear, ref cumulativeTime,
                    feedOverride, spindleOverride);
                cachedResults.Add(result);

                if (result.peakForceN > forceWarningThresholdN ||
                    result.powerW > powerWarningThresholdW ||
                    result.cumulativeWearMM > wearWarningThresholdMM ||
                    result.collisionFlag)
                {
                    HasWarnings = true;
                }
            }

            return cachedResults;
        }

        private LookaheadResult AnalyzeSegment(
            ToolpathSegment seg, int index,
            ref float cumulativeWear, ref float cumulativeTime,
            float feedOverride, float spindleOverride)
        {
            var result = new LookaheadResult { segmentIndex = index };

            // Skip rapids — no cutting forces
            if (seg.type == SegmentType.Rapid)
            {
                result.remainingLifeMin = EstimateRemainingLife(cumulativeWear);
                return result;
            }

            float feedRate = seg.feedRate * feedOverride;
            float spindleRPM = seg.spindleRPM * spindleOverride;

            if (spindleRPM < 1f || feedRate < 0.01f)
            {
                result.remainingLifeMin = EstimateRemainingLife(cumulativeWear);
                return result;
            }

            // Feed per tooth
            float fz = feedRate / (spindleRPM * DefaultFlutes);

            // Chip thickness (simplified Altintas model)
            float chipThickness = fz; // Approximation for straight cuts

            // Cutting forces using Altintas mechanistic model
            float Ft = Ktc * DefaultAxialDepth * chipThickness + Kte * DefaultAxialDepth;
            float Fr = Krc * DefaultAxialDepth * chipThickness + Kre * DefaultAxialDepth;
            float resultantForce = Mathf.Sqrt(Ft * Ft + Fr * Fr);

            result.peakForceN = resultantForce;

            // Cutting power: P = Ft * Vc / 60000 (kW → W)
            float Vc = Mathf.PI * DefaultToolDiameter * spindleRPM / 1000f; // m/min
            result.powerW = Ft * Vc / 60f; // W

            // Temperature rise estimate (simplified): proportional to specific cutting energy
            float specificEnergy = Ft / (DefaultAxialDepth * chipThickness);
            result.tempRiseC = specificEnergy * 0.002f; // Empirical scaling

            // Wear increment (Taylor model simplified)
            float segmentLength = Vector3.Distance(seg.startPos, seg.endPos);
            float cuttingTime = (segmentLength / feedRate) * 60f; // seconds
            cumulativeTime += cuttingTime;

            // VB wear rate proportional to cutting speed
            float wearRate = 0.001f * Mathf.Pow(Vc / 100f, 1.5f); // mm/min
            cumulativeWear += wearRate * (cuttingTime / 60f);
            result.cumulativeWearMM = cumulativeWear;

            // Remaining tool life
            result.remainingLifeMin = EstimateRemainingLife(cumulativeWear);

            // Chatter risk from stability lobe predictor
            if (stabilityPredictor != null)
            {
                result.chatterRiskScore = stabilityPredictor.GetChatterRiskScore();
                // Re-evaluate at this RPM/depth
                var risk = stabilityPredictor.EvaluateChatterRisk(spindleRPM, DefaultAxialDepth);
                result.chatterRiskScore = risk switch
                {
                    ChatterRisk.LOW => 0.1f,
                    ChatterRisk.MEDIUM => 0.5f,
                    ChatterRisk.HIGH => 0.9f,
                    _ => 0f
                };
            }

            // AABB collision check
            if (fixtureCollider != null)
            {
                Bounds fixtureBounds = fixtureCollider.bounds;
                if (fixtureBounds.Contains(seg.endPos) || fixtureBounds.Contains(seg.startPos))
                {
                    result.collisionFlag = true;
                }
            }

            return result;
        }

        private float EstimateRemainingLife(float currentWear)
        {
            const float VBmax = 0.30f; // mm
            float remaining = VBmax - currentWear;
            if (remaining <= 0f) return 0f;
            // Rough estimate: assume constant wear rate from recent history
            return remaining * 60f; // minutes (placeholder linear estimate)
        }

        /// <summary>
        /// Check if overrides have changed and lookahead needs re-running.
        /// </summary>
        public bool NeedsUpdate(float feedOverride, float spindleOverride)
        {
            return Mathf.Abs(feedOverride - lastFeedOverride) > 0.01f ||
                   Mathf.Abs(spindleOverride - lastSpindleOverride) > 0.01f;
        }
    }
}
