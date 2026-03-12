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
        public FixtureProfile.CollisionSeverity collisionSeverity;
    }

    /// <summary>
    /// Describes a contiguous range of segments that share the same collision risk severity.
    /// Used by UI/visualization to highlight dangerous path segments.
    /// </summary>
    public struct CollisionRiskSegment
    {
        public int startIndex;
        public int endIndex;
        public FixtureProfile.CollisionSeverity severity;
    }

    /// <summary>
    /// Performs lookahead analysis on upcoming G-code segments to predict
    /// forces, power, temperature rise, cumulative wear, remaining tool life,
    /// chatter risk, and fixture collisions. Re-runs when feed/spindle overrides change.
    ///
    /// When a FixtureProfile is assigned, uses oriented clamp-zone collision checks
    /// (including tool shank) instead of the legacy AABB BoxCollider check.
    /// For rapid moves (G0), intermediate points along the path are also tested.
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
        [Tooltip("Fixture profile for oriented clamp-zone collision checks. Overrides legacy BoxCollider when assigned.")]
        [SerializeField] private FixtureProfile fixtureProfile;

        [Tooltip("Legacy fixture bounds for AABB collision check. Used only if fixtureProfile is null.")]
        [SerializeField] private BoxCollider fixtureCollider;

        [Header("Tool Geometry")]
        [SerializeField] private float toolDiameter = 6.35f;  // mm (1/4")
        [SerializeField] private float toolLength = 50f;       // mm

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

        /// <summary>Step size in mm for sampling intermediate points on rapid moves.</summary>
        private const float RapidCollisionSampleStep = 5f;

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

            // --- Fixture collision check (applies to all move types including rapids) ---
            CheckSegmentCollision(seg, ref result);

            // Skip force/wear analysis for rapids — no cutting forces
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

            return result;
        }

        /// <summary>
        /// Check a segment for fixture collision using the FixtureProfile (preferred)
        /// or legacy BoxCollider fallback.
        /// For rapid moves, intermediate points along the path are sampled.
        /// </summary>
        private void CheckSegmentCollision(ToolpathSegment seg, ref LookaheadResult result)
        {
            if (fixtureProfile != null)
            {
                CheckSegmentWithProfile(seg, ref result);
            }
            else if (fixtureCollider != null)
            {
                // Legacy AABB check
                Bounds fixtureBounds = fixtureCollider.bounds;
                if (fixtureBounds.Contains(seg.endPos) || fixtureBounds.Contains(seg.startPos))
                {
                    result.collisionFlag = true;
                    result.collisionSeverity = FixtureProfile.CollisionSeverity.FixtureCollision;
                }
            }
        }

        /// <summary>
        /// Perform collision checks using the FixtureProfile.
        /// For rapid moves (G0), sample intermediate points along the path to catch
        /// collisions during long traversals.
        /// </summary>
        private void CheckSegmentWithProfile(ToolpathSegment seg, ref LookaheadResult result)
        {
            // Always check start and end
            var startCheck = fixtureProfile.CheckCollision(seg.startPos, toolDiameter, toolLength);
            if (startCheck.isCollision)
            {
                result.collisionFlag = true;
                result.collisionSeverity = startCheck.severity;
                return;
            }

            var endCheck = fixtureProfile.CheckCollision(seg.endPos, toolDiameter, toolLength);
            if (endCheck.isCollision)
            {
                result.collisionFlag = true;
                result.collisionSeverity = endCheck.severity;
                return;
            }

            // For rapid moves, also check intermediate points along the path
            if (seg.type == SegmentType.Rapid)
            {
                float pathLength = Vector3.Distance(seg.startPos, seg.endPos);
                if (pathLength > RapidCollisionSampleStep)
                {
                    int steps = Mathf.CeilToInt(pathLength / RapidCollisionSampleStep);
                    for (int s = 1; s < steps; s++)
                    {
                        float t = (float)s / steps;
                        Vector3 samplePoint = Vector3.Lerp(seg.startPos, seg.endPos, t);
                        var midCheck = fixtureProfile.CheckCollision(samplePoint, toolDiameter, toolLength);
                        if (midCheck.isCollision)
                        {
                            result.collisionFlag = true;
                            result.collisionSeverity = midCheck.severity;
                            return;
                        }
                    }
                }
            }

            // Check for near-misses on endpoints (informational, does not set collisionFlag)
            var nearMiss = fixtureProfile.CheckNearMiss(seg.endPos, toolDiameter);
            if (nearMiss.severity == FixtureProfile.CollisionSeverity.NearMiss
                && result.collisionSeverity == FixtureProfile.CollisionSeverity.None)
            {
                result.collisionSeverity = FixtureProfile.CollisionSeverity.NearMiss;
            }
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

        /// <summary>
        /// Returns contiguous ranges of segments grouped by collision risk severity.
        /// Used by UI/visualization to highlight dangerous path segments with color coding.
        /// Only segments with severity > None are included.
        /// </summary>
        public List<CollisionRiskSegment> GetCollisionRiskSegments()
        {
            var riskSegments = new List<CollisionRiskSegment>();
            if (cachedResults.Count == 0) return riskSegments;

            int i = 0;
            while (i < cachedResults.Count)
            {
                var severity = cachedResults[i].collisionSeverity;
                if (severity == FixtureProfile.CollisionSeverity.None)
                {
                    i++;
                    continue;
                }

                // Start a new risk segment
                int startIdx = cachedResults[i].segmentIndex;
                int endIdx = startIdx;

                // Extend while consecutive results share the same severity
                int j = i + 1;
                while (j < cachedResults.Count && cachedResults[j].collisionSeverity == severity)
                {
                    endIdx = cachedResults[j].segmentIndex;
                    j++;
                }

                riskSegments.Add(new CollisionRiskSegment
                {
                    startIndex = startIdx,
                    endIndex = endIdx,
                    severity = severity
                });

                i = j;
            }

            return riskSegments;
        }
    }
}
