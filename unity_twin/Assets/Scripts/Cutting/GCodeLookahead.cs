using System;
using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.CNC;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Describes fixture geometry using an axis-aligned bounding box,
    /// a list of clamp zones, and a safe retract height for collision avoidance.
    /// Used by CheckFixtureCollisions to test toolpath blocks against fixture bounds.
    /// </summary>
    [Serializable]
    public class FixtureCollisionProfile
    {
        /// <summary>Minimum corner of the fixture AABB (mm).</summary>
        public Vector3 boundsMin;
        /// <summary>Maximum corner of the fixture AABB (mm).</summary>
        public Vector3 boundsMax;
        /// <summary>Individual clamp zone AABBs within the fixture.</summary>
        public List<Bounds> clampZones = new();
        /// <summary>Safe Z height (mm) to retract to when avoiding collisions.</summary>
        public float safeRetractHeight = 50f;

        /// <summary>Returns the fixture AABB as a Unity Bounds.</summary>
        public Bounds GetBounds()
        {
            var center = (boundsMin + boundsMax) * 0.5f;
            var size = boundsMax - boundsMin;
            return new Bounds(center, size);
        }

        /// <summary>
        /// Tests whether a point lies inside the fixture AABB (with optional margin).
        /// </summary>
        public bool ContainsPoint(Vector3 point, float margin = 0f)
        {
            return point.x >= boundsMin.x - margin && point.x <= boundsMax.x + margin
                && point.y >= boundsMin.y - margin && point.y <= boundsMax.y + margin
                && point.z >= boundsMin.z - margin && point.z <= boundsMax.z + margin;
        }

        /// <summary>
        /// Computes minimum distance from a point to the surface of the fixture AABB.
        /// Returns negative values when the point is inside the AABB.
        /// </summary>
        public float DistanceToSurface(Vector3 point)
        {
            // Compute signed distance to each face, take the minimum absolute
            float dx = Mathf.Max(boundsMin.x - point.x, point.x - boundsMax.x);
            float dy = Mathf.Max(boundsMin.y - point.y, point.y - boundsMax.y);
            float dz = Mathf.Max(boundsMin.z - point.z, point.z - boundsMax.z);

            // If all components are negative, point is inside — return the max (least negative)
            if (dx < 0 && dy < 0 && dz < 0)
                return Mathf.Max(dx, Mathf.Max(dy, dz));

            // Outside: Euclidean distance from clamped components
            float cx = Mathf.Max(dx, 0f);
            float cy = Mathf.Max(dy, 0f);
            float cz = Mathf.Max(dz, 0f);
            return Mathf.Sqrt(cx * cx + cy * cy + cz * cz);
        }
    }

    /// <summary>
    /// Result of a batch fixture-collision check across multiple lookahead blocks.
    /// Reports the first detected collision with its block index, contact point,
    /// minimum clearance distance, and a human-readable recommendation.
    /// </summary>
    [Serializable]
    public class CollisionCheckResult
    {
        /// <summary>True if any block intersects the fixture AABB.</summary>
        public bool hasCollision;
        /// <summary>Index into the LookaheadResult array of the first colliding block (-1 if none).</summary>
        public int collisionBlockIndex = -1;
        /// <summary>World-space point of the first detected collision.</summary>
        public Vector3 collisionPoint;
        /// <summary>Minimum clearance distance (mm) across all checked blocks. Negative means penetration.</summary>
        public float clearanceDistance = float.MaxValue;
        /// <summary>Human-readable recommendation for resolving the collision.</summary>
        public string recommendation = "";
    }

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

    // ────────────────────────────────────────────────────────────────────
    //  Tool-path smoothing & corner optimization types
    // ────────────────────────────────────────────────────────────────────

    /// <summary>Corner sharpness classification.</summary>
    public enum CornerType
    {
        SHARP,      // < 30°
        MODERATE,   // 30°–90°
        GENTLE,     // 90°–150°
        STRAIGHT    // ≥ 150°
    }

    /// <summary>
    /// Analysis of a single corner (junction between two consecutive toolpath segments).
    /// </summary>
    [Serializable]
    public class CornerAnalysis
    {
        /// <summary>Index of the block where the corner occurs (end of incoming segment).</summary>
        public int blockIndex;
        /// <summary>Angle in radians between incoming and outgoing direction vectors.</summary>
        public float angleRadians;
        /// <summary>Angle in degrees between incoming and outgoing direction vectors.</summary>
        public float angleDegrees;
        /// <summary>Classification of the corner sharpness.</summary>
        public CornerType cornerType;
        /// <summary>Programmed feed rate of the incoming segment (mm/min).</summary>
        public float incomingFeed;
        /// <summary>Programmed feed rate of the outgoing segment (mm/min).</summary>
        public float outgoingFeed;
        /// <summary>Recommended feed rate through the corner (mm/min).</summary>
        public float recommendedCornerFeed;
        /// <summary>Distance (mm) required to decelerate from cruise to the corner feed.</summary>
        public float decelerationDistance;
        /// <summary>Distance (mm) required to accelerate from the corner feed back to cruise.</summary>
        public float accelerationDistance;
    }

    /// <summary>
    /// Aggregate result of tool-path smoothing and corner optimisation.
    /// </summary>
    [Serializable]
    public class PathSmoothingResult
    {
        public int originalSegmentCount;
        public int smoothedSegmentCount;
        public List<CornerAnalysis> corners = new();
        public float totalPathLength;
        /// <summary>Estimated cycle time (seconds) using original programmed feeds.</summary>
        public float estimatedCycleTime;
        /// <summary>Optimised cycle time (seconds) after feed-profile smoothing.</summary>
        public float optimizedCycleTime;
        /// <summary>Percentage of time saved by the optimisation.</summary>
        public float timeSavingsPct;
        /// <summary>Maximum jerk (mm/s³) encountered during the optimised profile.</summary>
        public float maxJerk;
    }

    /// <summary>
    /// Analyses corners between toolpath segments and produces an optimised
    /// trapezoidal velocity profile that respects machine acceleration, jerk,
    /// and corner-tolerance constraints.
    /// </summary>
    public class ToolPathSmoother
    {
        /// <summary>Maximum linear acceleration (mm/s²).</summary>
        public float maxAcceleration = 5000f;
        /// <summary>Maximum jerk (mm/s³).</summary>
        public float maxJerk = 50000f;
        /// <summary>Corner tolerance / blending radius (mm).</summary>
        public float cornerTolerance = 0.01f;

        // ── Corner analysis ─────────────────────────────────────────────

        /// <summary>
        /// Compute the angle and classification for every junction between
        /// consecutive segments.
        /// </summary>
        public List<CornerAnalysis> AnalyzeCorners(List<ToolpathSegment> segments)
        {
            var corners = new List<CornerAnalysis>();
            if (segments == null || segments.Count < 2) return corners;

            for (int i = 0; i < segments.Count - 1; i++)
            {
                var incoming = segments[i];
                var outgoing = segments[i + 1];

                Vector3 dirIn = (incoming.endPos - incoming.startPos).normalized;
                Vector3 dirOut = (outgoing.endPos - outgoing.startPos).normalized;

                // Angle between the two direction vectors
                float dot = Mathf.Clamp(Vector3.Dot(dirIn, dirOut), -1f, 1f);
                float angleRad = Mathf.Acos(dot); // 0 = same direction, π = reversal
                float angleDeg = angleRad * Mathf.Rad2Deg;

                var ca = new CornerAnalysis
                {
                    blockIndex = i,
                    angleRadians = angleRad,
                    angleDegrees = angleDeg,
                    cornerType = ClassifyCorner(angleDeg),
                    incomingFeed = incoming.feedRate,
                    outgoingFeed = outgoing.feedRate
                };

                // Blending radius derived from corner tolerance
                float halfAngle = angleRad * 0.5f;
                float blendRadius = (halfAngle > 1e-6f)
                    ? cornerTolerance / (1f - Mathf.Cos(halfAngle))
                    : float.MaxValue;

                ca.recommendedCornerFeed = ComputeCornerFeedLimit(
                    angleRad, blendRadius, Mathf.Min(incoming.feedRate, outgoing.feedRate));

                // Trapezoidal decel / accel distances: d = (v² - v_c²) / (2a)
                float vCruiseIn = incoming.feedRate / 60f;   // mm/s
                float vCruiseOut = outgoing.feedRate / 60f;
                float vCorner = ca.recommendedCornerFeed / 60f;

                ca.decelerationDistance = Mathf.Max(0f,
                    (vCruiseIn * vCruiseIn - vCorner * vCorner) / (2f * maxAcceleration));
                ca.accelerationDistance = Mathf.Max(0f,
                    (vCruiseOut * vCruiseOut - vCorner * vCorner) / (2f * maxAcceleration));

                corners.Add(ca);
            }

            return corners;
        }

        /// <summary>Classify a corner by its deflection angle (degrees).</summary>
        private static CornerType ClassifyCorner(float angleDeg)
        {
            if (angleDeg < 30f)  return CornerType.STRAIGHT;   // nearly collinear
            if (angleDeg < 90f)  return CornerType.GENTLE;
            if (angleDeg < 150f) return CornerType.MODERATE;
            return CornerType.SHARP;                            // near reversal
        }

        // ── Corner feed limit ───────────────────────────────────────────

        /// <summary>
        /// Compute the maximum allowable feed through a corner given the
        /// deflection angle, blending radius, and programmed feed.
        /// Uses centripetal acceleration constraint: v = sqrt(a_max * r).
        /// </summary>
        public float ComputeCornerFeedLimit(float angleRad, float radius, float programmedFeed)
        {
            if (angleRad < 1e-6f) return programmedFeed; // straight – no limit

            // Centripetal limit (mm/s)
            float vMax = Mathf.Sqrt(maxAcceleration * Mathf.Max(radius, 1e-6f));
            // Convert back to mm/min
            float feedLimit = vMax * 60f;
            return Mathf.Min(feedLimit, programmedFeed);
        }

        // ── Feed-profile optimisation (forward + backward pass) ─────────

        /// <summary>
        /// Produce a per-segment optimal feed array using a bidirectional
        /// trapezoidal velocity planner.
        /// </summary>
        public float[] OptimizeFeedProfile(
            List<ToolpathSegment> segments,
            List<CornerAnalysis> corners)
        {
            if (segments == null || segments.Count == 0)
                return Array.Empty<float>();

            int n = segments.Count;
            float[] feeds = new float[n];

            // Initialise to programmed feeds
            for (int i = 0; i < n; i++)
                feeds[i] = segments[i].feedRate;

            if (corners == null || corners.Count == 0)
                return feeds;

            // ── Forward pass: limit acceleration away from each corner ──
            for (int c = 0; c < corners.Count; c++)
            {
                float vCorner = corners[c].recommendedCornerFeed / 60f; // mm/s
                int startSeg = corners[c].blockIndex + 1; // segment after the corner

                float vPrev = vCorner;
                for (int i = startSeg; i < n; i++)
                {
                    float segLen = Vector3.Distance(segments[i].startPos, segments[i].endPos);
                    // v² = v_prev² + 2*a*d
                    float vReachable = Mathf.Sqrt(vPrev * vPrev + 2f * maxAcceleration * segLen);
                    float vReachableFeed = vReachable * 60f;

                    if (vReachableFeed < feeds[i])
                        feeds[i] = vReachableFeed;
                    else
                        break; // already at or below cruise – stop propagating

                    vPrev = feeds[i] / 60f;
                }
            }

            // ── Backward pass: limit deceleration into each corner ──────
            for (int c = 0; c < corners.Count; c++)
            {
                float vCorner = corners[c].recommendedCornerFeed / 60f; // mm/s
                int endSeg = corners[c].blockIndex; // segment arriving at the corner

                float vPrev = vCorner;
                for (int i = endSeg; i >= 0; i--)
                {
                    float segLen = Vector3.Distance(segments[i].startPos, segments[i].endPos);
                    float vReachable = Mathf.Sqrt(vPrev * vPrev + 2f * maxAcceleration * segLen);
                    float vReachableFeed = vReachable * 60f;

                    if (vReachableFeed < feeds[i])
                        feeds[i] = vReachableFeed;
                    else
                        break;

                    vPrev = feeds[i] / 60f;
                }
            }

            return feeds;
        }

        // ── Cycle-time estimation ───────────────────────────────────────

        /// <summary>
        /// Estimate total cycle time (seconds) given segments and their
        /// effective feed rates (mm/min).
        /// </summary>
        public float EstimateCycleTime(List<ToolpathSegment> segments, float[] feeds)
        {
            if (segments == null || feeds == null) return 0f;

            float totalTime = 0f;
            int count = Mathf.Min(segments.Count, feeds.Length);

            for (int i = 0; i < count; i++)
            {
                float len = Vector3.Distance(segments[i].startPos, segments[i].endPos);
                float feedMmPerSec = feeds[i] / 60f;
                if (feedMmPerSec > 1e-6f)
                    totalTime += len / feedMmPerSec;
            }

            return totalTime;
        }

        // ── High-level entry point ──────────────────────────────────────

        /// <summary>
        /// Run the full smoothing pipeline: analyse corners, optimise the
        /// feed profile, and return aggregate metrics.
        /// </summary>
        public PathSmoothingResult SmoothPath(List<ToolpathSegment> segments)
        {
            var result = new PathSmoothingResult();
            if (segments == null || segments.Count == 0)
                return result;

            result.originalSegmentCount = segments.Count;
            result.smoothedSegmentCount = segments.Count; // no segment merging in this pass

            // Total path length
            float totalLen = 0f;
            for (int i = 0; i < segments.Count; i++)
                totalLen += Vector3.Distance(segments[i].startPos, segments[i].endPos);
            result.totalPathLength = totalLen;

            // Corner analysis
            var corners = AnalyzeCorners(segments);
            result.corners = corners;

            // Original cycle time (programmed feeds)
            float[] originalFeeds = new float[segments.Count];
            for (int i = 0; i < segments.Count; i++)
                originalFeeds[i] = segments[i].feedRate;
            result.estimatedCycleTime = EstimateCycleTime(segments, originalFeeds);

            // Optimised cycle time
            float[] optFeeds = OptimizeFeedProfile(segments, corners);
            result.optimizedCycleTime = EstimateCycleTime(segments, optFeeds);

            // Time savings
            if (result.estimatedCycleTime > 1e-6f)
            {
                result.timeSavingsPct =
                    (1f - result.optimizedCycleTime / result.estimatedCycleTime) * 100f;
            }

            result.maxJerk = maxJerk;

            return result;
        }
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

        // ────────────────────────────────────────────────────────────────────
        //  Batch fixture-collision detection (FixtureCollisionProfile)
        // ────────────────────────────────────────────────────────────────────

        /// <summary>Safety margin (mm) added to AABB checks for rapid (G00) moves.</summary>
        private const float RapidSafetyMargin = 2f;

        /// <summary>Number of sample points when checking arc segments (G02/G03).</summary>
        private const int ArcCollisionSamples = 8;

        /// <summary>
        /// Check a batch of lookahead results against a <see cref="FixtureCollisionProfile"/>.
        /// <para>
        /// For each block the method inspects the toolpath segment positions:
        /// <list type="bullet">
        ///   <item>Rapid moves (G00) use a 2 mm safety margin around the fixture AABB.</item>
        ///   <item>Arc moves (G02/G03) are sampled at 8 evenly-spaced points along the arc.</item>
        ///   <item>Linear cutting moves check start and end positions.</item>
        /// </list>
        /// Returns on the first collision found (earliest block index).
        /// </para>
        /// </summary>
        /// <param name="results">Lookahead blocks to check. May be null or empty.</param>
        /// <param name="fixture">Fixture geometry. May be null (returns no-collision).</param>
        /// <param name="segments">Original toolpath segments aligned with <paramref name="results"/>.</param>
        public CollisionCheckResult CheckFixtureCollisions(
            LookaheadResult[] results,
            FixtureCollisionProfile fixture,
            IReadOnlyList<ToolpathSegment> segments = null)
        {
            var output = new CollisionCheckResult();

            if (results == null || results.Length == 0 || fixture == null)
                return output;

            for (int i = 0; i < results.Length; i++)
            {
                var r = results[i];
                ToolpathSegment seg = default;
                bool hasSeg = segments != null && r.segmentIndex >= 0 && r.segmentIndex < segments.Count;
                if (hasSeg) seg = segments[r.segmentIndex];

                // Determine move type from the segment (default to Linear if unavailable)
                SegmentType moveType = hasSeg ? seg.type : SegmentType.Linear;
                float margin = moveType == SegmentType.Rapid ? RapidSafetyMargin : 0f;

                // Gather candidate points to test
                var points = new List<Vector3>();

                if (hasSeg)
                {
                    points.Add(seg.startPos);
                    points.Add(seg.endPos);

                    // Arc moves: sample intermediate points along the arc
                    if (moveType == SegmentType.CWArc || moveType == SegmentType.CCWArc)
                    {
                        AddArcSamplePoints(seg, points);
                    }
                }
                else
                {
                    // Fallback: no segment data, skip geometric check
                    continue;
                }

                // Evaluate each candidate point
                foreach (var pt in points)
                {
                    float dist = fixture.DistanceToSurface(pt);
                    float effectiveClearance = dist - margin;

                    // Track global minimum clearance
                    if (effectiveClearance < output.clearanceDistance)
                        output.clearanceDistance = effectiveClearance;

                    // Collision when the point (with margin) is inside the fixture
                    if (fixture.ContainsPoint(pt, margin))
                    {
                        output.hasCollision = true;
                        output.collisionBlockIndex = i;
                        output.collisionPoint = pt;

                        string moveLabel = moveType == SegmentType.Rapid ? "rapid move (G00)"
                            : (moveType == SegmentType.CWArc || moveType == SegmentType.CCWArc) ? "arc move"
                            : "cutting move";
                        output.recommendation =
                            $"Collision detected at block {i} during {moveLabel}. " +
                            $"Retract to safe height Z={fixture.safeRetractHeight:F1} mm before traversing fixture zone.";

                        return output; // Report first collision only
                    }
                }
            }

            return output;
        }

        /// <summary>
        /// Samples <see cref="ArcCollisionSamples"/> evenly-spaced points along an arc segment
        /// and appends them to <paramref name="points"/>.
        /// The arc is defined by <c>startPos</c>, <c>endPos</c>, and <c>arcCenter</c> on the XY plane.
        /// </summary>
        private void AddArcSamplePoints(ToolpathSegment seg, List<Vector3> points)
        {
            Vector3 center = seg.arcCenter;
            Vector3 startOffset = seg.startPos - center;
            Vector3 endOffset = seg.endPos - center;

            float startAngle = Mathf.Atan2(startOffset.z, startOffset.x);
            float endAngle = Mathf.Atan2(endOffset.z, endOffset.x);
            float radius = startOffset.magnitude;

            // Determine sweep direction
            float sweep;
            if (seg.type == SegmentType.CWArc)
            {
                sweep = startAngle - endAngle;
                if (sweep <= 0f) sweep += Mathf.PI * 2f;
                sweep = -sweep; // CW is negative sweep
            }
            else
            {
                sweep = endAngle - startAngle;
                if (sweep <= 0f) sweep += Mathf.PI * 2f;
            }

            // Linearly interpolate Z between start and end
            float zStart = seg.startPos.y; // Unity Y is vertical
            float zEnd = seg.endPos.y;

            for (int s = 1; s <= ArcCollisionSamples; s++)
            {
                float t = (float)s / (ArcCollisionSamples + 1);
                float angle = startAngle + sweep * t;
                float y = Mathf.Lerp(zStart, zEnd, t);
                var samplePt = new Vector3(
                    center.x + radius * Mathf.Cos(angle),
                    y,
                    center.z + radius * Mathf.Sin(angle)
                );
                points.Add(samplePt);
            }
        }

        /// <summary>
        /// Given a collision at <paramref name="collisionBlock"/>, generate a safe
        /// retract-traverse-plunge alternative path that avoids the fixture.
        /// Returns a list of waypoints: retract straight up to safe Z, traverse
        /// horizontally to the target XZ, then plunge back down.
        /// </summary>
        /// <param name="collisionBlock">Index into the lookahead results where the collision occurs.</param>
        /// <param name="fixture">Fixture profile supplying the safe retract height.</param>
        /// <param name="segments">Original toolpath segments (optional; used to derive start/end positions).</param>
        public List<Vector3> GenerateCollisionAvoidancePath(
            int collisionBlock,
            FixtureCollisionProfile fixture,
            IReadOnlyList<ToolpathSegment> segments = null)
        {
            var waypoints = new List<Vector3>();

            if (fixture == null || collisionBlock < 0)
                return waypoints;

            // Determine start and end positions from segment data
            Vector3 startPos = Vector3.zero;
            Vector3 endPos = Vector3.zero;

            if (segments != null && cachedResults.Count > collisionBlock)
            {
                int segIdx = cachedResults[collisionBlock].segmentIndex;
                if (segIdx >= 0 && segIdx < segments.Count)
                {
                    startPos = segments[segIdx].startPos;
                    endPos = segments[segIdx].endPos;
                }
            }

            float safeZ = fixture.safeRetractHeight;

            // 1. Retract: move straight up from current position to safe height
            var retractPoint = new Vector3(startPos.x, safeZ, startPos.z);
            waypoints.Add(retractPoint);

            // 2. Traverse: move horizontally at safe height to above the target XZ
            var traversePoint = new Vector3(endPos.x, safeZ, endPos.z);
            waypoints.Add(traversePoint);

            // 3. Plunge: descend to the target position
            waypoints.Add(endPos);

            return waypoints;
        }
    }

    // -----------------------------------------------------------------------
    // Dwell Time Analyzer
    // -----------------------------------------------------------------------

    /// <summary>Type of dwell detected in the program.</summary>
    public enum DwellType
    {
        Explicit,   // G4 command
        Implicit    // Consecutive blocks at same position
    }

    /// <summary>A single dwell event in the G-code program.</summary>
    [Serializable]
    public class DwellEvent
    {
        public int blockIndex;
        public DwellType type;
        public float durationSeconds;
        public string rawLine;
        public bool isExcessive;
    }

    /// <summary>An optimization suggestion for dwell usage.</summary>
    [Serializable]
    public class DwellOptimizationSuggestion
    {
        public string description;
        public int blockIndex;
        public float timeSavingSeconds;
    }

    /// <summary>Report summarising all dwells in a program.</summary>
    [Serializable]
    public class DwellReport
    {
        public float totalDwellTimeSeconds;
        public int dwellCount;
        public int excessiveDwellCount;
        public int implicitDwellCount;
        public float dwellPercentage;
        public float totalProgramTimeSeconds;
        public List<DwellEvent> events = new();
        public List<DwellOptimizationSuggestion> suggestions = new();
    }

    /// <summary>
    /// Analyses G-code programs for explicit (G4) and implicit dwells,
    /// flags excessive dwell times, and suggests optimizations.
    /// </summary>
    public class DwellAnalyzer
    {
        private readonly float _excessiveThresholdSec;

        /// <param name="excessiveThresholdSec">
        /// Dwell duration above which a dwell is flagged as excessive (default 2.0 s).
        /// </param>
        public DwellAnalyzer(float excessiveThresholdSec = 2.0f)
        {
            _excessiveThresholdSec = excessiveThresholdSec;
        }

        /// <summary>
        /// Analyse a list of G-code lines and an estimated total program time.
        /// </summary>
        public DwellReport Analyze(List<string> lines, float totalProgramTimeSec = 0f)
        {
            var report = new DwellReport { totalProgramTimeSeconds = totalProgramTimeSec };

            Vector3 lastPos = Vector3.zero;
            bool hasLastPos = false;
            int consecutiveSame = 0;

            for (int i = 0; i < lines.Count; i++)
            {
                string line = lines[i].Trim().ToUpperInvariant();
                if (string.IsNullOrEmpty(line) || line.StartsWith("%") || line.StartsWith("("))
                    continue;

                // Check for explicit G4 dwell
                float dwellSec = ParseG4Dwell(line);
                if (dwellSec > 0f)
                {
                    var evt = new DwellEvent
                    {
                        blockIndex = i,
                        type = DwellType.Explicit,
                        durationSeconds = dwellSec,
                        rawLine = lines[i],
                        isExcessive = dwellSec > _excessiveThresholdSec,
                    };
                    report.events.Add(evt);
                    report.totalDwellTimeSeconds += dwellSec;
                    report.dwellCount++;
                    if (evt.isExcessive) report.excessiveDwellCount++;
                    continue;
                }

                // Track position for implicit dwell detection
                Vector3 pos = ParsePosition(line, lastPos);
                if (hasLastPos && pos == lastPos)
                {
                    consecutiveSame++;
                    if (consecutiveSame >= 2) // 3+ blocks at same position
                    {
                        // Only record once per run
                        if (consecutiveSame == 2)
                        {
                            var evt = new DwellEvent
                            {
                                blockIndex = i - 2,
                                type = DwellType.Implicit,
                                durationSeconds = 0f, // unknown without feed simulation
                                rawLine = $"(implicit dwell: {consecutiveSame + 1} blocks at same position)",
                                isExcessive = false,
                            };
                            report.events.Add(evt);
                            report.implicitDwellCount++;
                            report.dwellCount++;
                        }
                    }
                }
                else
                {
                    consecutiveSame = 0;
                }

                lastPos = pos;
                hasLastPos = true;
            }

            // Calculate dwell percentage
            if (totalProgramTimeSec > 0f)
                report.dwellPercentage = (report.totalDwellTimeSeconds / totalProgramTimeSec) * 100f;

            // Generate optimization suggestions
            GenerateSuggestions(report, lines);

            return report;
        }

        /// <summary>Parse a G4 dwell command. Returns seconds, or 0 if not a G4.</summary>
        public static float ParseG4Dwell(string line)
        {
            if (!line.Contains("G4") && !line.Contains("G04"))
                return 0f;

            // P parameter = milliseconds
            int pIdx = line.IndexOf('P');
            if (pIdx >= 0)
            {
                string numStr = ExtractNumber(line, pIdx + 1);
                if (float.TryParse(numStr, out float pVal))
                    return pVal / 1000f; // ms -> sec
            }

            // X parameter = seconds (Fanuc-style)
            int xIdx = line.IndexOf('X');
            if (xIdx >= 0)
            {
                string numStr = ExtractNumber(line, xIdx + 1);
                if (float.TryParse(numStr, out float xVal))
                    return xVal; // already seconds
            }

            return 0f;
        }

        private static string ExtractNumber(string line, int startIdx)
        {
            int end = startIdx;
            bool hasDot = false;
            if (end < line.Length && line[end] == '-') end++;
            while (end < line.Length && (char.IsDigit(line[end]) || (line[end] == '.' && !hasDot)))
            {
                if (line[end] == '.') hasDot = true;
                end++;
            }
            return line.Substring(startIdx, end - startIdx);
        }

        private static Vector3 ParsePosition(string line, Vector3 current)
        {
            float x = current.x, y = current.y, z = current.z;
            int xi = line.IndexOf('X');
            int yi = line.IndexOf('Y');
            int zi = line.IndexOf('Z');
            if (xi >= 0) { string n = ExtractNumber(line, xi + 1); float.TryParse(n, out x); }
            if (yi >= 0) { string n = ExtractNumber(line, yi + 1); float.TryParse(n, out y); }
            if (zi >= 0) { string n = ExtractNumber(line, zi + 1); float.TryParse(n, out z); }
            return new Vector3(x, y, z);
        }

        private void GenerateSuggestions(DwellReport report, List<string> lines)
        {
            // Suggest removing dwells before rapids (G0)
            for (int i = 0; i < report.events.Count; i++)
            {
                var evt = report.events[i];
                if (evt.type != DwellType.Explicit) continue;

                // Check if next non-empty line is a rapid
                int nextIdx = evt.blockIndex + 1;
                while (nextIdx < lines.Count)
                {
                    string next = lines[nextIdx].Trim().ToUpperInvariant();
                    if (string.IsNullOrEmpty(next) || next.StartsWith("("))
                    {
                        nextIdx++;
                        continue;
                    }
                    if (next.Contains("G0 ") || next.StartsWith("G0") || next.Contains("G00"))
                    {
                        report.suggestions.Add(new DwellOptimizationSuggestion
                        {
                            description = "Dwell before rapid move may be unnecessary",
                            blockIndex = evt.blockIndex,
                            timeSavingSeconds = evt.durationSeconds,
                        });
                    }
                    break;
                }

                // Flag excessive dwells
                if (evt.isExcessive)
                {
                    report.suggestions.Add(new DwellOptimizationSuggestion
                    {
                        description = $"Excessive dwell ({evt.durationSeconds:F1}s > {_excessiveThresholdSec:F1}s threshold)",
                        blockIndex = evt.blockIndex,
                        timeSavingSeconds = evt.durationSeconds - _excessiveThresholdSec,
                    });
                }
            }

            // Suggest consolidating consecutive explicit dwells
            for (int i = 1; i < report.events.Count; i++)
            {
                if (report.events[i].type == DwellType.Explicit &&
                    report.events[i - 1].type == DwellType.Explicit &&
                    report.events[i].blockIndex == report.events[i - 1].blockIndex + 1)
                {
                    report.suggestions.Add(new DwellOptimizationSuggestion
                    {
                        description = "Consecutive dwells could be consolidated into one",
                        blockIndex = report.events[i].blockIndex,
                        timeSavingSeconds = 0f,
                    });
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Arc Fitting Optimizer
    // -----------------------------------------------------------------------

    /// <summary>Plane on which an arc lies.</summary>
    public enum ArcPlane
    {
        XY,
        XZ,
        YZ
    }

    /// <summary>Segment type emitted by the arc fitting optimizer.</summary>
    public enum PathSegmentType
    {
        LINE,
        ARC
    }

    /// <summary>
    /// Candidate arc produced by least-squares circle fitting.
    /// Contains the geometric parameters needed to emit a G2/G3 command.
    /// </summary>
    [Serializable]
    public struct ArcCandidate
    {
        /// <summary>Center of the fitted circle (mm).</summary>
        public Vector3 center;
        /// <summary>Radius of the fitted circle (mm).</summary>
        public float radius;
        /// <summary>Start angle of the arc (radians).</summary>
        public float startAngle;
        /// <summary>Sweep angle of the arc (radians, positive = CCW).</summary>
        public float sweepAngle;
        /// <summary>Plane on which the arc lies.</summary>
        public ArcPlane plane;
        /// <summary>First point of the arc.</summary>
        public Vector3 startPoint;
        /// <summary>Last point of the arc.</summary>
        public Vector3 endPoint;
        /// <summary>Maximum deviation of any source point from the fitted circle (mm).</summary>
        public float maxDeviation;
    }

    /// <summary>
    /// A single segment in the optimised tool path — either a straight line
    /// or a circular arc (G2/G3).
    /// </summary>
    [Serializable]
    public class PathSegment
    {
        /// <summary>Whether this segment is a line or an arc.</summary>
        public PathSegmentType segmentType;
        /// <summary>Start position (mm).</summary>
        public Vector3 startPoint;
        /// <summary>End position (mm).</summary>
        public Vector3 endPoint;
        /// <summary>Arc center (mm). Only meaningful when segmentType == ARC.</summary>
        public Vector3 center;
        /// <summary>Arc radius (mm). Only meaningful when segmentType == ARC.</summary>
        public float radius;
        /// <summary>True for clockwise (G2), false for counter-clockwise (G3). Only meaningful for arcs.</summary>
        public bool isClockwise;
    }

    /// <summary>
    /// Aggregate result of the arc-fitting optimization pass.
    /// </summary>
    [Serializable]
    public class ArcFitResult
    {
        /// <summary>Number of linear segments in the original path.</summary>
        public int originalSegmentCount;
        /// <summary>Number of arcs that replaced linear segment runs.</summary>
        public int fittedArcCount;
        /// <summary>Percentage reduction in segment count.</summary>
        public float lineReductionPct;
        /// <summary>Sum of max-deviation values across all fitted arcs (mm).</summary>
        public float totalDeviation;
    }

    /// <summary>
    /// Replaces sequences of short linear G1 moves with circular arcs (G2/G3)
    /// where the points are coplanar and lie within a configurable tolerance of
    /// a least-squares fitted circle. This reduces segment count and allows the
    /// CNC controller to maintain higher feed rates through smooth curves.
    /// </summary>
    public class ArcFittingOptimizer
    {
        /// <summary>Maximum allowed deviation (mm) between source points and the fitted arc.</summary>
        public float tolerance = 0.005f;

        /// <summary>Tolerance (mm) for coplanarity check — all points must lie within this
        /// distance of the candidate plane to be considered coplanar.</summary>
        public float planeTolerance = 0.001f;

        /// <summary>Minimum number of points required to attempt a circle fit.</summary>
        public int minPointsForArc = 3;

        // ── Plane detection ──────────────────────────────────────────────

        /// <summary>
        /// Determine which principal plane (XY, XZ, YZ) a set of points lies on.
        /// Returns true if all points are within <see cref="planeTolerance"/> of
        /// one of the three planes; outputs the detected plane.
        /// </summary>
        public bool DetectPlane(List<Vector3> points, out ArcPlane plane)
        {
            plane = ArcPlane.XY;
            if (points == null || points.Count < 2) return false;

            // Compute the range (span) along each axis
            float minX = float.MaxValue, maxX = float.MinValue;
            float minY = float.MaxValue, maxY = float.MinValue;
            float minZ = float.MaxValue, maxZ = float.MinValue;

            for (int i = 0; i < points.Count; i++)
            {
                var p = points[i];
                if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
                if (p.z < minZ) minZ = p.z; if (p.z > maxZ) maxZ = p.z;
            }

            float spanX = maxX - minX;
            float spanY = maxY - minY;
            float spanZ = maxZ - minZ;

            // The flat axis is the one with the smallest span
            // XY plane => Z is flat, XZ plane => Y is flat, YZ plane => X is flat
            if (spanZ <= spanX && spanZ <= spanY)
            {
                plane = ArcPlane.XY;
                return spanZ <= planeTolerance;
            }
            if (spanY <= spanX && spanY <= spanZ)
            {
                plane = ArcPlane.XZ;
                return spanY <= planeTolerance;
            }
            // spanX is smallest
            plane = ArcPlane.YZ;
            return spanX <= planeTolerance;
        }

        // ── 2-D coordinate extraction ────────────────────────────────────

        /// <summary>
        /// Extract the two in-plane coordinates for a point given the arc plane.
        /// </summary>
        private static void GetPlaneCoords(Vector3 p, ArcPlane plane, out float u, out float v)
        {
            switch (plane)
            {
                case ArcPlane.XY: u = p.x; v = p.y; break;
                case ArcPlane.XZ: u = p.x; v = p.z; break;
                case ArcPlane.YZ: u = p.y; v = p.z; break;
                default: u = p.x; v = p.y; break;
            }
        }

        // ── Least-squares circle fitting ─────────────────────────────────

        /// <summary>
        /// Fit a circle to a set of 2-D points using the algebraic least-squares
        /// method (Kasa method). Minimises the sum of (distance - radius)².
        /// Returns false if the fit is degenerate.
        /// </summary>
        public bool FitCircle(List<Vector3> points, ArcPlane plane,
            out float centerU, out float centerV, out float radius)
        {
            centerU = 0f; centerV = 0f; radius = 0f;
            int n = points.Count;
            if (n < 3) return false;

            // Kasa method: solve [ sum(ui²+vi²)*ui  ] = A * [a, b, c]^T
            // where circle equation: u² + v² + a*u + b*v + c = 0
            // center = (-a/2, -b/2), radius = sqrt(a²/4 + b²/4 - c)
            double sumU = 0, sumV = 0, sumU2 = 0, sumV2 = 0;
            double sumUV = 0, sumU3 = 0, sumV3 = 0, sumU2V = 0, sumUV2 = 0;

            for (int i = 0; i < n; i++)
            {
                GetPlaneCoords(points[i], plane, out float uf, out float vf);
                double u = uf, v = vf;
                double u2 = u * u, v2 = v * v;
                sumU += u; sumV += v;
                sumU2 += u2; sumV2 += v2;
                sumUV += u * v;
                sumU3 += u2 * u; sumV3 += v2 * v;
                sumU2V += u2 * v; sumUV2 += u * v2;
            }

            // Build the 3x3 normal equations:
            // | sumU2  sumUV  sumU | |a|   | -(sumU3 + sumUV2) |
            // | sumUV  sumV2  sumV | |b| = | -(sumU2V + sumV3) |
            // | sumU   sumV   n    | |c|   | -(sumU2 + sumV2)  |
            double[,] A = new double[3, 3];
            double[] B = new double[3];

            A[0, 0] = sumU2; A[0, 1] = sumUV; A[0, 2] = sumU;
            A[1, 0] = sumUV; A[1, 1] = sumV2; A[1, 2] = sumV;
            A[2, 0] = sumU;  A[2, 1] = sumV;  A[2, 2] = n;

            B[0] = -(sumU3 + sumUV2);
            B[1] = -(sumU2V + sumV3);
            B[2] = -(sumU2 + sumV2);

            // Solve via Cramer's rule
            double det = Det3(A);
            if (System.Math.Abs(det) < 1e-12) return false;

            double a = Det3Substituted(A, B, 0) / det;
            double b = Det3Substituted(A, B, 1) / det;
            double c = Det3Substituted(A, B, 2) / det;

            centerU = (float)(-a * 0.5);
            centerV = (float)(-b * 0.5);
            double rSquared = a * a * 0.25 + b * b * 0.25 - c;
            if (rSquared < 0) return false;
            radius = (float)System.Math.Sqrt(rSquared);
            return radius > 1e-6f;
        }

        /// <summary>Determinant of a 3x3 matrix.</summary>
        private static double Det3(double[,] m)
        {
            return m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
                 - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
                 + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
        }

        /// <summary>Determinant with column <paramref name="col"/> replaced by <paramref name="b"/>.</summary>
        private static double Det3Substituted(double[,] m, double[] b, int col)
        {
            double[,] tmp = (double[,])m.Clone();
            for (int r = 0; r < 3; r++) tmp[r, col] = b[r];
            return Det3(tmp);
        }

        // ── Build ArcCandidate ───────────────────────────────────────────

        /// <summary>
        /// Build an <see cref="ArcCandidate"/> from a fitted circle and the source
        /// points. Computes start angle, sweep angle, max deviation, and direction.
        /// </summary>
        public ArcCandidate BuildCandidate(
            List<Vector3> points, ArcPlane plane,
            float centerU, float centerV, float radius)
        {
            var candidate = new ArcCandidate
            {
                plane = plane,
                startPoint = points[0],
                endPoint = points[points.Count - 1],
                radius = radius,
            };

            // Set center in 3-D — place the out-of-plane coordinate at the
            // average value so the center is on the same plane as the points.
            float outOfPlane = 0f;
            for (int i = 0; i < points.Count; i++)
            {
                switch (plane)
                {
                    case ArcPlane.XY: outOfPlane += points[i].z; break;
                    case ArcPlane.XZ: outOfPlane += points[i].y; break;
                    case ArcPlane.YZ: outOfPlane += points[i].x; break;
                }
            }
            outOfPlane /= points.Count;

            switch (plane)
            {
                case ArcPlane.XY: candidate.center = new Vector3(centerU, centerV, outOfPlane); break;
                case ArcPlane.XZ: candidate.center = new Vector3(centerU, outOfPlane, centerV); break;
                case ArcPlane.YZ: candidate.center = new Vector3(outOfPlane, centerU, centerV); break;
            }

            // Angles
            GetPlaneCoords(points[0], plane, out float su, out float sv);
            GetPlaneCoords(points[points.Count - 1], plane, out float eu, out float ev);
            candidate.startAngle = Mathf.Atan2(sv - centerV, su - centerU);

            float endAngle = Mathf.Atan2(ev - centerV, eu - centerU);

            // Determine sweep direction from point ordering (cross-product sign)
            float sweepCCW = endAngle - candidate.startAngle;
            if (sweepCCW < 0) sweepCCW += Mathf.PI * 2f;
            float sweepCW = sweepCCW - Mathf.PI * 2f; // negative

            // Use cross-product of successive chords to determine winding
            if (points.Count >= 3)
            {
                GetPlaneCoords(points[1], plane, out float mu, out float mv);
                float cross = (mu - su) * (ev - sv) - (mv - sv) * (eu - su);
                candidate.sweepAngle = cross >= 0 ? sweepCCW : sweepCW;
            }
            else
            {
                candidate.sweepAngle = sweepCCW;
            }

            // Max deviation
            float maxDev = 0f;
            for (int i = 0; i < points.Count; i++)
            {
                GetPlaneCoords(points[i], plane, out float pu, out float pv);
                float dist = Mathf.Sqrt((pu - centerU) * (pu - centerU) + (pv - centerV) * (pv - centerV));
                float dev = Mathf.Abs(dist - radius);
                if (dev > maxDev) maxDev = dev;
            }
            candidate.maxDeviation = maxDev;

            return candidate;
        }

        // ── High-level optimisation entry point ──────────────────────────

        /// <summary>
        /// Analyse a sequence of linear toolpath points, replace runs that
        /// approximate circular arcs with arc segments, and return an optimised
        /// list of <see cref="PathSegment"/> objects. Also populates an
        /// <see cref="ArcFitResult"/> with aggregate metrics.
        /// </summary>
        public List<PathSegment> OptimizeToolPath(List<Vector3> points, float tolerance)
        {
            var segments = new List<PathSegment>();
            if (points == null || points.Count < 2)
                return segments;

            this.tolerance = tolerance;
            int n = points.Count;
            int i = 0;

            while (i < n - 1)
            {
                // Try to find the longest run starting at i that fits an arc
                int bestEnd = -1;
                ArcCandidate bestCandidate = default;

                if (i + minPointsForArc - 1 < n)
                {
                    // Expand window from minimum size outward
                    for (int end = i + minPointsForArc - 1; end < n; end++)
                    {
                        var window = points.GetRange(i, end - i + 1);

                        if (!DetectPlane(window, out ArcPlane plane))
                            break;

                        if (!FitCircle(window, plane, out float cu, out float cv, out float r))
                            break;

                        var candidate = BuildCandidate(window, plane, cu, cv, r);

                        if (candidate.maxDeviation <= tolerance)
                        {
                            bestEnd = end;
                            bestCandidate = candidate;
                        }
                        else
                        {
                            break; // deviation exceeded — stop expanding
                        }
                    }
                }

                if (bestEnd >= 0)
                {
                    // Emit an arc segment
                    segments.Add(new PathSegment
                    {
                        segmentType = PathSegmentType.ARC,
                        startPoint = bestCandidate.startPoint,
                        endPoint = bestCandidate.endPoint,
                        center = bestCandidate.center,
                        radius = bestCandidate.radius,
                        isClockwise = bestCandidate.sweepAngle < 0,
                    });
                    i = bestEnd; // advance past the arc
                }
                else
                {
                    // Emit a single line segment
                    segments.Add(new PathSegment
                    {
                        segmentType = PathSegmentType.LINE,
                        startPoint = points[i],
                        endPoint = points[i + 1],
                    });
                    i++;
                }
            }

            return segments;
        }

        /// <summary>
        /// Run <see cref="OptimizeToolPath"/> and return both the segment list and
        /// the aggregate <see cref="ArcFitResult"/>.
        /// </summary>
        public ArcFitResult Analyze(List<Vector3> points, float tolerance)
        {
            var result = new ArcFitResult();
            if (points == null || points.Count < 2)
                return result;

            result.originalSegmentCount = points.Count - 1;

            var segments = OptimizeToolPath(points, tolerance);
            int arcCount = 0;
            float totalDev = 0f;

            // Recompute total deviation by re-fitting each arc window
            int idx = 0;
            foreach (var seg in segments)
            {
                if (seg.segmentType == PathSegmentType.ARC)
                {
                    arcCount++;
                    // Find the sub-range of original points that form this arc
                    int arcStart = idx;
                    int arcEnd = arcStart;
                    for (int j = arcStart; j < points.Count; j++)
                    {
                        if (points[j] == seg.endPoint)
                        {
                            arcEnd = j;
                            break;
                        }
                    }

                    var window = points.GetRange(arcStart, arcEnd - arcStart + 1);
                    if (DetectPlane(window, out ArcPlane plane) &&
                        FitCircle(window, plane, out float cu, out float cv, out float r))
                    {
                        var cand = BuildCandidate(window, plane, cu, cv, r);
                        totalDev += cand.maxDeviation;
                    }

                    idx = arcEnd;
                }
                else
                {
                    idx++;
                }
            }

            result.fittedArcCount = arcCount;
            result.totalDeviation = totalDev;

            if (result.originalSegmentCount > 0)
            {
                result.lineReductionPct =
                    (1f - (float)segments.Count / result.originalSegmentCount) * 100f;
            }

            return result;
        }
    }

    // ────────────────────────────────────────────────────────────────────
    //  Fixture Library Manager
    // ────────────────────────────────────────────────────────────────────

    /// <summary>
    /// Defines a workholding fixture with its physical characteristics,
    /// clamping capacity, and machine compatibility.
    /// </summary>
    [Serializable]
    public class FixtureDefinition
    {
        /// <summary>Unique identifier for this fixture.</summary>
        public string fixtureId = "";
        /// <summary>Human-readable name of the fixture.</summary>
        public string name = "";
        /// <summary>Type of fixture: vise, chuck, vacuum, fixture_plate, tombstone.</summary>
        public string fixtureType = "";
        /// <summary>Maximum clamping force in newtons.</summary>
        public float maxClampingForceN;
        /// <summary>Jaw width in millimetres (vise-specific, 0 for others).</summary>
        public float jawWidthMm;
        /// <summary>Maximum workpiece diameter (mm) the fixture can hold.</summary>
        public float maxWorkpieceDiaMm;
        /// <summary>Minimum workpiece diameter (mm) the fixture can hold.</summary>
        public float minWorkpieceDiaMm;
        /// <summary>Repeatability of the fixture in millimetres.</summary>
        public float repeatabilityMm;
        /// <summary>Typical setup time in minutes.</summary>
        public float setupTimeMin;
        /// <summary>List of machine IDs this fixture is compatible with.</summary>
        public List<string> compatibleMachines = new();
    }

    /// <summary>
    /// A scored fixture recommendation with reasons explaining the suitability rating.
    /// </summary>
    [Serializable]
    public class FixtureRecommendation
    {
        /// <summary>The recommended fixture definition.</summary>
        public FixtureDefinition fixture;
        /// <summary>Suitability score from 0 (unsuitable) to 100 (ideal).</summary>
        public float suitabilityScore;
        /// <summary>Human-readable reasons supporting this recommendation.</summary>
        public List<string> reasons = new();
    }

    /// <summary>
    /// Manages a library of workholding fixture definitions, provides CRUD
    /// operations, fixture recommendations based on workpiece requirements,
    /// and side-by-side fixture comparison.
    /// </summary>
    public class FixtureLibraryManager
    {
        private readonly Dictionary<string, FixtureDefinition> _fixtures = new();

        /// <summary>
        /// Creates a new FixtureLibraryManager pre-loaded with five standard fixtures.
        /// </summary>
        public FixtureLibraryManager()
        {
            LoadDefaults();
        }

        // ── CRUD ────────────────────────────────────────────────────────

        /// <summary>Add a fixture definition to the library.</summary>
        public void AddFixture(FixtureDefinition def)
        {
            if (def == null) throw new ArgumentNullException(nameof(def));
            if (string.IsNullOrEmpty(def.fixtureId))
                throw new ArgumentException("fixtureId must not be empty");
            _fixtures[def.fixtureId] = def;
        }

        /// <summary>Remove a fixture by its ID. Returns true if found and removed.</summary>
        public bool RemoveFixture(string id)
        {
            if (string.IsNullOrEmpty(id)) return false;
            return _fixtures.Remove(id);
        }

        /// <summary>Retrieve a fixture by its ID. Returns null if not found.</summary>
        public FixtureDefinition GetFixture(string id)
        {
            if (string.IsNullOrEmpty(id)) return null;
            return _fixtures.TryGetValue(id, out var def) ? def : null;
        }

        /// <summary>Returns all fixture definitions in the library.</summary>
        public List<FixtureDefinition> GetAllFixtures()
        {
            return new List<FixtureDefinition>(_fixtures.Values);
        }

        // ── Query / Recommendation ─────────────────────────────────────

        /// <summary>Filter fixtures by type (e.g. "vise", "chuck").</summary>
        public List<FixtureDefinition> GetFixturesByType(string fixtureType)
        {
            var results = new List<FixtureDefinition>();
            foreach (var f in _fixtures.Values)
            {
                if (string.Equals(f.fixtureType, fixtureType, StringComparison.OrdinalIgnoreCase))
                    results.Add(f);
            }
            return results;
        }

        /// <summary>
        /// Recommend fixtures for a given workpiece diameter, required clamping force,
        /// and machine identifier.  Returns a list of <see cref="FixtureRecommendation"/>
        /// sorted by descending suitability score.
        /// </summary>
        public List<FixtureRecommendation> RecommendFixture(
            float workpieceDiaMm, float requiredForceN, string machineId)
        {
            var recommendations = new List<FixtureRecommendation>();

            foreach (var fixture in _fixtures.Values)
            {
                float score = 0f;
                var reasons = new List<string>();

                // ── Machine compatibility (hard filter — 0 if incompatible) ───
                bool machineOk = fixture.compatibleMachines.Count == 0 ||
                    fixture.compatibleMachines.Contains(machineId);
                if (machineOk)
                {
                    score += 30f;
                    reasons.Add("Compatible with machine " + machineId);
                }
                else
                {
                    reasons.Add("Not compatible with machine " + machineId);
                }

                // ── Clamping force capacity ────────────────────────────────
                if (fixture.maxClampingForceN >= requiredForceN)
                {
                    float forceRatio = requiredForceN / Mathf.Max(fixture.maxClampingForceN, 1f);
                    // Best score when force requirement is 50-80% of capacity
                    float forceScore;
                    if (forceRatio <= 0.8f && forceRatio >= 0.3f)
                        forceScore = 30f;
                    else if (forceRatio < 0.3f)
                        forceScore = 30f * (forceRatio / 0.3f);
                    else
                        forceScore = 30f * ((1f - forceRatio) / 0.2f);
                    score += forceScore;
                    reasons.Add($"Force capacity {fixture.maxClampingForceN}N meets requirement of {requiredForceN}N");
                }
                else
                {
                    reasons.Add($"Insufficient clamping force ({fixture.maxClampingForceN}N < {requiredForceN}N)");
                }

                // ── Workpiece size range ───────────────────────────────────
                if (workpieceDiaMm >= fixture.minWorkpieceDiaMm &&
                    workpieceDiaMm <= fixture.maxWorkpieceDiaMm)
                {
                    float range = fixture.maxWorkpieceDiaMm - fixture.minWorkpieceDiaMm;
                    float mid = (fixture.maxWorkpieceDiaMm + fixture.minWorkpieceDiaMm) * 0.5f;
                    float distFromMid = Mathf.Abs(workpieceDiaMm - mid);
                    float sizeScore = range > 0f
                        ? 25f * (1f - distFromMid / (range * 0.5f))
                        : 25f;
                    score += Mathf.Max(sizeScore, 5f);
                    reasons.Add("Workpiece diameter within fixture range");
                }
                else
                {
                    reasons.Add($"Workpiece diameter {workpieceDiaMm}mm outside range " +
                        $"[{fixture.minWorkpieceDiaMm}–{fixture.maxWorkpieceDiaMm}mm]");
                }

                // ── Repeatability bonus ────────────────────────────────────
                if (fixture.repeatabilityMm <= 0.01f)
                {
                    score += 10f;
                    reasons.Add("Excellent repeatability");
                }
                else if (fixture.repeatabilityMm <= 0.025f)
                {
                    score += 7f;
                    reasons.Add("Good repeatability");
                }
                else
                {
                    score += 3f;
                    reasons.Add("Moderate repeatability");
                }

                // ── Setup time bonus ───────────────────────────────────────
                if (fixture.setupTimeMin <= 5f)
                {
                    score += 5f;
                    reasons.Add("Quick setup time");
                }
                else if (fixture.setupTimeMin <= 15f)
                {
                    score += 3f;
                    reasons.Add("Moderate setup time");
                }
                else
                {
                    score += 1f;
                    reasons.Add("Lengthy setup time");
                }

                score = Mathf.Clamp(score, 0f, 100f);

                recommendations.Add(new FixtureRecommendation
                {
                    fixture = fixture,
                    suitabilityScore = score,
                    reasons = reasons,
                });
            }

            // Sort descending by suitability score
            recommendations.Sort((a, b) => b.suitabilityScore.CompareTo(a.suitabilityScore));
            return recommendations;
        }

        /// <summary>
        /// Compare two fixtures side-by-side.  Returns a dictionary mapping
        /// attribute names to a tuple of (fixture1Value, fixture2Value) strings.
        /// Returns null if either fixture ID is not found.
        /// </summary>
        public Dictionary<string, (string, string)> CompareFixtures(string id1, string id2)
        {
            var f1 = GetFixture(id1);
            var f2 = GetFixture(id2);
            if (f1 == null || f2 == null)
                return null;

            var diff = new Dictionary<string, (string, string)>();
            diff["name"] = (f1.name, f2.name);
            diff["fixtureType"] = (f1.fixtureType, f2.fixtureType);
            diff["maxClampingForceN"] = (f1.maxClampingForceN.ToString("F1"), f2.maxClampingForceN.ToString("F1"));
            diff["jawWidthMm"] = (f1.jawWidthMm.ToString("F1"), f2.jawWidthMm.ToString("F1"));
            diff["maxWorkpieceDiaMm"] = (f1.maxWorkpieceDiaMm.ToString("F1"), f2.maxWorkpieceDiaMm.ToString("F1"));
            diff["minWorkpieceDiaMm"] = (f1.minWorkpieceDiaMm.ToString("F1"), f2.minWorkpieceDiaMm.ToString("F1"));
            diff["repeatabilityMm"] = (f1.repeatabilityMm.ToString("F4"), f2.repeatabilityMm.ToString("F4"));
            diff["setupTimeMin"] = (f1.setupTimeMin.ToString("F1"), f2.setupTimeMin.ToString("F1"));
            diff["compatibleMachines"] = (
                string.Join(",", f1.compatibleMachines),
                string.Join(",", f2.compatibleMachines));

            return diff;
        }

        // ── Default fixtures ───────────────────────────────────────────

        private void LoadDefaults()
        {
            AddFixture(new FixtureDefinition
            {
                fixtureId = "KURT-DL640",
                name = "Kurt DL640 6\" Double Lock Vise",
                fixtureType = "vise",
                maxClampingForceN = 44480f,   // ~10,000 lbf
                jawWidthMm = 152.4f,          // 6"
                maxWorkpieceDiaMm = 152.4f,
                minWorkpieceDiaMm = 5.0f,
                repeatabilityMm = 0.0127f,    // 0.0005"
                setupTimeMin = 5f,
                compatibleMachines = new List<string>
                    { "VMC-500", "VMC-750", "VMC-1000", "HMC-500" },
            });

            AddFixture(new FixtureDefinition
            {
                fixtureId = "3JAW-200",
                name = "200mm 3-Jaw Universal Chuck",
                fixtureType = "chuck",
                maxClampingForceN = 55000f,
                jawWidthMm = 0f,
                maxWorkpieceDiaMm = 200f,
                minWorkpieceDiaMm = 10f,
                repeatabilityMm = 0.025f,
                setupTimeMin = 10f,
                compatibleMachines = new List<string>
                    { "LATHE-200", "LATHE-300", "MILL-TURN-500" },
            });

            AddFixture(new FixtureDefinition
            {
                fixtureId = "VAC-TABLE-600",
                name = "600x400mm Vacuum Table",
                fixtureType = "vacuum",
                maxClampingForceN = 8000f,
                jawWidthMm = 0f,
                maxWorkpieceDiaMm = 600f,
                minWorkpieceDiaMm = 50f,
                repeatabilityMm = 0.05f,
                setupTimeMin = 3f,
                compatibleMachines = new List<string>
                    { "VMC-500", "VMC-750", "VMC-1000", "ROUTER-1200" },
            });

            AddFixture(new FixtureDefinition
            {
                fixtureId = "MOD-PLATE-400",
                name = "400x400mm Modular Fixture Plate",
                fixtureType = "fixture_plate",
                maxClampingForceN = 35000f,
                jawWidthMm = 0f,
                maxWorkpieceDiaMm = 380f,
                minWorkpieceDiaMm = 20f,
                repeatabilityMm = 0.005f,
                setupTimeMin = 15f,
                compatibleMachines = new List<string>
                    { "VMC-500", "VMC-750", "VMC-1000", "HMC-500", "5AX-400" },
            });

            AddFixture(new FixtureDefinition
            {
                fixtureId = "TOMB-4SIDE-300",
                name = "300mm 4-Sided Tombstone",
                fixtureType = "tombstone",
                maxClampingForceN = 60000f,
                jawWidthMm = 0f,
                maxWorkpieceDiaMm = 280f,
                minWorkpieceDiaMm = 15f,
                repeatabilityMm = 0.01f,
                setupTimeMin = 25f,
                compatibleMachines = new List<string>
                    { "HMC-500", "HMC-630", "5AX-400" },
            });
        }
    }
}
