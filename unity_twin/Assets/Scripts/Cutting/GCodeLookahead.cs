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
}
