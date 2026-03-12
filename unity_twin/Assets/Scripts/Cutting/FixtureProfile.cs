using UnityEngine;
using System.Collections.Generic;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// ScriptableObject that defines fixture geometry for collision checking.
    /// Includes clamp zones, work envelope, safe retract height, and clearance settings.
    /// Used by GCodeLookahead to replace the simple AABB collision check with
    /// per-clamp oriented box tests including tool shank checks.
    /// </summary>
    [CreateAssetMenu(fileName = "NewFixtureProfile", menuName = "MIRACLE/Fixture Profile")]
    public class FixtureProfile : ScriptableObject
    {
        [Header("Fixture Identification")]
        public string fixtureId = "default";
        public string description = "";

        [Header("Clamp Zones (tool must avoid)")]
        public List<ClampZone> clampZones = new List<ClampZone>();

        [Header("Work Envelope")]
        public Bounds workEnvelope = new Bounds(Vector3.zero, new Vector3(200, 200, 100));

        [Header("Safe Retract Height")]
        public float safeRetractZ = 50f; // mm above workpiece top

        [Header("Clearance")]
        public float minToolClearance = 2f; // mm minimum clearance from fixtures

        [System.Serializable]
        public class ClampZone
        {
            public string label = "Clamp";
            public Vector3 center;
            public Vector3 size; // AABB full extents
            public float rotationY; // degrees rotation around Y axis

            /// <summary>
            /// Check whether a point (with added clearance) lies inside this clamp zone,
            /// accounting for Y-axis rotation.
            /// </summary>
            public bool ContainsPoint(Vector3 point, float clearance)
            {
                // Rotate point into clamp local space
                var rot = Quaternion.Euler(0, -rotationY, 0);
                var local = rot * (point - center);
                var halfExt = size * 0.5f + Vector3.one * clearance;
                return Mathf.Abs(local.x) <= halfExt.x
                    && Mathf.Abs(local.y) <= halfExt.y
                    && Mathf.Abs(local.z) <= halfExt.z;
            }

            public Bounds GetBounds()
            {
                return new Bounds(center, size);
            }
        }

        /// <summary>
        /// Check if a tool position collides with any fixture element.
        /// Tests the work envelope first, then each clamp zone against the tool tip
        /// and tool shank (vertical cylinder above the tip).
        /// </summary>
        public CollisionResult CheckCollision(Vector3 toolTip, float toolDiameter, float toolLength)
        {
            var result = new CollisionResult();

            // Check work envelope
            if (!workEnvelope.Contains(toolTip))
            {
                result.isCollision = true;
                result.severity = CollisionSeverity.OutOfEnvelope;
                result.message = $"Tool position {toolTip} outside work envelope";
                return result;
            }

            // Check each clamp zone with tool radius clearance
            float toolRadius = toolDiameter * 0.5f;
            foreach (var clamp in clampZones)
            {
                if (clamp.ContainsPoint(toolTip, toolRadius + minToolClearance))
                {
                    result.isCollision = true;
                    result.severity = CollisionSeverity.FixtureCollision;
                    result.collidingClamp = clamp.label;
                    result.message = $"Tool collides with {clamp.label} at {toolTip}";
                    return result;
                }

                // Check tool shank (vertical cylinder above tip)
                for (float h = 0; h < toolLength; h += 5f)
                {
                    var shankPoint = toolTip + Vector3.up * h;
                    if (clamp.ContainsPoint(shankPoint, toolRadius + minToolClearance))
                    {
                        result.isCollision = true;
                        result.severity = CollisionSeverity.ShankCollision;
                        result.collidingClamp = clamp.label;
                        result.message = $"Tool shank collides with {clamp.label} at height {h:F1}mm";
                        return result;
                    }
                }
            }

            // No collision
            result.isCollision = false;
            return result;
        }

        /// <summary>
        /// Near-miss check: returns NearMiss severity if the tool tip is within
        /// 2x the minimum clearance of any clamp zone but not actually colliding.
        /// </summary>
        public CollisionResult CheckNearMiss(Vector3 toolTip, float toolDiameter)
        {
            var result = new CollisionResult();
            float toolRadius = toolDiameter * 0.5f;
            float nearMissDistance = minToolClearance * 2f;

            foreach (var clamp in clampZones)
            {
                // Check with expanded clearance (2x) but only if not already a collision
                if (clamp.ContainsPoint(toolTip, toolRadius + nearMissDistance)
                    && !clamp.ContainsPoint(toolTip, toolRadius + minToolClearance))
                {
                    result.isCollision = false;
                    result.severity = CollisionSeverity.NearMiss;
                    result.collidingClamp = clamp.label;
                    result.message = $"Tool near-miss with {clamp.label} at {toolTip}";
                    return result;
                }
            }

            result.isCollision = false;
            result.severity = CollisionSeverity.None;
            return result;
        }

        public struct CollisionResult
        {
            public bool isCollision;
            public CollisionSeverity severity;
            public string collidingClamp;
            public string message;
        }

        public enum CollisionSeverity
        {
            None,
            NearMiss,        // Within 2x clearance
            ShankCollision,  // Tool shank hits fixture
            FixtureCollision, // Tool tip hits fixture
            OutOfEnvelope    // Outside machine bounds
        }
    }
}
