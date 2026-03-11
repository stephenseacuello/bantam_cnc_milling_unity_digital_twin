using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.CNC;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Validates G-code toolpath segments against machine physical limits.
    /// Call ValidateProgram() before execution to catch out-of-bounds moves,
    /// excessive speeds, or invalid parameters.
    /// </summary>
    public static class GCodeValidator
    {
        public struct ValidationResult
        {
            public bool IsValid;
            public List<string> Warnings;
            public List<string> Errors;

            public static ValidationResult Create()
            {
                return new ValidationResult
                {
                    IsValid = true,
                    Warnings = new List<string>(),
                    Errors = new List<string>()
                };
            }
        }

        /// <summary>
        /// Validate all toolpath segments against machine limits from the CNC profile.
        /// </summary>
        public static ValidationResult ValidateProgram(
            IReadOnlyList<ToolpathSegment> segments,
            CNCMachineProfileSO profile)
        {
            var result = ValidationResult.Create();

            if (segments == null || segments.Count == 0)
            {
                result.Warnings.Add("Empty program — nothing to validate.");
                return result;
            }

            if (profile == null)
            {
                result.Warnings.Add("No machine profile assigned — skipping limit checks.");
                return result;
            }

            for (int i = 0; i < segments.Count; i++)
            {
                var seg = segments[i];
                int line = seg.gcodeLine;

                // Check travel limits (X, Y, Z)
                ValidatePosition(seg.endPos, line, profile, result);

                // Check spindle speed
                if (seg.spindleRPM > profile.maxRPM)
                {
                    result.Errors.Add(
                        $"Line {line}: Spindle speed {seg.spindleRPM:F0} RPM exceeds max {profile.maxRPM:F0}");
                    result.IsValid = false;
                }

                // Warn on zero feedrate for cutting moves (non-rapid linear moves)
                if (seg.type == SegmentType.Linear && seg.feedRate <= 0)
                {
                    result.Warnings.Add($"Line {line}: Cutting move with zero feed rate.");
                }
            }

            return result;
        }

        private static void ValidatePosition(
            Vector3 pos, int line, CNCMachineProfileSO profile, ValidationResult result)
        {
            if (pos.x < profile.minPosition.x || pos.x > profile.maxPosition.x)
            {
                result.Errors.Add(
                    $"Line {line}: X={pos.x:F3} out of travel [{profile.minPosition.x}, {profile.maxPosition.x}]");
                result.IsValid = false;
            }
            if (pos.y < profile.minPosition.y || pos.y > profile.maxPosition.y)
            {
                result.Errors.Add(
                    $"Line {line}: Y={pos.y:F3} out of travel [{profile.minPosition.y}, {profile.maxPosition.y}]");
                result.IsValid = false;
            }
            if (pos.z < profile.minPosition.z || pos.z > profile.maxPosition.z)
            {
                result.Errors.Add(
                    $"Line {line}: Z={pos.z:F3} out of travel [{profile.minPosition.z}, {profile.maxPosition.z}]");
                result.IsValid = false;
            }
        }
    }
}
