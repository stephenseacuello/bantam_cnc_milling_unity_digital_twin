using System.Collections.Generic;
using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>Toolpath segment types.</summary>
    public enum SegmentType { Rapid, Linear, CWArc, CCWArc }

    /// <summary>A single toolpath segment with start/end positions and parameters.</summary>
    public struct ToolpathSegment
    {
        public SegmentType type;
        public Vector3 startPos;     // mm
        public Vector3 endPos;       // mm
        public Vector3 arcCenter;    // mm (for arcs)
        public float feedRate;       // mm/min
        public float spindleRPM;
        public int gcodeLine;
        public float length;         // mm

        public float Duration => feedRate > 0 ? (length / feedRate) * 60f : 0f; // seconds
    }

    /// <summary>
    /// Interprets parsed G-code commands into a sequence of toolpath segments.
    /// Maintains modal state (G90/G91, current feed, spindle speed, etc.)
    /// </summary>
    public class GCodeInterpreter
    {
        // Modal state
        private bool absoluteMode = true;  // G90 default
        private bool metricMode = true;    // G21 default
        private float currentFeedRate = 1000f;
        private float currentSpindleRPM = 0f;
        private Vector3 currentPosition = Vector3.zero;
        private bool spindleOn = false;

        private GCodeParser parser = new();

        /// <summary>Convert a G-code program string into toolpath segments.</summary>
        public List<ToolpathSegment> Interpret(string program)
        {
            var commands = parser.Parse(program);
            return InterpretCommands(commands);
        }

        /// <summary>Convert parsed commands into toolpath segments.</summary>
        public List<ToolpathSegment> InterpretCommands(List<GCodeCommand> commands)
        {
            var segments = new List<ToolpathSegment>();

            foreach (var cmd in commands)
            {
                ProcessCommand(cmd, segments);
            }

            return segments;
        }

        private void ProcessCommand(GCodeCommand cmd, List<ToolpathSegment> segments)
        {
            if (cmd.type == 'G')
            {
                switch ((int)cmd.code)
                {
                    case 0: // Rapid
                        ProcessMotion(cmd, SegmentType.Rapid, segments);
                        break;
                    case 1: // Linear feed
                        ProcessMotion(cmd, SegmentType.Linear, segments);
                        break;
                    case 2: // CW arc
                        ProcessMotion(cmd, SegmentType.CWArc, segments);
                        break;
                    case 3: // CCW arc
                        ProcessMotion(cmd, SegmentType.CCWArc, segments);
                        break;
                    case 17: // XY plane (default)
                        break;
                    case 20: // Inch mode
                        metricMode = false;
                        break;
                    case 21: // Metric mode
                        metricMode = true;
                        break;
                    case 28: // Home
                        currentPosition = Vector3.zero;
                        break;
                    case 90: // Absolute
                        absoluteMode = true;
                        break;
                    case 91: // Incremental
                        absoluteMode = false;
                        break;
                }
            }
            else if (cmd.type == 'M')
            {
                switch ((int)cmd.code)
                {
                    case 3: // Spindle on CW
                    case 4: // Spindle on CCW
                        spindleOn = true;
                        if (cmd.HasParam('S'))
                            currentSpindleRPM = cmd.GetParam('S');
                        break;
                    case 5: // Spindle off
                        spindleOn = false;
                        currentSpindleRPM = 0;
                        break;
                    case 30: // Program end
                    case 2:  // Program end
                        break;
                }
            }

            // Update feed rate if F parameter present
            if (cmd.HasParam('F'))
                currentFeedRate = cmd.GetParam('F');

            // Update spindle if S parameter present
            if (cmd.HasParam('S'))
                currentSpindleRPM = cmd.GetParam('S');
        }

        private void ProcessMotion(GCodeCommand cmd, SegmentType type, List<ToolpathSegment> segments)
        {
            Vector3 target = currentPosition;
            float scale = metricMode ? 1f : 25.4f; // Convert inches to mm if needed

            if (absoluteMode)
            {
                if (cmd.HasParam('X')) target.x = cmd.GetParam('X') * scale;
                if (cmd.HasParam('Y')) target.y = cmd.GetParam('Y') * scale;
                if (cmd.HasParam('Z')) target.z = cmd.GetParam('Z') * scale;
            }
            else
            {
                if (cmd.HasParam('X')) target.x += cmd.GetParam('X') * scale;
                if (cmd.HasParam('Y')) target.y += cmd.GetParam('Y') * scale;
                if (cmd.HasParam('Z')) target.z += cmd.GetParam('Z') * scale;
            }

            float feedRate = type == SegmentType.Rapid ? 5000f : currentFeedRate;

            Vector3 arcCenter = Vector3.zero;
            float length;

            if (type == SegmentType.CWArc || type == SegmentType.CCWArc)
            {
                // Arc center offsets (incremental from start)
                arcCenter = currentPosition + new Vector3(
                    cmd.GetParam('I') * scale,
                    cmd.GetParam('J') * scale,
                    cmd.GetParam('K') * scale
                );
                float radius = Vector3.Distance(currentPosition, arcCenter);
                float angle = Vector3.Angle(currentPosition - arcCenter, target - arcCenter);
                length = radius * angle * Mathf.Deg2Rad;
            }
            else
            {
                length = Vector3.Distance(currentPosition, target);
            }

            if (length > 0.001f) // Skip zero-length moves
            {
                segments.Add(new ToolpathSegment
                {
                    type = type,
                    startPos = currentPosition,
                    endPos = target,
                    arcCenter = arcCenter,
                    feedRate = feedRate,
                    spindleRPM = currentSpindleRPM,
                    gcodeLine = cmd.lineNumber,
                    length = length
                });
            }

            currentPosition = target;
        }

        /// <summary>Reset interpreter state.</summary>
        public void Reset()
        {
            absoluteMode = true;
            metricMode = true;
            currentFeedRate = 1000f;
            currentSpindleRPM = 0;
            currentPosition = Vector3.zero;
            spindleOn = false;
        }

        /// <summary>Calculate total machining time for a set of segments.</summary>
        public static float CalculateTotalTime(List<ToolpathSegment> segments)
        {
            float total = 0;
            foreach (var seg in segments)
                total += seg.Duration;
            return total;
        }
    }
}
