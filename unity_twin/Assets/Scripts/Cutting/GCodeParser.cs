using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;
using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>Parsed G-code command.</summary>
    public struct GCodeCommand
    {
        public string rawLine;
        public int lineNumber;
        public char type;           // 'G' or 'M'
        public float code;          // e.g., 0, 1, 2, 3 for G codes
        public Dictionary<char, float> parameters;

        public bool IsRapid => type == 'G' && code == 0;
        public bool IsLinearFeed => type == 'G' && code == 1;
        public bool IsCWArc => type == 'G' && code == 2;
        public bool IsCCWArc => type == 'G' && code == 3;
        public bool IsMotion => type == 'G' && code <= 3;
        public bool IsMCode => type == 'M';

        public float GetParam(char key, float defaultValue = 0f)
        {
            return parameters != null && parameters.TryGetValue(key, out var val) ? val : defaultValue;
        }

        public bool HasParam(char key) => parameters != null && parameters.ContainsKey(key);
    }

    /// <summary>
    /// G-code parser supporting Bantam TinyG dialect.
    /// Handles G0/G1/G2/G3/M3/M5/M30 and standard parameter words.
    /// </summary>
    public class GCodeParser
    {
        private static readonly Regex CodePattern = new(@"([GM])(\d+\.?\d*)", RegexOptions.Compiled);
        private static readonly Regex ParamPattern = new(@"([XYZIJKFSRPQHLTD])(-?\d+\.?\d*)", RegexOptions.Compiled);

        public List<GCodeCommand> Parse(string program)
        {
            var commands = new List<GCodeCommand>();
            if (string.IsNullOrEmpty(program)) return commands;

            var lines = program.Split('\n');

            for (int i = 0; i < lines.Length; i++)
            {
                var line = lines[i].Trim();

                // Skip empty lines and comments
                if (string.IsNullOrEmpty(line)) continue;
                if (line.StartsWith('(') || line.StartsWith(';') || line.StartsWith('%')) continue;

                // Remove inline comments
                int commentIdx = line.IndexOf('(');
                if (commentIdx >= 0) line = line.Substring(0, commentIdx).Trim();
                commentIdx = line.IndexOf(';');
                if (commentIdx >= 0) line = line.Substring(0, commentIdx).Trim();

                if (string.IsNullOrEmpty(line)) continue;

                var cmd = new GCodeCommand
                {
                    rawLine = lines[i].Trim(),
                    lineNumber = i + 1,
                    parameters = new Dictionary<char, float>()
                };

                // Extract G/M code
                var codeMatch = CodePattern.Match(line);
                if (codeMatch.Success)
                {
                    cmd.type = codeMatch.Groups[1].Value[0];
                    cmd.code = float.Parse(codeMatch.Groups[2].Value, CultureInfo.InvariantCulture);
                }

                // Extract all parameters
                var paramMatches = ParamPattern.Matches(line);
                foreach (Match pm in paramMatches)
                {
                    char key = pm.Groups[1].Value[0];
                    float val = float.Parse(pm.Groups[2].Value, CultureInfo.InvariantCulture);
                    cmd.parameters[key] = val;
                }

                commands.Add(cmd);
            }

            return commands;
        }

        /// <summary>Parse a single line of G-code.</summary>
        public GCodeCommand ParseLine(string line, int lineNumber = 0)
        {
            var program = line;
            var commands = Parse(program);
            if (commands.Count > 0)
            {
                var cmd = commands[0];
                cmd.lineNumber = lineNumber;
                return cmd;
            }
            return new GCodeCommand { rawLine = line, lineNumber = lineNumber, parameters = new Dictionary<char, float>() };
        }

        /// <summary>Validate a G-code program and return errors.</summary>
        public List<string> Validate(string program)
        {
            var errors = new List<string>();
            var commands = Parse(program);

            bool hasSpindleOn = false;
            bool hasProgramEnd = false;

            foreach (var cmd in commands)
            {
                // Check for motion without spindle
                if (cmd.IsLinearFeed && !hasSpindleOn)
                    errors.Add($"Line {cmd.lineNumber}: Feed move without spindle on (M3)");

                if (cmd.IsMCode && cmd.code == 3) hasSpindleOn = true;
                if (cmd.IsMCode && cmd.code == 5) hasSpindleOn = false;
                if (cmd.IsMCode && cmd.code == 30) hasProgramEnd = true;

                // Check axis limits for Bantam Explorer
                if (cmd.HasParam('X'))
                {
                    float x = cmd.GetParam('X');
                    if (x < -5 || x > 160) errors.Add($"Line {cmd.lineNumber}: X={x} out of range [-5, 160]");
                }
                if (cmd.HasParam('Y'))
                {
                    float y = cmd.GetParam('Y');
                    if (y < -5 || y > 105) errors.Add($"Line {cmd.lineNumber}: Y={y} out of range [-5, 105]");
                }
                if (cmd.HasParam('Z'))
                {
                    float z = cmd.GetParam('Z');
                    if (z < -72 || z > 10) errors.Add($"Line {cmd.lineNumber}: Z={z} out of range [-72, 10]");
                }
            }

            if (!hasProgramEnd)
                errors.Add("Warning: No M30 program end found");

            return errors;
        }
    }
}
