#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using MiracleTwin.Cutting;

namespace MiracleTwin.Editor
{
    /// <summary>
    /// Editor utility that creates ToolDefinition ScriptableObject assets
    /// with calibrated cutting coefficients from CuttingCoefficients.json.
    /// </summary>
    public static class ToolDefinitionCreator
    {
        private const string ToolFolder = "Assets/ScriptableObjects/Tools";

        [MenuItem("MIRACLE/Create Tool Definitions")]
        public static void CreateAllToolDefinitions()
        {
            EnsureFolderExists(ToolFolder);

            int created = 0;
            created += CreateHSSQuarterInch2Flute();
            created += CreateCarbideQuarterInch4Flute();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            if (created > 0)
                Debug.Log($"[ToolDefinitionCreator] Created {created} tool definition(s).");
            else
                Debug.Log("[ToolDefinitionCreator] All tool definitions already exist.");
        }

        private static int CreateHSSQuarterInch2Flute()
        {
            string path = $"{ToolFolder}/HSS_QuarterInch_2Flute.asset";
            if (AssetDatabase.LoadAssetAtPath<ToolDefinition>(path) != null)
                return 0;

            var tool = ScriptableObject.CreateInstance<ToolDefinition>();
            tool.toolId = "HSS_quarter_inch_2flute";
            tool.description = "1/4\" 2-Flute HSS End Mill (6061-T6 Al)";
            tool.diameter = 6.35f;
            tool.fluteCount = 2;
            tool.helixAngle = 30f;
            tool.rakeAngle = 10f;
            tool.reliefAngle = 12f;
            tool.noseRadius = 0.4f;
            tool.cuttingLength = 19f;
            tool.overallLength = 57f;
            tool.shankDiameter = 6.35f;
            tool.colletType = "ER-11";
            tool.maxRunout = 0.01f;
            tool.minRPM = 10000f;
            tool.maxRPM = 23000f;
            tool.recommendedRPM = 16000f;
            tool.maxFeedRate = 2000f;
            tool.recommendedFeedPerTooth = 0.05f;
            tool.maxAxialDepth = 3.0f;
            tool.maxRadialDepth = 6.35f;

            // Cutting coefficients from CuttingCoefficients.json entry 1 (6061-T6 + HSS)
            tool.Ktc = 796f;
            tool.Krc = 168f;
            tool.Kac = 80f;
            tool.Kte = 14.5f;
            tool.Kre = 10.2f;
            tool.Kae = 4.8f;

            tool.taylorC = 300f;
            tool.taylorN = 0.125f;
            tool.initialWearVB = 0.02f;
            tool.maxWearVB = 0.30f;

            AssetDatabase.CreateAsset(tool, path);
            Debug.Log($"[ToolDefinitionCreator] Created: {path}");
            return 1;
        }

        private static int CreateCarbideQuarterInch4Flute()
        {
            string path = $"{ToolFolder}/Carbide_QuarterInch_4Flute.asset";
            if (AssetDatabase.LoadAssetAtPath<ToolDefinition>(path) != null)
                return 0;

            var tool = ScriptableObject.CreateInstance<ToolDefinition>();
            tool.toolId = "Carbide_quarter_inch_4flute";
            tool.description = "1/4\" 4-Flute Carbide End Mill (6061-T6 Al)";
            tool.diameter = 6.35f;
            tool.fluteCount = 4;
            tool.helixAngle = 35f;
            tool.rakeAngle = 12f;
            tool.reliefAngle = 10f;
            tool.noseRadius = 0.2f;
            tool.cuttingLength = 19f;
            tool.overallLength = 57f;
            tool.shankDiameter = 6.35f;
            tool.colletType = "ER-11";
            tool.maxRunout = 0.005f;
            tool.minRPM = 12000f;
            tool.maxRPM = 23000f;
            tool.recommendedRPM = 18000f;
            tool.maxFeedRate = 3000f;
            tool.recommendedFeedPerTooth = 0.04f;
            tool.maxAxialDepth = 3.0f;
            tool.maxRadialDepth = 6.35f;

            // Cutting coefficients from CuttingCoefficients.json entry 2 (6061-T6 + Carbide)
            tool.Ktc = 680f;
            tool.Krc = 145f;
            tool.Kac = 72f;
            tool.Kte = 11.2f;
            tool.Kre = 7.8f;
            tool.Kae = 3.6f;

            tool.taylorC = 800f;
            tool.taylorN = 0.25f;
            tool.initialWearVB = 0.01f;
            tool.maxWearVB = 0.30f;

            AssetDatabase.CreateAsset(tool, path);
            Debug.Log($"[ToolDefinitionCreator] Created: {path}");
            return 1;
        }

        private static void EnsureFolderExists(string folderPath)
        {
            string[] parts = folderPath.Split('/');
            string current = parts[0];
            for (int i = 1; i < parts.Length; i++)
            {
                string next = current + "/" + parts[i];
                if (!AssetDatabase.IsValidFolder(next))
                    AssetDatabase.CreateFolder(current, parts[i]);
                current = next;
            }
        }
    }
}
#endif
