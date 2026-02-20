#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using MiracleTwin.CNC;

namespace MiracleTwin.Editor
{
    public static class CNCProfileCreator
    {
        private const string ProfileFolder = "Assets/ScriptableObjects/CNCProfiles";

        [MenuItem("MIRACLE/Create CNC Profiles")]
        public static void CreateAllProfiles()
        {
            EnsureFolderExists(ProfileFolder);

            int created = 0;
            created += CreateBantamExplorerProfile();
            created += CreateCoastRunnerCR1Profile();

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            if (created > 0)
                Debug.Log($"[CNCProfileCreator] Created {created} CNC profile(s).");
            else
                Debug.Log("[CNCProfileCreator] All CNC profiles already exist.");
        }

        private static int CreateBantamExplorerProfile()
        {
            string path = $"{ProfileFolder}/BantamExplorerProfile.asset";
            if (AssetDatabase.LoadAssetAtPath<CNCMachineProfileSO>(path) != null)
            {
                Debug.Log($"[CNCProfileCreator] Skipped (already exists): {path}");
                return 0;
            }

            var profile = ScriptableObject.CreateInstance<CNCMachineProfileSO>();
            profile.machineName = "Bantam Desktop Explorer";
            profile.rosTopicId = "cnc1";
            profile.minPosition = Vector3.zero;
            profile.maxPosition = new Vector3(152.4f, 101.6f, 69.85f);
            profile.minRPM = 0f;
            profile.maxRPM = 23000f;
            profile.spindleType = "BLDC";
            profile.colletType = "ER-11";
            profile.overallDimensionsMM = new Vector3(400f, 387f, 260f);
            profile.weightKg = 15.4f;
            profile.defaultWorkpieceSizeMM = new Vector3(76.2f, 50.8f, 76.2f);
            profile.defaultMaterial = "6061-T6 Aluminum";

            AssetDatabase.CreateAsset(profile, path);
            Debug.Log($"[CNCProfileCreator] Created: {path}");
            return 1;
        }

        private static int CreateCoastRunnerCR1Profile()
        {
            string path = $"{ProfileFolder}/CoastRunnerCR1Profile.asset";
            if (AssetDatabase.LoadAssetAtPath<CNCMachineProfileSO>(path) != null)
            {
                Debug.Log($"[CNCProfileCreator] Skipped (already exists): {path}");
                return 0;
            }

            var profile = ScriptableObject.CreateInstance<CNCMachineProfileSO>();
            profile.machineName = "CoastRunner CR-1";
            profile.rosTopicId = "cnc2";
            profile.minPosition = Vector3.zero;
            profile.maxPosition = new Vector3(89f, 242f, 79f);
            profile.minRPM = 1500f;
            profile.maxRPM = 8000f;
            profile.spindleType = "VFD";
            profile.colletType = "ER-11";
            profile.overallDimensionsMM = new Vector3(508f, 406f, 330f);
            profile.weightKg = 19f;
            profile.defaultWorkpieceSizeMM = new Vector3(60f, 100f, 50f);
            profile.defaultMaterial = "6061-T6 Aluminum";

            AssetDatabase.CreateAsset(profile, path);
            Debug.Log($"[CNCProfileCreator] Created: {path}");
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
                {
                    AssetDatabase.CreateFolder(current, parts[i]);
                }
                current = next;
            }
        }
    }
}
#endif
