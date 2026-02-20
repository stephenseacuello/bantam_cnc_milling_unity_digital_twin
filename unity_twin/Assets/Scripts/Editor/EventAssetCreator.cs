#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using MiracleTwin.Core;

namespace MiracleTwin.Editor
{
    public static class EventAssetCreator
    {
        private const string EventFolder = "Assets/ScriptableObjects/Events";

        [MenuItem("MIRACLE/Create All Event Assets")]
        public static void CreateAllEventAssets()
        {
            EnsureFolderExists(EventFolder);

            int createdCount = 0;

            createdCount += CreateEventAsset<MachineStateEventSO>("MachineStateEvent");
            createdCount += CreateEventAsset<SystemKPIsEventSO>("SystemKPIsEvent");
            createdCount += CreateEventAsset<CuttingStateEventSO>("CuttingStateEvent");
            createdCount += CreateEventAsset<AnomalyAlertEventSO>("AnomalyAlertEvent");
            createdCount += CreateEventAsset<ToolWearEventSO>("ToolWearEvent");
            createdCount += CreateEventAsset<TwinSyncEventSO>("TwinSyncEvent");
            createdCount += CreateEventAsset<JobStatusEventSO>("JobStatusEvent");
            createdCount += CreateEventAsset<TaskAwardEventSO>("TaskAwardEvent");
            createdCount += CreateEventAsset<SecurityAlertEventSO>("SecurityAlertEvent");
            createdCount += CreateEventAsset<RobotJointStateEventSO>("RobotJointStateEvent");

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            if (createdCount > 0)
            {
                Debug.Log($"[EventAssetCreator] Created {createdCount} event asset(s).");
            }
            else
            {
                Debug.Log("[EventAssetCreator] All event assets already exist. Nothing created.");
            }
        }

        private static int CreateEventAsset<T>(string assetName) where T : ScriptableObject
        {
            string path = $"{EventFolder}/{assetName}.asset";

            if (AssetDatabase.LoadAssetAtPath<T>(path) != null)
            {
                Debug.Log($"[EventAssetCreator] Skipped (already exists): {path}");
                return 0;
            }

            T instance = ScriptableObject.CreateInstance<T>();
            AssetDatabase.CreateAsset(instance, path);
            Debug.Log($"[EventAssetCreator] Created: {path}");
            return 1;
        }

        private static void EnsureFolderExists(string folderPath)
        {
            string[] parts = folderPath.Split('/');
            string current = parts[0]; // "Assets"

            for (int i = 1; i < parts.Length; i++)
            {
                string next = current + "/" + parts[i];
                if (!AssetDatabase.IsValidFolder(next))
                {
                    AssetDatabase.CreateFolder(current, parts[i]);
                    Debug.Log($"[EventAssetCreator] Created folder: {next}");
                }
                current = next;
            }
        }
    }
}
#endif
