using System.Collections.Generic;
using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Routes replayed binary messages from ReplayController back to the
    /// appropriate GameEventSO channels. Maps FNV-1a topic hashes to
    /// deserialization + event raise logic.
    /// </summary>
    public class ReplayDispatcher : MonoBehaviour
    {
        [Header("Event Channels")]
        [SerializeField] private MachineStateEventSO machineStateEvent;
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private SystemKPIsEventSO systemKPIsEvent;
        [SerializeField] private AnomalyAlertEventSO anomalyAlertEvent;
        [SerializeField] private JobStatusEventSO jobStatusEvent;
        [SerializeField] private SecurityAlertEventSO securityAlertEvent;

        [Header("References")]
        [SerializeField] private ReplayController replayController;

        private Dictionary<uint, System.Action<byte[]>> dispatchTable;

        void Awake()
        {
            BuildDispatchTable();
        }

        void OnEnable()
        {
            if (replayController != null)
                replayController.OnMessageReplay += OnMessageReplay;
        }

        void OnDisable()
        {
            if (replayController != null)
                replayController.OnMessageReplay -= OnMessageReplay;
        }

        private void BuildDispatchTable()
        {
            dispatchTable = new Dictionary<uint, System.Action<byte[]>>();

            // Register known topics with their FNV-1a hashes
            RegisterTopic("/miracle/cnc1/state", data => {
                if (machineStateEvent != null)
                {
                    var msg = new RosMessageTypes.Miracle.MachineStateMsg();
                    msg.Deserialize(data, 0);
                    machineStateEvent.Raise(msg);
                }
            });

            RegisterTopic("/miracle/cnc1/anomaly", data => {
                if (anomalyAlertEvent != null)
                {
                    var msg = new RosMessageTypes.Miracle.AnomalyAlertMsg();
                    msg.Deserialize(data, 0);
                    anomalyAlertEvent.Raise(msg);
                }
            });

            RegisterTopic("/miracle/system_kpis", data => {
                if (systemKPIsEvent != null)
                {
                    var msg = new RosMessageTypes.Miracle.SystemKPIsMsg();
                    msg.Deserialize(data, 0);
                    systemKPIsEvent.Raise(msg);
                }
            });

            RegisterTopic("/miracle/cnc1/job_status", data => {
                if (jobStatusEvent != null)
                {
                    var msg = new RosMessageTypes.Miracle.JobStatusMsg();
                    msg.Deserialize(data, 0);
                    jobStatusEvent.Raise(msg);
                }
            });

            RegisterTopic("/miracle/security/alerts", data => {
                if (securityAlertEvent != null)
                {
                    var msg = new RosMessageTypes.Miracle.SecurityAlertMsg();
                    msg.Deserialize(data, 0);
                    securityAlertEvent.Raise(msg);
                }
            });

            // Also register cnc2, cnc3 topics
            foreach (string mid in new[] { "cnc2", "cnc3" })
            {
                RegisterTopic($"/miracle/{mid}/state", data => {
                    if (machineStateEvent != null)
                    {
                        var msg = new RosMessageTypes.Miracle.MachineStateMsg();
                        msg.Deserialize(data, 0);
                        machineStateEvent.Raise(msg);
                    }
                });

                RegisterTopic($"/miracle/{mid}/anomaly", data => {
                    if (anomalyAlertEvent != null)
                    {
                        var msg = new RosMessageTypes.Miracle.AnomalyAlertMsg();
                        msg.Deserialize(data, 0);
                        anomalyAlertEvent.Raise(msg);
                    }
                });
            }
        }

        private void RegisterTopic(string topicName, System.Action<byte[]> handler)
        {
            uint hash = HashTopic(topicName);
            dispatchTable[hash] = handler;
        }

        private void OnMessageReplay(uint topicHash, byte[] payload)
        {
            if (dispatchTable.TryGetValue(topicHash, out var handler))
            {
                try
                {
                    handler(payload);
                }
                catch (System.Exception ex)
                {
                    Debug.LogWarning($"[ReplayDispatcher] Failed to dispatch hash {topicHash}: {ex.Message}");
                }
            }
        }

        /// <summary>FNV-1a 32-bit hash of a topic name string. Must match MiracleBridge.HashTopic.</summary>
        public static uint HashTopic(string topic)
        {
            uint hash = 2166136261u;
            foreach (char c in topic)
            {
                hash ^= (byte)c;
                hash *= 16777619u;
            }
            return hash;
        }
    }
}
