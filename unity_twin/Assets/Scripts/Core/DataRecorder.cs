using System;
using System.IO;
using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Records all incoming ROS2 message streams to a binary log file
    /// for later replay. Format: [timestamp_double][topic_hash_uint][length_int][payload_bytes]
    /// </summary>
    public class DataRecorder : MonoBehaviour
    {
        [SerializeField] private string recordingsFolder = "Recordings";

        public bool IsRecording { get; private set; }
        public string CurrentFilePath { get; private set; }
        public double RecordingDuration { get; private set; }

        private BinaryWriter writer;
        private FileStream fileStream;
        private double recordingStartTime;
        private int frameCount;

        public void StartRecording()
        {
            if (IsRecording) return;

            string folder = Path.Combine(Application.streamingAssetsPath, recordingsFolder);
            Directory.CreateDirectory(folder);

            string filename = $"miracle_{DateTime.Now:yyyyMMdd_HHmmss}.miracle";
            CurrentFilePath = Path.Combine(folder, filename);

            fileStream = new FileStream(CurrentFilePath, FileMode.Create, FileAccess.Write, FileShare.None, 65536);
            writer = new BinaryWriter(fileStream);

            // Write header
            writer.Write("MIRACLE_REC");  // Magic
            writer.Write(1);              // Version
            writer.Write(DateTime.UtcNow.ToBinary()); // Start time

            recordingStartTime = SimulationClock.Instance?.SimTime ?? 0;
            frameCount = 0;
            IsRecording = true;

            Debug.Log($"[DataRecorder] Recording started: {CurrentFilePath}");
        }

        public void StopRecording()
        {
            if (!IsRecording) return;

            IsRecording = false;
            RecordingDuration = (SimulationClock.Instance?.SimTime ?? 0) - recordingStartTime;

            writer?.Flush();
            writer?.Dispose();
            fileStream?.Dispose();
            writer = null;
            fileStream = null;

            Debug.Log($"[DataRecorder] Recording stopped. Duration: {RecordingDuration:F1}s, Frames: {frameCount}");
        }

        public void RecordMessage(uint topicHash, byte[] payload)
        {
            if (!IsRecording || writer == null) return;

            double timestamp = SimulationClock.Instance?.SimTime ?? 0;
            writer.Write(timestamp);
            writer.Write(topicHash);
            writer.Write(payload.Length);
            writer.Write(payload);
            frameCount++;
        }

        public void ToggleRecording()
        {
            if (IsRecording) StopRecording();
            else StartRecording();
        }

        void OnDestroy()
        {
            StopRecording();
        }
    }
}
