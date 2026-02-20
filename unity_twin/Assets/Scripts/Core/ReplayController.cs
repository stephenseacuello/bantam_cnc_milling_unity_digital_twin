using System;
using System.IO;
using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Plays back recorded .miracle files, driving SimulationClock in Replay mode.
    /// Supports scrubbing, speed control, and rewind.
    /// </summary>
    public class ReplayController : MonoBehaviour
    {
        [SerializeField, Range(0.1f, 10f)] private float playbackSpeed = 1f;

        public bool IsLoaded { get; private set; }
        public bool IsPlaying { get; private set; }
        public double Duration { get; private set; }
        public double CurrentTime { get; private set; }
        public float Progress => Duration > 0 ? (float)(CurrentTime / Duration) : 0f;
        public float PlaybackSpeed
        {
            get => playbackSpeed;
            set => playbackSpeed = Mathf.Clamp(value, 0.1f, 10f);
        }

        public event Action<uint, byte[]> OnMessageReplay;

        private BinaryReader reader;
        private FileStream fileStream;
        private double nextMessageTime;
        private uint nextTopicHash;
        private byte[] nextPayload;
        private bool hasNextMessage;

        public bool LoadRecording(string filePath)
        {
            Unload();

            if (!File.Exists(filePath))
            {
                Debug.LogError($"[ReplayController] File not found: {filePath}");
                return false;
            }

            fileStream = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read, 65536);
            reader = new BinaryReader(fileStream);

            // Read header
            string magic = reader.ReadString();
            if (magic != "MIRACLE_REC")
            {
                Debug.LogError("[ReplayController] Invalid recording file");
                Unload();
                return false;
            }

            int version = reader.ReadInt32();
            long startTimeBinary = reader.ReadInt64();

            // Scan for duration (read to end, then reset)
            double lastTimestamp = 0;
            long dataStart = fileStream.Position;
            while (fileStream.Position < fileStream.Length)
            {
                try
                {
                    lastTimestamp = reader.ReadDouble();
                    reader.ReadUInt32(); // topic hash
                    int len = reader.ReadInt32();
                    fileStream.Seek(len, SeekOrigin.Current);
                }
                catch { break; }
            }
            Duration = lastTimestamp;
            fileStream.Seek(dataStart, SeekOrigin.Begin);

            ReadNextMessage();
            IsLoaded = true;
            CurrentTime = 0;

            Debug.Log($"[ReplayController] Loaded recording. Duration: {Duration:F1}s");
            return true;
        }

        public void Play()
        {
            if (!IsLoaded) return;
            IsPlaying = true;
            SimulationClock.Instance?.SetMode(SimulationClock.Mode.Replay);
        }

        public void PauseReplay()
        {
            IsPlaying = false;
        }

        public void Seek(float normalizedTime)
        {
            if (!IsLoaded) return;
            CurrentTime = normalizedTime * Duration;
            SimulationClock.Instance?.SeekTo(CurrentTime);
        }

        void Update()
        {
            if (!IsLoaded || !IsPlaying) return;

            double dt = Time.unscaledDeltaTime * playbackSpeed;
            CurrentTime += dt;
            SimulationClock.Instance?.AdvanceReplay(dt);

            // Dispatch messages up to current time
            int dispatched = 0;
            while (hasNextMessage && nextMessageTime <= CurrentTime && dispatched < 50)
            {
                OnMessageReplay?.Invoke(nextTopicHash, nextPayload);
                ReadNextMessage();
                dispatched++;
            }

            if (CurrentTime >= Duration)
            {
                IsPlaying = false;
                Debug.Log("[ReplayController] Playback complete");
            }
        }

        private void ReadNextMessage()
        {
            if (reader == null || fileStream.Position >= fileStream.Length)
            {
                hasNextMessage = false;
                return;
            }

            try
            {
                nextMessageTime = reader.ReadDouble();
                nextTopicHash = reader.ReadUInt32();
                int len = reader.ReadInt32();
                nextPayload = reader.ReadBytes(len);
                hasNextMessage = true;
            }
            catch
            {
                hasNextMessage = false;
            }
        }

        public void Unload()
        {
            IsPlaying = false;
            IsLoaded = false;
            reader?.Dispose();
            fileStream?.Dispose();
            reader = null;
            fileStream = null;
        }

        void OnDestroy() => Unload();
    }
}
