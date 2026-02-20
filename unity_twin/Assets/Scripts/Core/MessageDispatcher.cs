using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Thread-safe message dispatcher that marshals ROS2 callbacks to the Unity main thread.
    /// ROSConnection already handles basic thread marshaling, but this adds:
    /// - Timestamped ring buffers for Replay mode
    /// - Message skipping for Accelerated mode
    /// - Frame budget limiting (max messages processed per frame)
    /// </summary>
    public class MessageDispatcher : MonoBehaviour
    {
        public static MessageDispatcher Instance { get; private set; }

        [SerializeField] private int maxMessagesPerFrame = 50;
        [SerializeField] private int ringBufferCapacity = 120;

        private readonly ConcurrentQueue<Action> mainThreadQueue = new();
        private int messagesThisFrame;

        void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
            DontDestroyOnLoad(transform.root.gameObject);
        }

        void Update()
        {
            messagesThisFrame = 0;
            while (messagesThisFrame < maxMessagesPerFrame && mainThreadQueue.TryDequeue(out var action))
            {
                try
                {
                    action();
                }
                catch (Exception e)
                {
                    Debug.LogError($"[MessageDispatcher] Error processing message: {e}");
                }
                messagesThisFrame++;
            }
        }

        /// <summary>Enqueue an action to run on the main thread next frame.</summary>
        public void Enqueue(Action action)
        {
            mainThreadQueue.Enqueue(action);
        }

        /// <summary>Enqueue a typed message callback.</summary>
        public void Enqueue<T>(Action<T> callback, T message)
        {
            mainThreadQueue.Enqueue(() => callback(message));
        }

        public int PendingCount => mainThreadQueue.Count;

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
        }
    }

    /// <summary>
    /// Timestamped ring buffer for recording and replay.
    /// </summary>
    public class TimestampedRingBuffer<T>
    {
        private readonly (double timestamp, T value)[] buffer;
        private int head;
        private int count;

        public int Count => count;
        public int Capacity { get; }

        public TimestampedRingBuffer(int capacity)
        {
            Capacity = capacity;
            buffer = new (double, T)[capacity];
        }

        public void Add(double timestamp, T value)
        {
            buffer[head] = (timestamp, value);
            head = (head + 1) % Capacity;
            if (count < Capacity) count++;
        }

        public bool TryGetAtTime(double time, out T value)
        {
            value = default;
            if (count == 0) return false;

            int start = (head - count + Capacity) % Capacity;
            int bestIdx = start;
            double bestDist = double.MaxValue;

            for (int i = 0; i < count; i++)
            {
                int idx = (start + i) % Capacity;
                double dist = Math.Abs(buffer[idx].timestamp - time);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            value = buffer[bestIdx].value;
            return true;
        }

        public void Clear()
        {
            head = 0;
            count = 0;
        }
    }
}
