using System;
using System.Collections.Generic;
using UnityEngine;

namespace MiracleTwin.Core
{
    /// <summary>
    /// Generic ScriptableObject-based event channel.
    /// Enables decoupled communication between systems without direct references.
    /// </summary>
    public abstract class GameEventSO<T> : ScriptableObject
    {
        private readonly List<Action<T>> listeners = new();

#if UNITY_EDITOR
        [SerializeField, TextArea(2, 4)] private string description;
        [SerializeField] private bool debugLog;
#endif

        public void Raise(T value)
        {
#if UNITY_EDITOR
            if (debugLog)
                Debug.Log($"[Event] {name} raised with {value}");
#endif
            for (int i = listeners.Count - 1; i >= 0; i--)
            {
                try
                {
                    listeners[i](value);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[Event] Error in listener for {name}: {e}");
                }
            }
        }

        public void Register(Action<T> listener)
        {
            if (!listeners.Contains(listener))
                listeners.Add(listener);
        }

        public void Unregister(Action<T> listener)
        {
            listeners.Remove(listener);
        }

        public int ListenerCount => listeners.Count;

        void OnDisable() => listeners.Clear();
    }

    /// <summary>Void event (no payload) for simple signals.</summary>
    [CreateAssetMenu(menuName = "MIRACLE/Events/Void Event")]
    public class VoidEventSO : ScriptableObject
    {
        private readonly List<Action> listeners = new();

        public void Raise()
        {
            for (int i = listeners.Count - 1; i >= 0; i--)
                listeners[i]();
        }

        public void Register(Action listener) => listeners.Add(listener);
        public void Unregister(Action listener) => listeners.Remove(listener);
        void OnDisable() => listeners.Clear();
    }
}
