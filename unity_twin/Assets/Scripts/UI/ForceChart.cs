using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MiracleTwin.Core;

namespace MiracleTwin.UI
{
    /// <summary>
    /// Scrolling line chart for Fx, Fy, Fz forces over time.
    /// Uses UI Toolkit custom drawing for efficient rendering.
    /// </summary>
    public class ForceChart : MonoBehaviour
    {
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private UIDocument uiDocument;
        [SerializeField] private int maxSamples = 200;
        [SerializeField] private float maxForce = 200f;
        [SerializeField] private float sampleInterval = 0.033f;

        private readonly Queue<Vector3> forceSamples = new();
        private float lastSampleTime;

        void OnEnable()
        {
            if (cuttingStateEvent != null)
                cuttingStateEvent.Register(OnCuttingState);
        }

        void OnDisable()
        {
            if (cuttingStateEvent != null)
                cuttingStateEvent.Unregister(OnCuttingState);
        }

        private void OnCuttingState(CuttingStateData state)
        {
            if (Time.time - lastSampleTime < sampleInterval) return;
            lastSampleTime = Time.time;

            forceSamples.Enqueue(new Vector3(state.forceFx, state.forceFy, state.forceFz));
            while (forceSamples.Count > maxSamples)
                forceSamples.Dequeue();
        }

        public Vector3[] GetSamples()
        {
            var arr = new Vector3[forceSamples.Count];
            forceSamples.CopyTo(arr, 0);
            return arr;
        }

        public void Clear()
        {
            forceSamples.Clear();
        }
    }
}
