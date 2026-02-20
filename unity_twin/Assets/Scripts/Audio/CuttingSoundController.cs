using UnityEngine;
using MiracleTwin.Core;

namespace MiracleTwin.Audio
{
    /// <summary>
    /// Procedural audio for cutting sounds.
    /// Base frequency = tooth-passing frequency: f_tooth = N * RPM / 60 Hz
    /// Amplitude proportional to cutting power.
    /// Silent during rapids (G0).
    /// </summary>
    [RequireComponent(typeof(AudioSource))]
    public class CuttingSoundController : MonoBehaviour
    {
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;

        [Header("Audio Settings")]
        [SerializeField] private float baseVolume = 0.3f;
        [SerializeField] private float noiseVolume = 0.15f;
        [SerializeField] private float fadeSpeed = 10f;
        [SerializeField] private int fluteCount = 2;

        private AudioSource audioSource;
        private float targetVolume;
        private float currentVolume;
        private float toothFrequency;
        private float power;
        private float phase;
        private bool isCutting;

        private System.Random rng = new(42);

        void Awake()
        {
            audioSource = GetComponent<AudioSource>();
            audioSource.playOnAwake = false;
            audioSource.loop = true;
            audioSource.spatialBlend = 1f;
        }

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
            isCutting = state.isCutting;
            toothFrequency = state.spindleRPM * fluteCount / 60f;
            power = state.powerWatts;
        }

        void Update()
        {
            targetVolume = isCutting ? baseVolume * Mathf.Clamp01(power / 250f) : 0f;
            currentVolume = Mathf.Lerp(currentVolume, targetVolume, fadeSpeed * Time.deltaTime);
        }

        void OnAudioFilterRead(float[] data, int channels)
        {
            if (currentVolume < 0.001f) return;

            float sampleRate = AudioSettings.outputSampleRate;

            for (int i = 0; i < data.Length; i += channels)
            {
                // Tooth-passing tone
                float tone = Mathf.Sin(2f * Mathf.PI * toothFrequency * phase);

                // Filtered noise (cutting broadband)
                float noise = ((float)rng.NextDouble() * 2f - 1f) * noiseVolume;

                // Combined signal
                float sample = (tone * 0.6f + noise * 0.4f) * currentVolume;

                for (int ch = 0; ch < channels; ch++)
                    data[i + ch] += sample;

                phase += 1f / sampleRate;
                if (phase > 1f) phase -= 1f;
            }
        }
    }
}
