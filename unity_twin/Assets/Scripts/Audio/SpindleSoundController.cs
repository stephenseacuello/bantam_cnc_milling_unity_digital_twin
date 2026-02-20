using UnityEngine;
using MiracleTwin.Core;

namespace MiracleTwin.Audio
{
    /// <summary>
    /// Spindle motor sound: continuous tone proportional to RPM.
    /// Pitch shifts with RPM changes (Doppler-like).
    /// Fades in on M3, fades out on M5.
    /// </summary>
    [RequireComponent(typeof(AudioSource))]
    public class SpindleSoundController : MonoBehaviour
    {
        [SerializeField] private MachineStateEventSO machineStateEvent;

        [Header("Settings")]
        [SerializeField] private float baseFrequency = 200f;
        [SerializeField] private float maxFrequency = 2000f;
        [SerializeField] private float baseVolume = 0.1f;
        [SerializeField] private float fadeSpeed = 5f;

        private AudioSource audioSource;
        private float targetFrequency;
        private float currentFrequency;
        private float targetVolume;
        private float currentVol;
        private float phase;
        private float spindleRPM;
        private bool spindleOn;

        void Awake()
        {
            audioSource = GetComponent<AudioSource>();
            audioSource.playOnAwake = false;
            audioSource.loop = true;
            audioSource.spatialBlend = 1f;
        }

        void OnEnable()
        {
            if (machineStateEvent != null)
                machineStateEvent.Register(OnMachineState);
        }

        void OnDisable()
        {
            if (machineStateEvent != null)
                machineStateEvent.Unregister(OnMachineState);
        }

        private void OnMachineState(RosMessageTypes.Miracle.MachineStateMsg msg)
        {
            spindleRPM = (float)msg.spindle_speed;
            spindleOn = msg.status == "RUNNING" || msg.status == "PAUSED";
        }

        void Update()
        {
            targetFrequency = Mathf.Lerp(baseFrequency, maxFrequency,
                Mathf.Clamp01(spindleRPM / 23000f));
            currentFrequency = Mathf.Lerp(currentFrequency, targetFrequency,
                fadeSpeed * Time.deltaTime);

            targetVolume = spindleOn && spindleRPM > 100f ? baseVolume : 0f;
            currentVol = Mathf.Lerp(currentVol, targetVolume, fadeSpeed * Time.deltaTime);
        }

        void OnAudioFilterRead(float[] data, int channels)
        {
            if (currentVol < 0.001f) return;

            float sampleRate = AudioSettings.outputSampleRate;

            for (int i = 0; i < data.Length; i += channels)
            {
                // Simple sine wave at spindle frequency
                float sample = Mathf.Sin(2f * Mathf.PI * currentFrequency * phase) * currentVol;

                // Add harmonics for richer sound
                sample += Mathf.Sin(4f * Mathf.PI * currentFrequency * phase) * currentVol * 0.3f;
                sample += Mathf.Sin(6f * Mathf.PI * currentFrequency * phase) * currentVol * 0.1f;

                for (int ch = 0; ch < channels; ch++)
                    data[i + ch] += sample;

                phase += 1f / sampleRate;
                if (phase > 1f) phase -= 1f;
            }
        }
    }
}
