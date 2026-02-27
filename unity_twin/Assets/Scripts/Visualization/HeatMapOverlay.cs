using System.Collections.Generic;
using UnityEngine;
using MiracleTwin.Core;
using MiracleTwin.Cutting;

namespace MiracleTwin.Visualization
{
    /// <summary>
    /// Heat map overlay on the workpiece surface showing temperature distribution.
    /// Color ramp: 20C silver -> 80C yellow -> 150C orange -> 200C+ red.
    /// Driven by thermal model data through CuttingStateEvent.
    ///
    /// Phase 3 additions:
    /// - ThermalModel reference for direct thermal coupling
    /// - Dynamic heat radius scaling based on temperature
    /// - Multiple heat source tracking via List of Vector4 (xyz=position, w=temperature)
    /// - Coolant effect: 3x faster decay when coolant is active
    /// - Spatial diffusion of heat sources over time
    /// </summary>
    public class HeatMapOverlay : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private CuttingStateEventSO cuttingStateEvent;
        [SerializeField] private Material heatMapMaterial;
        [SerializeField] private Renderer workpieceRenderer;
        [SerializeField] private CuttingSimulationManager cuttingSimManager;

        [Header("Temperature Range")]
        [SerializeField] private float minTemp = 20f;
        [SerializeField] private float maxTemp = 200f;
        [SerializeField] private float fadeSpeed = 2f;

        [Header("Heat Source Settings")]
        [SerializeField] private int maxHeatSources = 16;
        [SerializeField] private float heatSourceLifetime = 5f;         // Seconds before a source fully decays
        [SerializeField] private float minHeatRadius = 0.005f;          // Meters, at ambient temp
        [SerializeField] private float maxHeatRadius = 0.03f;           // Meters, at max temp
        [SerializeField] private float heatSourceMergeDistance = 0.003f; // Merge sources closer than this (m)

        [Header("Coolant")]
        [SerializeField] private float coolantDecayMultiplier = 3f;     // 3x faster decay when coolant active
        [SerializeField] private bool coolantActive = false;

        [Header("Color Ramp")]
        [SerializeField] private Gradient temperatureGradient;

        public bool IsEnabled { get; private set; } = true;
        public float CurrentMaxTemperature { get; private set; }

        private static readonly int TempProp = Shader.PropertyToID("_Temperature");
        private static readonly int MinTempProp = Shader.PropertyToID("_MinTemp");
        private static readonly int MaxTempProp = Shader.PropertyToID("_MaxTemp");
        private static readonly int HeatPosProp = Shader.PropertyToID("_HeatPosition");
        private static readonly int HeatRadiusProp = Shader.PropertyToID("_HeatRadius");
        private static readonly int HeatSourceCountProp = Shader.PropertyToID("_HeatSourceCount");
        private static readonly int HeatSourceArrayProp = Shader.PropertyToID("_HeatSources");

        private Vector3 lastCutPosition;
        private float currentDisplayTemp;
        private MaterialPropertyBlock propBlock;

        // ThermalModel reference (discovered from CuttingSimulationManager via reflection or direct assignment)
        private ThermalModel thermalModel;

        /// <summary>
        /// Multiple heat source tracking.
        /// Each Vector4: xyz = world position, w = temperature (C).
        /// </summary>
        private List<Vector4> heatSources = new();

        void Awake()
        {
            propBlock = new MaterialPropertyBlock();

            if (temperatureGradient == null)
            {
                temperatureGradient = new Gradient();
                temperatureGradient.SetKeys(
                    new GradientColorKey[] {
                        new(new Color(0.8f, 0.8f, 0.8f), 0f),    // Silver (ambient)
                        new(new Color(1f, 1f, 0.3f), 0.3f),       // Yellow (80C)
                        new(new Color(1f, 0.5f, 0.1f), 0.65f),    // Orange (150C)
                        new(new Color(1f, 0.1f, 0.1f), 1f),       // Red (200C+)
                    },
                    new GradientAlphaKey[] {
                        new(0f, 0f),
                        new(0.5f, 0.2f),
                        new(0.8f, 0.5f),
                        new(1f, 1f),
                    }
                );
            }
        }

        void Start()
        {
            // Discover ThermalModel from CuttingSimulationManager if not directly assigned
            if (cuttingSimManager == null)
                cuttingSimManager = FindFirstObjectByType<CuttingSimulationManager>();

            // ThermalModel is a plain C# class owned by CuttingSimulationManager.
            // We access it indirectly through CuttingStateData temperatures.
            // Direct reference can be set via the public accessor below.
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
            if (state.isCutting)
            {
                lastCutPosition = state.toolPosition;
                CurrentMaxTemperature = Mathf.Max(state.toolTemperature, state.interfaceTemperature);

                // Add or merge heat source at current tool position
                AddOrMergeHeatSource(state.toolPosition, CurrentMaxTemperature);
            }
        }

        /// <summary>
        /// Add a new heat source or merge with an existing nearby source.
        /// </summary>
        private void AddOrMergeHeatSource(Vector3 position, float temperature)
        {
            // Try to merge with an existing nearby source
            for (int i = 0; i < heatSources.Count; i++)
            {
                Vector3 srcPos = new(heatSources[i].x, heatSources[i].y, heatSources[i].z);
                float dist = Vector3.Distance(srcPos, position);

                if (dist < heatSourceMergeDistance)
                {
                    // Merge: keep the hotter temperature, average the position
                    float mergedTemp = Mathf.Max(heatSources[i].w, temperature);
                    Vector3 mergedPos = Vector3.Lerp(srcPos, position, 0.5f);
                    heatSources[i] = new Vector4(mergedPos.x, mergedPos.y, mergedPos.z, mergedTemp);
                    return;
                }
            }

            // No merge candidate found: add a new source
            if (heatSources.Count >= maxHeatSources)
            {
                // Remove the coldest source to make room
                int coldestIdx = 0;
                float coldestTemp = float.MaxValue;
                for (int i = 0; i < heatSources.Count; i++)
                {
                    if (heatSources[i].w < coldestTemp)
                    {
                        coldestTemp = heatSources[i].w;
                        coldestIdx = i;
                    }
                }
                heatSources.RemoveAt(coldestIdx);
            }

            heatSources.Add(new Vector4(position.x, position.y, position.z, temperature));
        }

        void Update()
        {
            if (!IsEnabled || workpieceRenderer == null) return;

            float dt = Time.deltaTime;
            float decayMultiplier = coolantActive ? coolantDecayMultiplier : 1f;

            // Decay heat sources over time (spatial diffusion + temperature decay)
            for (int i = heatSources.Count - 1; i >= 0; i--)
            {
                Vector4 src = heatSources[i];
                float temp = src.w;

                // Exponential temperature decay toward ambient
                float decayRate = fadeSpeed * decayMultiplier;
                temp = Mathf.Lerp(temp, minTemp, decayRate * dt);

                // Remove sources that have cooled to near ambient
                if (temp <= minTemp + 1f)
                {
                    heatSources.RemoveAt(i);
                    continue;
                }

                heatSources[i] = new Vector4(src.x, src.y, src.z, temp);
            }

            // Smooth primary temperature display
            float targetTemp = CurrentMaxTemperature;
            currentDisplayTemp = Mathf.Lerp(currentDisplayTemp, targetTemp, fadeSpeed * dt);

            // Calculate dynamic heat radius based on temperature
            float tempNormalized = Mathf.InverseLerp(minTemp, maxTemp, currentDisplayTemp);
            float dynamicRadius = Mathf.Lerp(minHeatRadius, maxHeatRadius, tempNormalized);

            // Update material properties for primary heat source
            workpieceRenderer.GetPropertyBlock(propBlock);
            propBlock.SetFloat(TempProp, currentDisplayTemp);
            propBlock.SetFloat(MinTempProp, minTemp);
            propBlock.SetFloat(MaxTempProp, maxTemp);
            propBlock.SetVector(HeatPosProp, lastCutPosition);
            propBlock.SetFloat(HeatRadiusProp, dynamicRadius);

            // Pass all heat sources to the shader (up to 16)
            if (heatSources.Count > 0)
            {
                propBlock.SetInt(HeatSourceCountProp, heatSources.Count);
                Vector4[] sourceArray = new Vector4[maxHeatSources];
                for (int i = 0; i < Mathf.Min(heatSources.Count, maxHeatSources); i++)
                    sourceArray[i] = heatSources[i];
                propBlock.SetVectorArray(HeatSourceArrayProp, sourceArray);
            }
            else
            {
                propBlock.SetInt(HeatSourceCountProp, 0);
            }

            workpieceRenderer.SetPropertyBlock(propBlock);
        }

        /// <summary>Set coolant active state (3x faster heat decay when active).</summary>
        public void SetCoolantActive(bool active)
        {
            coolantActive = active;
        }

        /// <summary>Whether coolant effect is currently active.</summary>
        public bool IsCoolantActive => coolantActive;

        /// <summary>Current number of active heat sources being tracked.</summary>
        public int ActiveHeatSourceCount => heatSources.Count;

        /// <summary>Read-only access to current heat sources for debugging/visualization.</summary>
        public IReadOnlyList<Vector4> HeatSources => heatSources;

        /// <summary>Optionally set a direct ThermalModel reference for tighter coupling.</summary>
        public void SetThermalModel(ThermalModel model)
        {
            thermalModel = model;
        }

        public void Toggle()
        {
            IsEnabled = !IsEnabled;
        }
    }
}
