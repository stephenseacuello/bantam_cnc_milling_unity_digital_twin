using UnityEngine;
using System;
using System.Collections.Generic;
using RosMessageTypes.Miracle;

namespace MiracleTwin.Cutting
{
    // -----------------------------------------------------------------------
    // Spindle Vibration Spectrum Analysis
    // -----------------------------------------------------------------------

    /// <summary>
    /// Frequency-domain representation of a spindle vibration signal.
    /// </summary>
    [Serializable]
    public class VibrationSpectrum
    {
        public float[] frequencies;          // Hz
        public float[] amplitudes;           // mm
        public float[] phases;              // radians
        public float dominantFrequencyHz;
        public float dominantAmplitudeMM;
        public float totalRMS;
        public double timestamp;
    }

    /// <summary>
    /// Result of chatter detection analysis.
    /// </summary>
    [Serializable]
    public class ChatterDetectionResult
    {
        public bool isChatter;
        public float chatterFrequencyHz;
        public float chatterAmplitudeMM;
        public float toothPassingFrequencyHz;
        public float confidence;
    }

    /// <summary>
    /// Composite spindle health report derived from vibration analysis.
    /// </summary>
    [Serializable]
    public class SpindleHealthReport
    {
        public float vibrationRMS;
        public float dominantFrequency;
        public bool chatterDetected;
        public float runoutMM;
        public float bearingHealth;
        public float overallHealth;          // weighted composite 0-1
        public List<string> recommendations;
    }

    /// <summary>
    /// Analyzes spindle vibration signals using DFT to detect chatter,
    /// estimate runout, and assess bearing health.
    /// </summary>
    public class SpindleVibrationAnalyzer
    {
        private readonly float[] sampleBuffer;
        private int writeIndex;
        private int sampleCount;
        public float sampleRateHz;

        /// <summary>Number of valid samples currently in the buffer.</summary>
        public int SampleCount => sampleCount;

        /// <summary>Buffer capacity.</summary>
        public int Capacity => sampleBuffer.Length;

        public SpindleVibrationAnalyzer(int capacity = 4096, float sampleRate = 10000f)
        {
            sampleBuffer = new float[capacity];
            writeIndex = 0;
            sampleCount = 0;
            sampleRateHz = sampleRate;
        }

        /// <summary>
        /// Add a vibration amplitude sample to the circular buffer.
        /// </summary>
        public void AddSample(float amplitude)
        {
            sampleBuffer[writeIndex] = amplitude;
            writeIndex = (writeIndex + 1) % sampleBuffer.Length;
            if (sampleCount < sampleBuffer.Length)
                sampleCount++;
        }

        /// <summary>
        /// Retrieve samples in chronological order from the circular buffer.
        /// </summary>
        private float[] GetOrderedSamples()
        {
            float[] ordered = new float[sampleCount];
            if (sampleCount < sampleBuffer.Length)
            {
                Array.Copy(sampleBuffer, 0, ordered, 0, sampleCount);
            }
            else
            {
                int start = writeIndex; // oldest sample
                int firstChunk = sampleBuffer.Length - start;
                Array.Copy(sampleBuffer, start, ordered, 0, firstChunk);
                Array.Copy(sampleBuffer, 0, ordered, firstChunk, start);
            }
            return ordered;
        }

        /// <summary>
        /// Compute the vibration spectrum using a simplified DFT.
        /// Only computes bins up to Nyquist (N/2).
        /// </summary>
        public VibrationSpectrum ComputeSpectrum()
        {
            if (sampleCount == 0)
            {
                return new VibrationSpectrum
                {
                    frequencies = Array.Empty<float>(),
                    amplitudes = Array.Empty<float>(),
                    phases = Array.Empty<float>(),
                    dominantFrequencyHz = 0f,
                    dominantAmplitudeMM = 0f,
                    totalRMS = 0f,
                    timestamp = Time.timeAsDouble
                };
            }

            float[] samples = GetOrderedSamples();
            int N = samples.Length;
            int numBins = N / 2;

            float[] frequencies = new float[numBins];
            float[] amplitudes = new float[numBins];
            float[] phases = new float[numBins];

            float freqResolution = sampleRateHz / N;

            // DFT: X[k] = sum_{n=0}^{N-1} x[n] * e^{-j*2*pi*k*n/N}
            for (int k = 0; k < numBins; k++)
            {
                float realPart = 0f;
                float imagPart = 0f;

                for (int n = 0; n < N; n++)
                {
                    float angle = 2f * Mathf.PI * k * n / N;
                    realPart += samples[n] * Mathf.Cos(angle);
                    imagPart -= samples[n] * Mathf.Sin(angle);
                }

                // Normalize by N; multiply by 2 for single-sided spectrum (except DC)
                float mag = Mathf.Sqrt(realPart * realPart + imagPart * imagPart) / N;
                if (k > 0) mag *= 2f;

                frequencies[k] = k * freqResolution;
                amplitudes[k] = mag;
                phases[k] = Mathf.Atan2(imagPart, realPart);
            }

            // Find dominant (skip DC bin k=0)
            float domFreq = 0f;
            float domAmp = 0f;
            for (int k = 1; k < numBins; k++)
            {
                if (amplitudes[k] > domAmp)
                {
                    domAmp = amplitudes[k];
                    domFreq = frequencies[k];
                }
            }

            // RMS from time domain
            float sumSq = 0f;
            for (int n = 0; n < N; n++)
                sumSq += samples[n] * samples[n];
            float rms = Mathf.Sqrt(sumSq / N);

            return new VibrationSpectrum
            {
                frequencies = frequencies,
                amplitudes = amplitudes,
                phases = phases,
                dominantFrequencyHz = domFreq,
                dominantAmplitudeMM = domAmp,
                totalRMS = rms,
                timestamp = Time.timeAsDouble
            };
        }

        /// <summary>
        /// Detect chatter by looking for spectral peaks that are NOT at
        /// tooth passing frequency harmonics (1x, 2x, 3x).
        /// </summary>
        public ChatterDetectionResult DetectChatterFrequency(float spindleRPM, int numFlutes)
        {
            var spectrum = ComputeSpectrum();
            float toothPassFreq = spindleRPM * numFlutes / 60f;

            var result = new ChatterDetectionResult
            {
                isChatter = false,
                chatterFrequencyHz = 0f,
                chatterAmplitudeMM = 0f,
                toothPassingFrequencyHz = toothPassFreq,
                confidence = 0f
            };

            if (spectrum.amplitudes == null || spectrum.amplitudes.Length < 2 || toothPassFreq <= 0f)
                return result;

            // Frequency tolerance: +/- 1 bin width
            float freqResolution = spectrum.frequencies.Length > 1
                ? spectrum.frequencies[1] - spectrum.frequencies[0]
                : sampleRateHz;
            float tolerance = freqResolution * 1.5f;

            // Find the largest non-harmonic peak (skip DC)
            float maxNonHarmonicAmp = 0f;
            float maxNonHarmonicFreq = 0f;
            float maxHarmonicAmp = 0f;

            for (int k = 1; k < spectrum.amplitudes.Length; k++)
            {
                float freq = spectrum.frequencies[k];
                float amp = spectrum.amplitudes[k];

                bool isHarmonic = false;
                for (int h = 1; h <= 3; h++)
                {
                    if (Mathf.Abs(freq - h * toothPassFreq) < tolerance)
                    {
                        isHarmonic = true;
                        if (amp > maxHarmonicAmp)
                            maxHarmonicAmp = amp;
                        break;
                    }
                }

                if (!isHarmonic && amp > maxNonHarmonicAmp)
                {
                    maxNonHarmonicAmp = amp;
                    maxNonHarmonicFreq = freq;
                }
            }

            // Chatter if a non-harmonic peak is significant relative to the harmonic energy
            float threshold = maxHarmonicAmp * 0.3f; // 30% of strongest harmonic
            if (maxNonHarmonicAmp > threshold && maxNonHarmonicAmp > 1e-6f)
            {
                result.isChatter = true;
                result.chatterFrequencyHz = maxNonHarmonicFreq;
                result.chatterAmplitudeMM = maxNonHarmonicAmp;
                // Confidence: how much larger the chatter peak is vs threshold
                float ratio = maxHarmonicAmp > 0
                    ? maxNonHarmonicAmp / maxHarmonicAmp
                    : 1f;
                result.confidence = Mathf.Clamp01(ratio);
            }

            return result;
        }

        /// <summary>
        /// Estimate spindle runout (mm) — the once-per-revolution vibration component.
        /// Returns the amplitude at the 1x RPM frequency bin.
        /// </summary>
        public float GetRunoutEstimate(float spindleRPM)
        {
            if (spindleRPM <= 0f || sampleCount == 0) return 0f;

            var spectrum = ComputeSpectrum();
            float oncePerRevHz = spindleRPM / 60f;

            if (spectrum.frequencies == null || spectrum.frequencies.Length < 2)
                return 0f;

            float freqRes = spectrum.frequencies[1] - spectrum.frequencies[0];
            if (freqRes <= 0) return 0f;

            // Find closest bin to 1x RPM
            int bin = Mathf.RoundToInt(oncePerRevHz / freqRes);
            if (bin < 0 || bin >= spectrum.amplitudes.Length)
                return 0f;

            return spectrum.amplitudes[bin];
        }

        /// <summary>
        /// Bearing health indicator: ratio of high-frequency energy (>2 kHz) to total.
        /// Returns 0-1 where 1 = healthy (little high-freq energy), 0 = degraded.
        /// </summary>
        public float GetBearingHealthIndicator()
        {
            if (sampleCount == 0) return 1f;

            var spectrum = ComputeSpectrum();
            if (spectrum.amplitudes == null || spectrum.amplitudes.Length < 2)
                return 1f;

            float freqRes = spectrum.frequencies[1] - spectrum.frequencies[0];
            float totalEnergy = 0f;
            float highFreqEnergy = 0f;
            float highFreqThreshold = 2000f; // Hz

            for (int k = 1; k < spectrum.amplitudes.Length; k++)
            {
                float energy = spectrum.amplitudes[k] * spectrum.amplitudes[k];
                totalEnergy += energy;
                if (spectrum.frequencies[k] > highFreqThreshold)
                    highFreqEnergy += energy;
            }

            if (totalEnergy < 1e-12f) return 1f;

            float highFreqRatio = highFreqEnergy / totalEnergy;
            // Invert: high ratio = bad bearing health
            return Mathf.Clamp01(1f - highFreqRatio);
        }

        /// <summary>
        /// Generate a comprehensive spindle health report.
        /// </summary>
        public SpindleHealthReport GenerateHealthReport(float spindleRPM, int numFlutes)
        {
            var spectrum = ComputeSpectrum();
            var chatterResult = DetectChatterFrequency(spindleRPM, numFlutes);
            float runout = GetRunoutEstimate(spindleRPM);
            float bearingHealth = GetBearingHealthIndicator();

            var recommendations = new List<string>();

            // Weighted composite health: bearing 30%, vibration RMS 30%, chatter 25%, runout 15%
            float rmsScore = Mathf.Clamp01(1f - spectrum.totalRMS / 0.1f);  // 0.1mm RMS = bad
            float chatterScore = chatterResult.isChatter ? (1f - chatterResult.confidence) : 1f;
            float runoutScore = Mathf.Clamp01(1f - runout / 0.05f);         // 0.05mm runout = bad

            float overallHealth = 0.30f * bearingHealth
                                + 0.30f * rmsScore
                                + 0.25f * chatterScore
                                + 0.15f * runoutScore;

            if (chatterResult.isChatter)
                recommendations.Add($"Chatter detected at {chatterResult.chatterFrequencyHz:F0} Hz. Consider adjusting RPM or depth of cut.");

            if (runout > 0.025f)
                recommendations.Add($"Runout {runout:F3} mm exceeds 0.025 mm threshold. Inspect collet and tool holder.");

            if (bearingHealth < 0.7f)
                recommendations.Add("Elevated high-frequency vibration. Schedule bearing inspection.");

            if (spectrum.totalRMS > 0.05f)
                recommendations.Add($"RMS vibration {spectrum.totalRMS:F3} mm is elevated. Check tool condition and workholding.");

            if (recommendations.Count == 0)
                recommendations.Add("Spindle health is within normal operating parameters.");

            return new SpindleHealthReport
            {
                vibrationRMS = spectrum.totalRMS,
                dominantFrequency = spectrum.dominantFrequencyHz,
                chatterDetected = chatterResult.isChatter,
                runoutMM = runout,
                bearingHealth = bearingHealth,
                overallHealth = Mathf.Clamp01(overallHealth),
                recommendations = recommendations
            };
        }
    }

    /// <summary>
    /// Chatter risk levels for machining operations.
    /// </summary>
    public enum ChatterRisk
    {
        LOW,
        MEDIUM,
        HIGH
    }

    /// <summary>
    /// Analytical stability lobe predictor using Altintas zeroth-order approximation (ZOA).
    ///
    /// Computes the stability boundary: ap_lim = -1/(2·Ktc·N·Re[G(jω)])
    /// where G(jω) is the structural transfer function at chatter frequency ω.
    ///
    /// Modal parameters default to 1/4" HSS endmill in ER-11 collet.
    /// </summary>
    public class StabilityLobePredictor : MonoBehaviour
    {
        [Header("Modal Parameters (Tap-Test Calibration)")]
        [SerializeField] private float naturalFrequencyHz = 1800f;
        [SerializeField] private float dampingRatio = 0.03f;
        [SerializeField] private float stiffnessNpm = 8e6f;  // N/m

        [Header("Cutting Parameters")]
        [SerializeField] private float Ktc = 796f;   // N/mm² tangential cutting coefficient
        [SerializeField] private int fluteCount = 2;

        [Header("Risk Thresholds")]
        [Tooltip("Margin below stability limit for MEDIUM risk (fraction of ap_lim)")]
        [SerializeField] private float mediumRiskMargin = 0.8f;
        [Tooltip("Margin below stability limit for HIGH risk (fraction of ap_lim)")]
        [SerializeField] private float highRiskMargin = 0.95f;

        [Header("RPM Search")]
        [SerializeField] private float minRPM = 3000f;
        [SerializeField] private float maxRPM = 25000f;
        [SerializeField] private float rpmSearchStep = 50f;

        [Header("Tool Wear Adjustment")]
        [SerializeField] private float currentWearVB = 0f;           // Current flank wear in mm
        [SerializeField] private float wearStiffnessReduction = 0.15f; // % stiffness loss per 0.1mm wear
        [SerializeField] private float wearDampingIncrease = 0.05f;    // % damping increase per 0.1mm wear
        [SerializeField] private float maxWearVB = 0.3f;              // VBmax clamp (mm)

        // Cached lobe surface for visualization
        private float[,] lobeSurface; // [rpmIndex, depthIndex]
        private float[] lobeSurfaceRPMs;
        private float[] lobeSurfaceDepths;
        private bool lobeSurfaceDirty = true;

        /// <summary>Last evaluated chatter risk.</summary>
        public ChatterRisk LastRisk { get; private set; } = ChatterRisk.LOW;

        /// <summary>Last evaluated stability limit (mm).</summary>
        public float LastStabilityLimit { get; private set; }

        /// <summary>Current flank wear value (mm).</summary>
        public float CurrentWearVB => currentWearVB;

        /// <summary>
        /// Update tool wear and recompute stability limits.
        /// Marks the lobe surface cache as dirty so it will be recomputed on next access.
        /// </summary>
        public void UpdateToolWear(float wearVB_mm)
        {
            currentWearVB = Mathf.Max(0f, wearVB_mm);
            lobeSurfaceDirty = true;
        }

        /// <summary>
        /// Get wear-adjusted modal stiffness: k_eff = k * (1 - wearFactor * VB / 0.1).
        /// Wear reduces effective stiffness (tool becomes more compliant).
        /// Clamped so stiffness never drops below 10% of nominal.
        /// </summary>
        private float GetEffectiveStiffness(float wearVB)
        {
            float clampedVB = Mathf.Clamp(wearVB, 0f, maxWearVB);
            float factor = 1f - wearStiffnessReduction * (clampedVB / 0.1f);
            factor = Mathf.Max(factor, 0.1f); // Never below 10% of nominal
            return stiffnessNpm * factor;
        }

        /// <summary>
        /// Get wear-adjusted damping: zeta_eff = zeta * (1 + wearDampingIncrease * VB / 0.1).
        /// Wear increases effective damping slightly (contact area grows).
        /// </summary>
        private float GetEffectiveDamping(float wearVB)
        {
            float clampedVB = Mathf.Clamp(wearVB, 0f, maxWearVB);
            float factor = 1f + wearDampingIncrease * (clampedVB / 0.1f);
            return dampingRatio * factor;
        }

        /// <summary>
        /// Calculate the stability limit (ap_lim in mm) at a given RPM
        /// using the Altintas-Budak ZOA method, with wear-adjusted modal parameters.
        /// </summary>
        public float CalculateStabilityLimit(float rpm)
        {
            if (rpm <= 0) return float.MaxValue;

            float k_eff = GetEffectiveStiffness(currentWearVB);
            float zeta_eff = GetEffectiveDamping(currentWearVB);
            float omega_n = naturalFrequencyHz * 2f * Mathf.PI;
            float bestApLim = float.MaxValue;

            // Sweep chatter frequencies near natural frequency
            for (float ratio = 0.5f; ratio < 2.0f; ratio += 0.005f)
            {
                float omega_c = omega_n * ratio;
                float r = omega_c / omega_n;
                float r2 = r * r;
                float dr = 2f * zeta_eff * r;

                // Real part of FRF: Re[G(jω)]
                float denom = k_eff * ((1f - r2) * (1f - r2) + dr * dr);
                if (Mathf.Abs(denom) < 1e-12f) continue;
                float realG = (1f - r2) / denom;

                if (realG >= 0) continue; // Only unstable when Re[G] < 0

                // ap_lim (mm) = -1 / (2 * Ktc(N/mm²) * N * Re[G](mm/N))
                float realG_mmN = realG * 1000f;  // Convert m/N to mm/N
                float apLim = -1f / (2f * Ktc * fluteCount * realG_mmN);

                if (apLim > 0 && apLim < bestApLim)
                {
                    // Verify this frequency corresponds to a valid lobe
                    float toothPassFreq = rpm * fluteCount / 60f;
                    if (toothPassFreq > 0)
                    {
                        float phaseAngle = Mathf.Atan2(-dr, 1f - r2);
                        float N_lobe = (omega_c - phaseAngle) / (2f * Mathf.PI * toothPassFreq);
                        if (N_lobe > 0)
                            bestApLim = apLim;
                    }
                }
            }

            return bestApLim < float.MaxValue ? bestApLim : 100f; // 100mm = effectively unlimited
        }

        /// <summary>
        /// Compute full lobe surface grid for visualization.
        /// The surface stores stability limits at each (RPM, depth) grid point.
        /// </summary>
        public void ComputeLobeSurface(float minRPMRange, float maxRPMRange, float minDepth, float maxDepth, int resolution = 50)
        {
            lobeSurface = new float[resolution, resolution];
            lobeSurfaceRPMs = new float[resolution];
            lobeSurfaceDepths = new float[resolution];

            for (int i = 0; i < resolution; i++)
            {
                lobeSurfaceRPMs[i] = Mathf.Lerp(minRPMRange, maxRPMRange, (float)i / (resolution - 1));
                lobeSurfaceDepths[i] = Mathf.Lerp(minDepth, maxDepth, (float)i / (resolution - 1));
            }

            for (int ri = 0; ri < resolution; ri++)
            {
                float apLim = CalculateStabilityLimit(lobeSurfaceRPMs[ri]);
                for (int di = 0; di < resolution; di++)
                {
                    // Store margin: positive = stable, negative = unstable
                    lobeSurface[ri, di] = apLim - lobeSurfaceDepths[di];
                }
            }

            lobeSurfaceDirty = false;
        }

        /// <summary>
        /// Get interpolated stability limit at specific RPM from the cached surface.
        /// Returns the stability limit in mm. Falls back to direct calculation if surface not computed.
        /// </summary>
        public float GetStabilityLimitFromSurface(float rpm)
        {
            if (lobeSurface == null || lobeSurfaceRPMs == null || lobeSurfaceRPMs.Length < 2)
                return CalculateStabilityLimit(rpm);

            // Find bounding RPM indices
            int len = lobeSurfaceRPMs.Length;
            if (rpm <= lobeSurfaceRPMs[0])
                return CalculateStabilityLimit(lobeSurfaceRPMs[0]);
            if (rpm >= lobeSurfaceRPMs[len - 1])
                return CalculateStabilityLimit(lobeSurfaceRPMs[len - 1]);

            // Binary search for interval
            int lo = 0, hi = len - 1;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (lobeSurfaceRPMs[mid] <= rpm) lo = mid;
                else hi = mid;
            }

            // The stability limit at each RPM column is where margin crosses zero.
            // margin = apLim - depth, so apLim = margin + depth. At depth index 0, margin = apLim - minDepth.
            // Simpler: just interpolate the direct calculation values.
            float apLo = CalculateStabilityLimit(lobeSurfaceRPMs[lo]);
            float apHi = CalculateStabilityLimit(lobeSurfaceRPMs[hi]);
            float t = (rpm - lobeSurfaceRPMs[lo]) / (lobeSurfaceRPMs[hi] - lobeSurfaceRPMs[lo]);
            return Mathf.Lerp(apLo, apHi, t);
        }

        /// <summary>
        /// Get the full lobe surface data for visualization.
        /// Returns the surface grid, RPM axis values, and depth axis values.
        /// </summary>
        public (float[,] surface, float[] rpms, float[] depths) GetLobeSurfaceData()
        {
            return (lobeSurface, lobeSurfaceRPMs, lobeSurfaceDepths);
        }

        /// <summary>
        /// Check if operating point is in a stable pocket.
        /// A stable pocket is where the depth of cut is below the stability limit
        /// with at least the medium risk margin of headroom.
        /// </summary>
        public bool IsInStablePocket(float rpm, float depth)
        {
            float apLim = CalculateStabilityLimit(rpm);
            return depth < apLim * mediumRiskMargin;
        }

        /// <summary>
        /// Evaluate chatter risk for a given RPM and depth of cut.
        /// </summary>
        public ChatterRisk EvaluateChatterRisk(float rpm, float depthMM)
        {
            float apLim = CalculateStabilityLimit(rpm);
            LastStabilityLimit = apLim;

            float ratio = depthMM / apLim;

            if (ratio >= highRiskMargin)
            {
                LastRisk = ChatterRisk.HIGH;
            }
            else if (ratio >= mediumRiskMargin)
            {
                LastRisk = ChatterRisk.MEDIUM;
            }
            else
            {
                LastRisk = ChatterRisk.LOW;
            }

            return LastRisk;
        }

        /// <summary>
        /// Recommend a stable RPM by searching for the nearest stable lobe pocket.
        /// Returns the recommended RPM, or the current RPM if already stable.
        /// </summary>
        public float RecommendStableRPM(float currentRPM, float depthMM)
        {
            if (EvaluateChatterRisk(currentRPM, depthMM) == ChatterRisk.LOW)
                return currentRPM;

            float bestRPM = currentRPM;
            float bestMargin = 0f;

            // Search in both directions from current RPM
            for (float rpm = minRPM; rpm <= maxRPM; rpm += rpmSearchStep)
            {
                float apLim = CalculateStabilityLimit(rpm);
                float margin = apLim - depthMM;

                if (margin > bestMargin)
                {
                    bestMargin = margin;
                    bestRPM = rpm;
                }
            }

            // Prefer RPM close to current
            float closestStableRPM = bestRPM;
            float closestDist = Mathf.Abs(bestRPM - currentRPM);

            for (float rpm = minRPM; rpm <= maxRPM; rpm += rpmSearchStep)
            {
                float apLim = CalculateStabilityLimit(rpm);
                if (depthMM < apLim * mediumRiskMargin) // Must be in LOW risk zone
                {
                    float dist = Mathf.Abs(rpm - currentRPM);
                    if (dist < closestDist)
                    {
                        closestDist = dist;
                        closestStableRPM = rpm;
                    }
                }
            }

            return closestStableRPM;
        }

        /// <summary>
        /// Get the chatter risk score as a float (0=safe, 1=at limit).
        /// </summary>
        public float GetChatterRiskScore(float rpm, float depthMM)
        {
            float apLim = CalculateStabilityLimit(rpm);
            if (apLim <= 0) return 1f;
            return Mathf.Clamp01(depthMM / apLim);
        }

        /// <summary>
        /// Build a structured stability recommendation for the given operating point.
        /// Includes risk level, recommended RPM, max stable depth, stability margin,
        /// and a human-readable recommendation string.
        /// </summary>
        public StabilityRecommendationMsg GetStabilityRecommendation(string machineId, float rpm, float depthMM)
        {
            var risk = EvaluateChatterRisk(rpm, depthMM);
            float apLim = LastStabilityLimit;
            float recommendedRPM = RecommendStableRPM(rpm, depthMM);
            float margin = apLim > 0 ? Mathf.Clamp01(1f - depthMM / apLim) : 0f;

            string riskStr = risk.ToString();
            string text;

            switch (risk)
            {
                case ChatterRisk.HIGH:
                    text = $"Chatter risk HIGH. Reduce depth below {apLim:F2}mm or change RPM to {recommendedRPM:F0}. " +
                           $"Current depth {depthMM:F2}mm exceeds {highRiskMargin * 100f:F0}% of stability limit.";
                    break;
                case ChatterRisk.MEDIUM:
                    text = $"Chatter risk MEDIUM. Current depth {depthMM:F2}mm is approaching stability limit {apLim:F2}mm. " +
                           $"Consider reducing depth or shifting RPM to {recommendedRPM:F0}.";
                    break;
                default:
                    text = $"Operating within stable zone. Stability margin: {margin * 100f:F0}%.";
                    break;
            }

            return new StabilityRecommendationMsg(
                machine_id: machineId,
                current_rpm: rpm,
                recommended_rpm: recommendedRPM,
                current_depth: depthMM,
                max_stable_depth: apLim,
                risk_level: riskStr,
                stability_margin: margin,
                recommendation: text
            );
        }
    }
}
