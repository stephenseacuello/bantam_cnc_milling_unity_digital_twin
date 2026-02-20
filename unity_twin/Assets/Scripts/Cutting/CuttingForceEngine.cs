using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Full Altintas mechanistic cutting force model with Burst compilation.
    ///
    /// Model: dFt = (Ktc·h + Kte)·dz, dFr = (Krc·h + Kre)·dz, dFa = (Kac·h + Kae)·dz
    /// where h = fz·sin(φ) is the uncut chip thickness.
    ///
    /// Includes:
    /// - Helical flute discretization (axial elements)
    /// - Lag angle from helix
    /// - Wear-adjusted edge coefficients: Kte_w = Kte·(1 + kwear·VB)
    /// - Altintas coordinate transform (tool → machine frame)
    ///
    /// Calibrated for 6061-T6 Al + 1/4" 2-flute HSS end mill.
    /// Reference: Cutting_Physics_Digital_Twin_Reference.pdf
    /// </summary>
    public class CuttingForceEngine : MonoBehaviour
    {
        [Header("Tool Geometry")]
        [SerializeField] private float toolDiameter = 6.35f;    // mm (1/4")
        [SerializeField] private int fluteCount = 2;
        [SerializeField] private float helixAngle = 30f;         // degrees
        [SerializeField] private float rakeAngle = 10f;          // degrees

        [Header("Cutting Coefficients (6061-T6 + HSS)")]
        [SerializeField] private float Ktc = 796f;   // N/mm² (tangential shearing)
        [SerializeField] private float Krc = 168f;   // N/mm² (radial shearing)
        [SerializeField] private float Kac = 80f;    // N/mm² (axial shearing)
        [SerializeField] private float Kte = 14.5f;  // N/mm  (tangential edge)
        [SerializeField] private float Kre = 10.2f;  // N/mm  (radial edge)
        [SerializeField] private float Kae = 4.8f;   // N/mm  (axial edge)

        [Header("Wear Coupling")]
        [SerializeField] private float wearForceMultiplier = 12.5f; // mm⁻¹

        // Output properties
        public Vector3 PeakForce { get; private set; }
        public Vector3 AverageForce { get; private set; }
        public float PowerWatts { get; private set; }
        public float TorqueNm { get; private set; }
        public float MRR { get; private set; }
        public float SpecificCuttingEnergy { get; private set; }

        private NativeArray<float> outputBuffer;

        void Awake()
        {
            outputBuffer = new NativeArray<float>(12, Allocator.Persistent);
        }

        /// <summary>
        /// Calculate cutting forces for current parameters.
        /// Call each FixedUpdate when cutting is active.
        /// </summary>
        public void Calculate(float spindleRPM, float feedPerTooth, float axialDepth,
                             float radialDepth, float flankWearVB = 0f)
        {
            var job = new CuttingForceJob
            {
                spindleRPM = spindleRPM,
                feedPerTooth = feedPerTooth,
                axialDepth = axialDepth,
                radialDepth = radialDepth,
                toolDiameter = toolDiameter,
                fluteCount = fluteCount,
                helixAngle = helixAngle * Mathf.Deg2Rad,
                Ktc = Ktc, Krc = Krc, Kac = Kac,
                Kte = Kte, Kre = Kre, Kae = Kae,
                flankWearVB = flankWearVB,
                wearForceMultiplier = wearForceMultiplier,
                output = outputBuffer
            };

            job.Schedule().Complete();

            PeakForce = new Vector3(outputBuffer[0], outputBuffer[1], outputBuffer[2]);
            AverageForce = new Vector3(outputBuffer[3], outputBuffer[4], outputBuffer[5]);
            PowerWatts = outputBuffer[6];
            TorqueNm = outputBuffer[7];
            MRR = outputBuffer[8];
            SpecificCuttingEnergy = outputBuffer[9];
        }

        /// <summary>Quick estimate using Kienzle model (backup).</summary>
        public float KienzleForceEstimate(float h_mm, float ap_mm, float ae_mm)
        {
            // Kc = Kc1.1 * h^(-mc) for 6061-T6
            float Kc1_1 = 650f;  // N/mm²
            float mc = 0.23f;
            float Kc = Kc1_1 * Mathf.Pow(Mathf.Max(h_mm, 0.001f), -mc);
            return Kc * ap_mm * ae_mm;
        }

        void OnDestroy()
        {
            if (outputBuffer.IsCreated)
                outputBuffer.Dispose();
        }
    }

    /// <summary>
    /// Burst-compiled cutting force calculation job.
    /// Implements the full Altintas mechanistic model with:
    /// - Axial discretization (0.1mm elements)
    /// - Helical lag angle compensation
    /// - Engagement angle boundaries
    /// - Wear-adjusted edge coefficients
    /// </summary>
    [BurstCompile]
    public struct CuttingForceJob : IJob
    {
        // Inputs
        public float spindleRPM;
        public float feedPerTooth;
        public float axialDepth;
        public float radialDepth;
        public float toolDiameter;
        public int fluteCount;
        public float helixAngle;
        public float Ktc, Krc, Kac;
        public float Kte, Kre, Kae;
        public float flankWearVB;
        public float wearForceMultiplier;

        // Output: [Fx_peak, Fy_peak, Fz_peak, Fx_avg, Fy_avg, Fz_avg, Power, Torque, MRR, Kc_specific, 0, 0]
        public NativeArray<float> output;

        public void Execute()
        {
            if (spindleRPM <= 0 || feedPerTooth <= 0 || axialDepth <= 0 || radialDepth <= 0)
            {
                for (int i = 0; i < output.Length; i++) output[i] = 0;
                return;
            }

            float R = toolDiameter / 2f;
            int Ndisk = math.max(1, (int)(axialDepth / 0.1f));
            float dz = axialDepth / Ndisk;

            // Engagement boundaries (conventional milling)
            float ratioAeD = math.clamp(radialDepth / toolDiameter, 0f, 1f);
            float phiStart = math.acos(1f - 2f * ratioAeD);
            float phiExit = math.PI;

            // Wear-adjusted edge coefficients
            float wearFactor = 1f + wearForceMultiplier * flankWearVB;
            float Kte_w = Kte * wearFactor;
            float Kre_w = Kre * wearFactor;
            float Kae_w = Kae * wearFactor;

            float Fx_sum = 0, Fy_sum = 0, Fz_sum = 0;
            float Fx_peak = 0, Fy_peak = 0, Fz_peak = 0;
            int angleSteps = 360;
            float twoPi = 2f * math.PI;

            for (int a = 0; a < angleSteps; a++)
            {
                float phi = a * twoPi / angleSteps;
                float Fx = 0, Fy = 0, Fz = 0;

                for (int j = 0; j < fluteCount; j++)
                {
                    for (int k = 0; k < Ndisk; k++)
                    {
                        float z = k * dz;
                        // Helix lag angle
                        float lagAngle = (2f * math.tan(helixAngle) * z) / toolDiameter;
                        float phiJ = phi - j * (twoPi / fluteCount) - lagAngle;

                        // Normalize to [0, 2π]
                        phiJ = phiJ % twoPi;
                        if (phiJ < 0) phiJ += twoPi;

                        // Check engagement
                        if (phiJ >= phiStart && phiJ <= phiExit)
                        {
                            float h = feedPerTooth * math.sin(phiJ);
                            if (h <= 0) continue;

                            // Differential forces on this element
                            float dFt = (Ktc * h + Kte_w) * dz;
                            float dFr = (Krc * h + Kre_w) * dz;
                            float dFa = (Kac * h + Kae_w) * dz;

                            // Altintas coordinate transform (tool → machine)
                            float cosPhi = math.cos(phiJ);
                            float sinPhi = math.sin(phiJ);
                            Fx += -dFt * cosPhi - dFr * sinPhi;
                            Fy += dFt * sinPhi - dFr * cosPhi;
                            Fz += dFa;
                        }
                    }
                }

                Fx_sum += Fx;
                Fy_sum += Fy;
                Fz_sum += Fz;
                Fx_peak = math.max(Fx_peak, math.abs(Fx));
                Fy_peak = math.max(Fy_peak, math.abs(Fy));
                Fz_peak = math.max(Fz_peak, math.abs(Fz));
            }

            float Fx_avg = Fx_sum / angleSteps;
            float Fy_avg = Fy_sum / angleSteps;
            float Fz_avg = Fz_sum / angleSteps;

            // Cutting speed (m/min)
            float V = math.PI * toolDiameter * spindleRPM / 1000f;

            // Resultant cutting force
            float Fc = math.sqrt(Fx_avg * Fx_avg + Fy_avg * Fy_avg);

            // Power (W): P = Fc * V / 60
            float P = math.abs(Fc) * V / 60f;

            // Torque (Nm): T = P / ω
            float omega = 2f * math.PI * spindleRPM / 60f;
            float T = omega > 0 ? P / omega : 0;

            // Material Removal Rate (mm³/min)
            float mrr = radialDepth * axialDepth * feedPerTooth * fluteCount * spindleRPM;

            // Specific cutting energy (J/mm³)
            float Kc_specific = mrr > 0 ? P * 60f / mrr : 0;

            output[0] = Fx_peak;
            output[1] = Fy_peak;
            output[2] = Fz_peak;
            output[3] = Fx_avg;
            output[4] = Fy_avg;
            output[5] = Fz_avg;
            output[6] = P;
            output[7] = T;
            output[8] = mrr;
            output[9] = Kc_specific;
            output[10] = 0;
            output[11] = 0;
        }
    }
}
