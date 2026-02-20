using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Thermal model for CNC milling using:
    /// 1. Stephenson-Agapiou empirical interface temperature
    /// 2. Loewen-Shaw heat partition (chip/tool/workpiece)
    /// 3. Lumped-body ODE for tool temperature evolution
    ///
    /// Calibrated for 6061-T6 Al + HSS tooling.
    /// Reference: Cutting_Physics_Digital_Twin_Reference.pdf Section 3
    /// </summary>
    public class ThermalModel
    {
        // Stephenson-Agapiou empirical constant (calibrated)
        private const float C_thermal = 8.5f;

        // Loewen-Shaw heat partition fractions (Al/HSS)
        private const float ChipFraction = 0.80f;      // Most heat exits with chip
        private const float ToolFraction = 0.08f;       // Small fraction heats tool
        private const float WorkpieceFraction = 0.12f;  // Remainder heats workpiece

        // Material thermal properties — 6061-T6 Aluminum
        private const float k_wp = 167f;       // Thermal conductivity (W/m·K)
        private const float rho_wp = 2700f;    // Density (kg/m³)
        private const float cp_wp = 896f;      // Specific heat (J/kg·K)

        // Material thermal properties — HSS Tool
        private const float k_tool = 24f;      // Thermal conductivity (W/m·K)
        private const float rho_tool = 8100f;  // Density (kg/m³)
        private const float cp_tool = 460f;    // Specific heat (J/kg·K)

        // Convection and geometry — sized for a 1/4" (6.35mm) HSS endmill, ~50mm flute
        private const float h_conv = 50f;      // Convective coefficient (W/m²·K) natural + chip
        private const float A_tool = 0.003f;   // Tool surface area (m²) — shank + flutes
        private const float V_tool = 1.5e-6f;  // Tool volume (m³) — π·r²·L ≈ 1.58e-6
        private const float T_ambient = 20f;   // Ambient temperature (°C)

        // State
        public float ToolTemperature { get; private set; } = T_ambient;
        public float InterfaceTemperature { get; private set; } = T_ambient;
        public float WorkpieceTemperature { get; private set; } = T_ambient;
        public float ChipTemperature { get; private set; } = T_ambient;
        public float HeatGenerationRate { get; private set; }

        // Workpiece thermal tracking (simplified lumped model)
        private float wpThermalMass;
        private const float wp_volume = 0.0762f * 0.0508f * 0.0762f; // m³

        public ThermalModel()
        {
            wpThermalMass = rho_wp * cp_wp * wp_volume;
        }

        /// <summary>
        /// Update thermal state for one time step.
        /// </summary>
        /// <param name="V_mpm">Cutting speed (m/min)</param>
        /// <param name="fz_mm">Feed per tooth (mm)</param>
        /// <param name="ap_mm">Axial depth of cut (mm)</param>
        /// <param name="power_W">Cutting power (W)</param>
        /// <param name="dt">Time step (seconds)</param>
        public void Update(float V_mpm, float fz_mm, float ap_mm, float power_W, float dt)
        {
            if (dt <= 0 || power_W <= 0)
            {
                // Cooldown when not cutting
                Cooldown(dt);
                return;
            }

            HeatGenerationRate = power_W;

            // 1. Interface temperature (Stephenson-Agapiou empirical)
            // T_interface = T_amb + C * V^0.5 * f^0.35 * ap^0.15
            InterfaceTemperature = T_ambient +
                C_thermal *
                Mathf.Pow(Mathf.Max(V_mpm, 0.1f), 0.5f) *
                Mathf.Pow(Mathf.Max(fz_mm, 0.001f), 0.35f) *
                Mathf.Pow(Mathf.Max(ap_mm, 0.01f), 0.15f);

            // 2. Heat partition
            float Q_chip = power_W * ChipFraction;
            float Q_tool = power_W * ToolFraction;
            float Q_wp = power_W * WorkpieceFraction;

            // 3. Tool temperature ODE (lumped body)
            // ρ·c·V · dT/dt = Q_in - h·A·(T - T_amb)
            float thermalMass_tool = rho_tool * cp_tool * V_tool;
            float Q_convection = h_conv * A_tool * (ToolTemperature - T_ambient);
            float dTdt_tool = (Q_tool - Q_convection) / thermalMass_tool;
            ToolTemperature += dTdt_tool * dt;
            ToolTemperature = Mathf.Clamp(ToolTemperature, T_ambient, 600f);

            // 4. Workpiece temperature (simplified)
            float Q_wp_conv = h_conv * 0.01f * (WorkpieceTemperature - T_ambient); // larger surface
            float dTdt_wp = (Q_wp - Q_wp_conv) / wpThermalMass;
            WorkpieceTemperature += dTdt_wp * dt;
            WorkpieceTemperature = Mathf.Clamp(WorkpieceTemperature, T_ambient, 300f);

            // 5. Chip temperature (instantaneous, exits immediately)
            ChipTemperature = T_ambient + Q_chip / (rho_wp * cp_wp * 1e-8f); // small chip volume
            ChipTemperature = Mathf.Min(ChipTemperature, InterfaceTemperature * 0.9f);
        }

        /// <summary>Passive cooldown when not cutting.</summary>
        public void Cooldown(float dt)
        {
            if (dt <= 0) return;

            HeatGenerationRate = 0;
            InterfaceTemperature = T_ambient;
            ChipTemperature = T_ambient;

            // Tool cools via convection
            float thermalMass_tool = rho_tool * cp_tool * V_tool;
            float Q_conv = h_conv * A_tool * (ToolTemperature - T_ambient);
            ToolTemperature -= (Q_conv / thermalMass_tool) * dt;
            ToolTemperature = Mathf.Max(ToolTemperature, T_ambient);

            // Workpiece cools
            float Q_wp_conv = h_conv * 0.01f * (WorkpieceTemperature - T_ambient);
            WorkpieceTemperature -= (Q_wp_conv / wpThermalMass) * dt;
            WorkpieceTemperature = Mathf.Max(WorkpieceTemperature, T_ambient);
        }

        /// <summary>Reset all temperatures to ambient.</summary>
        public void Reset()
        {
            ToolTemperature = T_ambient;
            InterfaceTemperature = T_ambient;
            WorkpieceTemperature = T_ambient;
            ChipTemperature = T_ambient;
            HeatGenerationRate = 0;
        }
    }
}
