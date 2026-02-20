using UnityEngine;

namespace MiracleTwin.Cutting
{
    /// <summary>
    /// Material property database for cutting simulation.
    /// Contains mechanical and thermal properties for workpiece and tool materials.
    /// </summary>
    public static class MaterialDatabase
    {
        /// <summary>Material property set.</summary>
        public struct MaterialProperties
        {
            public string name;
            public float density;               // kg/m³
            public float youngsModulus;          // GPa
            public float poissonsRatio;
            public float yieldStrength;          // MPa
            public float ultimateTensileStrength;// MPa
            public float hardnessBrinell;        // HB
            public float thermalConductivity;    // W/m·K
            public float specificHeat;           // J/kg·K
            public float thermalExpansion;       // µm/m·°C
            public float meltingPoint;           // °C
        }

        /// <summary>Cutting coefficient set for a material/tool combination.</summary>
        public struct CuttingCoefficients
        {
            public string workpieceMaterial;
            public string toolMaterial;
            public float Ktc, Krc, Kac;  // Shearing coefficients (N/mm²)
            public float Kte, Kre, Kae;  // Edge coefficients (N/mm)
            public float Kc1_1;          // Kienzle specific cutting force (N/mm²)
            public float mc;             // Kienzle exponent
        }

        // ========== Workpiece Materials ==========

        public static readonly MaterialProperties Al6061T6 = new()
        {
            name = "6061-T6 Aluminum",
            density = 2700f,
            youngsModulus = 68.9f,
            poissonsRatio = 0.33f,
            yieldStrength = 276f,
            ultimateTensileStrength = 310f,
            hardnessBrinell = 95f,
            thermalConductivity = 167f,
            specificHeat = 896f,
            thermalExpansion = 23.6f,
            meltingPoint = 582f
        };

        public static readonly MaterialProperties Ti6Al4V = new()
        {
            name = "Ti-6Al-4V",
            density = 4430f,
            youngsModulus = 113.8f,
            poissonsRatio = 0.342f,
            yieldStrength = 880f,
            ultimateTensileStrength = 950f,
            hardnessBrinell = 334f,
            thermalConductivity = 6.7f,
            specificHeat = 526f,
            thermalExpansion = 8.6f,
            meltingPoint = 1660f
        };

        // ========== Tool Materials ==========

        public static readonly MaterialProperties HSS = new()
        {
            name = "High Speed Steel (M2)",
            density = 8100f,
            youngsModulus = 210f,
            poissonsRatio = 0.30f,
            yieldStrength = 2000f,
            ultimateTensileStrength = 2500f,
            hardnessBrinell = 800f,
            thermalConductivity = 24f,
            specificHeat = 460f,
            thermalExpansion = 11f,
            meltingPoint = 1430f
        };

        public static readonly MaterialProperties Carbide = new()
        {
            name = "Tungsten Carbide (WC-Co)",
            density = 14500f,
            youngsModulus = 620f,
            poissonsRatio = 0.22f,
            yieldStrength = 4000f,
            ultimateTensileStrength = 1500f,
            hardnessBrinell = 1600f,
            thermalConductivity = 84f,
            specificHeat = 292f,
            thermalExpansion = 5.2f,
            meltingPoint = 2870f
        };

        // ========== Cutting Coefficients ==========

        /// <summary>6061-T6 Al with HSS end mill (calibrated).</summary>
        public static readonly CuttingCoefficients Al6061_HSS = new()
        {
            workpieceMaterial = "6061-T6",
            toolMaterial = "HSS",
            Ktc = 796f, Krc = 168f, Kac = 80f,
            Kte = 14.5f, Kre = 10.2f, Kae = 4.8f,
            Kc1_1 = 650f, mc = 0.23f
        };

        /// <summary>6061-T6 Al with Carbide end mill.</summary>
        public static readonly CuttingCoefficients Al6061_Carbide = new()
        {
            workpieceMaterial = "6061-T6",
            toolMaterial = "Carbide",
            Ktc = 680f, Krc = 145f, Kac = 70f,
            Kte = 12.0f, Kre = 8.5f, Kae = 3.5f,
            Kc1_1 = 600f, mc = 0.25f
        };

        /// <summary>Ti-6Al-4V with Carbide end mill.</summary>
        public static readonly CuttingCoefficients Ti6Al4V_Carbide = new()
        {
            workpieceMaterial = "Ti-6Al-4V",
            toolMaterial = "Carbide",
            Ktc = 2000f, Krc = 700f, Kac = 300f,
            Kte = 35f, Kre = 25f, Kae = 12f,
            Kc1_1 = 1400f, mc = 0.25f
        };
    }
}
