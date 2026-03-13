"""
Cutting Simulation Proxy — Python port of Unity's CuttingForceEngine + ToolWearModel.

Provides force prediction, wear estimation, and RUL calculation for the
PredictionRunner to replace hardcoded values with real simulation results.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from miracle_twin.tool_library import ToolDefinition, ToolLibrary


@dataclass
class CoolantConfig:
    """Coolant/lubrication configuration affecting wear and thermal models."""
    coolant_type: str = 'flood'  # 'dry', 'mist', 'flood', 'high_pressure', 'cryogenic'
    flow_rate_lpm: float = 10.0  # liters per minute
    concentration_pct: float = 8.0  # coolant concentration (for emulsions)

    @property
    def wear_factor(self) -> float:
        """Multiplicative factor on Taylor wear rate. Lower = less wear."""
        factors = {
            'dry': 1.0,
            'mist': 0.85,       # MQL - minimal quantity lubrication
            'flood': 0.65,      # conventional flood coolant
            'high_pressure': 0.50,  # high-pressure through-tool
            'cryogenic': 0.40,  # CO2/LN2 cryogenic
        }
        base = factors.get(self.coolant_type, 1.0)
        # Flow rate effectiveness: diminishing returns above 15 lpm
        if self.coolant_type in ('flood', 'high_pressure'):
            flow_eff = min(1.0, self.flow_rate_lpm / 15.0)
            base *= (1.0 - 0.2 * flow_eff)  # up to 20% additional reduction
        return max(0.3, base)

    @property
    def thermal_factor(self) -> float:
        """Multiplicative factor on temperature rise. Lower = better cooling."""
        factors = {
            'dry': 1.0,
            'mist': 0.75,
            'flood': 0.45,
            'high_pressure': 0.30,
            'cryogenic': 0.15,
        }
        base = factors.get(self.coolant_type, 1.0)
        if self.coolant_type in ('flood', 'high_pressure'):
            flow_eff = min(1.0, self.flow_rate_lpm / 15.0)
            base *= (1.0 - 0.3 * flow_eff)
        return max(0.1, base)

    @property
    def chip_evacuation_factor(self) -> float:
        """Factor for chip recutting probability. Lower = better evacuation."""
        factors = {
            'dry': 1.0,
            'mist': 0.9,
            'flood': 0.5,
            'high_pressure': 0.3,
            'cryogenic': 0.7,  # CO2 doesn't wash chips well
        }
        return factors.get(self.coolant_type, 1.0)


@dataclass
class CuttingCoefficients:
    """Altintas mechanistic cutting coefficients for 6061-T6 + HSS."""
    Ktc: float = 796.0   # N/mm² tangential shearing
    Krc: float = 168.0   # N/mm² radial shearing
    Kac: float = 80.0    # N/mm² axial shearing
    Kte: float = 14.5    # N/mm tangential edge
    Kre: float = 10.2    # N/mm radial edge
    Kae: float = 4.8     # N/mm axial edge
    wear_force_multiplier: float = 12.5  # mm⁻¹


@dataclass
class ToolState:
    """Current tool state."""
    diameter_mm: float = 6.35
    flute_count: int = 2
    helix_angle_deg: float = 30.0
    flank_wear_vb: float = 0.02  # mm
    cutting_time_min: float = 0.0


@dataclass
class GCodeBlock:
    """Simplified G-code block for simulation."""
    feed_rate_mmpm: float = 0.0  # mm/min
    spindle_rpm: float = 0.0
    axial_depth_mm: float = 1.5
    radial_depth_mm: float = 3.175
    length_mm: float = 10.0  # segment length
    # Arc-specific fields (None for linear moves)
    arc_center_i: Optional[float] = None
    arc_center_j: Optional[float] = None
    arc_center_k: Optional[float] = None
    arc_direction: Optional[str] = None  # 'CW' or 'CCW'
    start_x: float = 0.0
    start_y: float = 0.0
    end_x: float = 0.0
    end_y: float = 0.0

    @property
    def is_arc(self) -> bool:
        """Return True if this block represents an arc move."""
        return self.arc_direction is not None


@dataclass
class BlockPrediction:
    """Prediction result for a single G-code block."""
    peak_force_n: float = 0.0
    avg_force_n: float = 0.0
    power_w: float = 0.0
    torque_nm: float = 0.0
    mrr_mm3pm: float = 0.0
    wear_after_block_mm: float = 0.0
    temperature_rise_c: float = 0.0
    deflection_mm: float = 0.0
    dimensional_error_mm: float = 0.0
    surface_ra_um: float = 0.0


@dataclass
class SimulationResult:
    """Full simulation result."""
    block_predictions: List[BlockPrediction] = field(default_factory=list)
    total_cutting_time_min: float = 0.0
    final_wear_mm: float = 0.0
    remaining_useful_life_hours: float = 0.0
    confidence: float = 0.0
    health_index: float = 0.0
    trend_data: List[float] = field(default_factory=list)
    recommended_action: str = ''
    max_deflection_mm: float = 0.0
    max_dimensional_error_mm: float = 0.0
    avg_surface_ra_um: float = 0.0
    max_surface_ra_um: float = 0.0


@dataclass
class WhatIfComparison:
    """Result of a what-if comparison between baseline and override parameters."""
    baseline: Dict[str, float] = field(default_factory=dict)
    override: Dict[str, float] = field(default_factory=dict)
    delta: Dict[str, float] = field(default_factory=dict)


class ToolDeflectionModel:
    """Cantilever beam deflection model for end mills.

    Treats the tool as a cantilever beam fixed at the collet.
    delta = F * L^3 / (3 * E * I)  where I = pi * d^4 / 64 for circular cross-section.
    """

    @staticmethod
    def compute_deflection(
        radial_force_n: float,
        tool_diameter_mm: float,
        overhang_mm: float,
        elastic_modulus_gpa: float,
    ) -> float:
        """Compute tool tip deflection in mm."""
        d = tool_diameter_mm * 1e-3  # to meters
        L = overhang_mm * 1e-3
        E = elastic_modulus_gpa * 1e9  # Pa
        I = math.pi * d ** 4 / 64  # second moment of area
        F = abs(radial_force_n)
        if E * I == 0:
            return 0.0
        delta_m = F * L ** 3 / (3 * E * I)
        return delta_m * 1e3  # back to mm

    @staticmethod
    def compute_dimensional_error(
        radial_force_n: float,
        tool_diameter_mm: float,
        overhang_mm: float,
        elastic_modulus_gpa: float,
    ) -> float:
        """Compute dimensional error on workpiece surface (mm).

        The tool deflects away from the cut, leaving extra material.
        Error = deflection (positive means oversized).
        """
        return ToolDeflectionModel.compute_deflection(
            radial_force_n, tool_diameter_mm, overhang_mm, elastic_modulus_gpa
        )


class SurfaceRoughnessModel:
    """Theoretical and empirical surface roughness prediction.

    Theoretical Ra for end milling (ideal):
        Ra_ideal = f^2 / (32 * R)  where f=feed/tooth, R=nose radius

    With vibration contribution:
        Ra_actual = Ra_ideal + K_vib * amplitude

    With wear contribution:
        Ra_actual += K_wear * VB  (flank wear increases roughness)
    """

    @staticmethod
    def compute_ra(
        feed_per_tooth_mm: float,
        nose_radius_mm: float,
        vibration_amplitude_mm: float = 0.0,
        flank_wear_vb_mm: float = 0.0,
        depth_of_cut_mm: float = 1.0,
    ) -> float:
        """Compute surface roughness Ra in micrometers."""
        if nose_radius_mm <= 0:
            nose_radius_mm = 0.4

        # Theoretical (kinematic) roughness
        f = feed_per_tooth_mm
        R = nose_radius_mm
        Ra_ideal = (f ** 2) / (32.0 * R) * 1000.0  # convert mm to um

        # Vibration contribution (K_vib ~ 5.0 um per mm amplitude)
        Ra_vibration = 5.0 * vibration_amplitude_mm

        # Wear contribution (K_wear ~ 15.0 um per mm VB)
        Ra_wear = 15.0 * flank_wear_vb_mm

        # Depth effect: deeper cuts slightly worsen finish
        depth_factor = 1.0 + 0.05 * max(0, depth_of_cut_mm - 1.0)

        Ra_total = (Ra_ideal + Ra_vibration + Ra_wear) * depth_factor
        return round(max(0.1, Ra_total), 3)  # minimum 0.1 um (polished)


@dataclass
class BlockOptimization:
    """Optimization suggestion for a specific block."""
    block_index: int = 0
    original_feed: float = 0.0
    optimized_feed: float = 0.0
    original_speed: float = 0.0
    optimized_speed: float = 0.0
    reason: str = ''  # e.g. "force_headroom", "chatter_avoidance", "wear_reduction"
    force_change_pct: float = 0.0
    time_change_pct: float = 0.0


@dataclass
class ProgramOptimizationResult:
    """Result of optimizing a complete G-code program."""
    original_cycle_time_min: float = 0.0
    optimized_cycle_time_min: float = 0.0
    time_savings_pct: float = 0.0
    original_max_force_n: float = 0.0
    optimized_max_force_n: float = 0.0
    original_tool_life_min: float = 0.0
    optimized_tool_life_min: float = 0.0
    optimization_actions: list = field(default_factory=list)  # list of BlockOptimization
    risk_assessment: str = 'low'  # "low", "medium", "high"


class CuttingSimProxy:
    """Python proxy for cutting force + wear simulation.

    Ports the Altintas mechanistic force model and 3-stage Taylor wear model
    from the Unity C# implementation to Python for use by PredictionRunner.
    """

    # Shared tool library — loaded once per process
    _tool_library: Optional[ToolLibrary] = None

    # Taylor equation constants (6061-T6 + HSS) — defaults / fallback
    TAYLOR_N = 0.125
    TAYLOR_A = 0.5
    TAYLOR_B = 0.15
    TAYLOR_C = 300.0
    REFERENCE_SPEED = 100.0  # m/min

    # Wear model constants
    VB0 = 0.02       # Initial wear (mm)
    VB1 = 0.08       # Wear at end of break-in (mm)
    T1 = 2.0         # Break-in period (min)
    C2 = 0.004       # Steady-state wear rate (mm/min at V=100)
    VBMAX = 0.30     # End-of-life criterion (mm)

    @classmethod
    def get_tool_library(cls) -> ToolLibrary:
        """Return the shared ToolLibrary, creating it on first access."""
        if cls._tool_library is None:
            cls._tool_library = ToolLibrary()
        return cls._tool_library

    def __init__(self, coefficients: Optional[CuttingCoefficients] = None,
                 coolant: Optional['CoolantConfig'] = None):
        self.coeffs = coefficients or CuttingCoefficients()
        self._active_tool_def: Optional[ToolDefinition] = None
        self._coolant: CoolantConfig = coolant or CoolantConfig()
        self._calibration_log: List[Dict] = []

    @property
    def coolant(self) -> 'CoolantConfig':
        """Return the current coolant configuration."""
        return self._coolant

    def set_coolant(self, config: 'CoolantConfig') -> None:
        """Set coolant/lubrication configuration."""
        self._coolant = config

    # ------------------------------------------------------------------
    # Calibration support
    # ------------------------------------------------------------------

    def get_coefficients(self) -> Dict[str, float]:
        """Return current cutting coefficients as a flat dictionary.

        Returns:
            Dict with keys Ktc, Krc, Kac, Kte, Kre, Kae and their values.
        """
        return {
            'Ktc': self.coeffs.Ktc,
            'Krc': self.coeffs.Krc,
            'Kac': self.coeffs.Kac,
            'Kte': self.coeffs.Kte,
            'Kre': self.coeffs.Kre,
            'Kae': self.coeffs.Kae,
        }

    def scale_force_coefficients(self, factor: float) -> None:
        """Multiply the shearing (force) coefficients Ktc, Krc, Kac by *factor*.

        Records the change in the calibration log.
        """
        before = {'Ktc': self.coeffs.Ktc, 'Krc': self.coeffs.Krc, 'Kac': self.coeffs.Kac}
        self.coeffs.Ktc *= factor
        self.coeffs.Krc *= factor
        self.coeffs.Kac *= factor
        after = {'Ktc': self.coeffs.Ktc, 'Krc': self.coeffs.Krc, 'Kac': self.coeffs.Kac}
        self._calibration_log.append({
            'timestamp': time.time(),
            'action': 'scale_force_coefficients',
            'factor': factor,
            'before': before,
            'after': after,
        })

    def scale_edge_coefficients(self, factor: float) -> None:
        """Multiply the edge (power) coefficients Kte, Kre, Kae by *factor*.

        Records the change in the calibration log.
        """
        before = {'Kte': self.coeffs.Kte, 'Kre': self.coeffs.Kre, 'Kae': self.coeffs.Kae}
        self.coeffs.Kte *= factor
        self.coeffs.Kre *= factor
        self.coeffs.Kae *= factor
        after = {'Kte': self.coeffs.Kte, 'Kre': self.coeffs.Kre, 'Kae': self.coeffs.Kae}
        self._calibration_log.append({
            'timestamp': time.time(),
            'action': 'scale_edge_coefficients',
            'factor': factor,
            'before': before,
            'after': after,
        })

    def scale_thermal_factor(self, factor: float) -> None:
        """Scale the coolant thermal_factor by *factor* (clamped to >= 0.1).

        This is implemented by replacing the CoolantConfig with an adjusted
        copy whose ``thermal_factor`` property is overridden.

        Records the change in the calibration log.
        """
        old_tf = self._coolant.thermal_factor
        # Create a lightweight wrapper that overrides thermal_factor
        new_tf = max(0.1, old_tf * factor)
        original_coolant = self._coolant

        class _AdjustedCoolant(CoolantConfig):
            """CoolantConfig with an externally adjusted thermal_factor."""
            _override_thermal: float = new_tf

            @property
            def thermal_factor(self) -> float:
                return self._override_thermal

        adjusted = _AdjustedCoolant(
            coolant_type=original_coolant.coolant_type,
            flow_rate_lpm=original_coolant.flow_rate_lpm,
            concentration_pct=original_coolant.concentration_pct,
        )
        adjusted._override_thermal = new_tf
        self._coolant = adjusted

        self._calibration_log.append({
            'timestamp': time.time(),
            'action': 'scale_thermal_factor',
            'factor': factor,
            'before': {'thermal_factor': old_tf},
            'after': {'thermal_factor': new_tf},
        })

    def set_tool(self, tool_id: str) -> bool:
        """Load a tool from the library by ID and apply its parameters.

        Updates cutting coefficients, Taylor wear parameters, and geometry.
        Returns True if the tool was found and applied, False otherwise.
        """
        library = self.get_tool_library()
        tool_def = library.get(tool_id)
        if tool_def is None:
            return False
        self.set_tool_definition(tool_def)
        return True

    def set_tool_definition(self, tool_def: ToolDefinition) -> None:
        """Apply a ToolDefinition directly, overriding all coefficients."""
        self._active_tool_def = tool_def

        # Update cutting coefficients
        self.coeffs.Ktc = tool_def.ktc
        self.coeffs.Krc = tool_def.krc
        self.coeffs.Kac = tool_def.kac
        self.coeffs.Kte = tool_def.kte
        self.coeffs.Kre = tool_def.kre
        self.coeffs.Kae = tool_def.kae

        # Update Taylor parameters
        self.TAYLOR_C = tool_def.taylor_C
        self.TAYLOR_N = tool_def.taylor_n
        self.TAYLOR_A = tool_def.taylor_f_exp
        self.TAYLOR_B = tool_def.taylor_ap_exp
        self.VBMAX = tool_def.vb_max_mm

    @property
    def active_tool(self) -> Optional[ToolDefinition]:
        """Return the currently active ToolDefinition, or None."""
        return self._active_tool_def

    def calculate_forces(
        self,
        spindle_rpm: float,
        feed_per_tooth: float,
        axial_depth: float,
        radial_depth: float,
        tool_diameter: float = 6.35,
        flute_count: int = 2,
        helix_angle_rad: float = None,
        flank_wear_vb: float = 0.0,
    ) -> Dict[str, float]:
        """Calculate cutting forces using the full Altintas mechanistic model.

        Returns dict with: fx_peak, fy_peak, fz_peak, fx_avg, fy_avg, fz_avg,
                          power_w, torque_nm, mrr, specific_cutting_energy
        """
        if helix_angle_rad is None:
            helix_angle_rad = math.radians(30.0)

        if spindle_rpm <= 0 or feed_per_tooth <= 0 or axial_depth <= 0 or radial_depth <= 0:
            return {k: 0.0 for k in [
                'fx_peak', 'fy_peak', 'fz_peak',
                'fx_avg', 'fy_avg', 'fz_avg',
                'power_w', 'torque_nm', 'mrr',
                'specific_cutting_energy',
            ]}

        R = tool_diameter / 2.0
        n_disk = max(1, int(axial_depth / 0.1))
        dz = axial_depth / n_disk

        # Engagement boundaries (conventional milling)
        ratio_ae_d = min(max(radial_depth / tool_diameter, 0.0), 1.0)
        phi_start = math.acos(1.0 - 2.0 * ratio_ae_d)
        phi_exit = math.pi

        # Wear-adjusted edge coefficients
        wear_factor = 1.0 + self.coeffs.wear_force_multiplier * flank_wear_vb
        Kte_w = self.coeffs.Kte * wear_factor
        Kre_w = self.coeffs.Kre * wear_factor
        Kae_w = self.coeffs.Kae * wear_factor

        fx_sum = fy_sum = fz_sum = 0.0
        fx_peak = fy_peak = fz_peak = 0.0
        angle_steps = 360
        two_pi = 2.0 * math.pi

        for a in range(angle_steps):
            phi = a * two_pi / angle_steps
            fx = fy = fz = 0.0

            for j in range(flute_count):
                for k in range(n_disk):
                    z = k * dz
                    lag_angle = (2.0 * math.tan(helix_angle_rad) * z) / tool_diameter
                    phi_j = phi - j * (two_pi / flute_count) - lag_angle
                    phi_j = phi_j % two_pi
                    if phi_j < 0:
                        phi_j += two_pi

                    if phi_start <= phi_j <= phi_exit:
                        h = feed_per_tooth * math.sin(phi_j)
                        if h <= 0:
                            continue

                        dFt = (self.coeffs.Ktc * h + Kte_w) * dz
                        dFr = (self.coeffs.Krc * h + Kre_w) * dz
                        dFa = (self.coeffs.Kac * h + Kae_w) * dz

                        cos_phi = math.cos(phi_j)
                        sin_phi = math.sin(phi_j)
                        fx += -dFt * cos_phi - dFr * sin_phi
                        fy += dFt * sin_phi - dFr * cos_phi
                        fz += dFa

            fx_sum += fx
            fy_sum += fy
            fz_sum += fz
            fx_peak = max(fx_peak, abs(fx))
            fy_peak = max(fy_peak, abs(fy))
            fz_peak = max(fz_peak, abs(fz))

        fx_avg = fx_sum / angle_steps
        fy_avg = fy_sum / angle_steps
        fz_avg = fz_sum / angle_steps

        V = math.pi * tool_diameter * spindle_rpm / 1000.0
        Fc = math.sqrt(fx_avg**2 + fy_avg**2)
        power = abs(Fc) * V / 60.0
        omega = 2.0 * math.pi * spindle_rpm / 60.0
        torque = power / omega if omega > 0 else 0.0
        mrr = radial_depth * axial_depth * feed_per_tooth * flute_count * spindle_rpm
        kc_specific = power * 60.0 / mrr if mrr > 0 else 0.0

        return {
            'fx_peak': fx_peak, 'fy_peak': fy_peak, 'fz_peak': fz_peak,
            'fx_avg': fx_avg, 'fy_avg': fy_avg, 'fz_avg': fz_avg,
            'power_w': power, 'torque_nm': torque, 'mrr': mrr,
            'specific_cutting_energy': kc_specific,
        }

    def update_wear(
        self,
        vb_current: float,
        cutting_time_min: float,
        v_mpm: float,
        fz_mm: float,
        ap_mm: float,
        dt_min: float,
        tool_temperature_c: float = 20.0,
    ) -> tuple:
        """Update wear for one time step. Returns (new_vb, new_cutting_time)."""
        if dt_min <= 0 or v_mpm <= 0:
            return vb_current, cutting_time_min

        cutting_time_min += dt_min
        speed_factor = v_mpm / self.REFERENCE_SPEED
        thermal_factor = min(
            3.0, max(1.0, 1.0 + 0.035 * max(0.0, tool_temperature_c - 20.0))
        )

        coolant_wear = self._coolant.wear_factor

        if cutting_time_min < self.T1:
            # Stage 1: Break-in
            vb = self.VB0 + (self.VB1 - self.VB0) * math.sqrt(
                cutting_time_min / self.T1
            ) * coolant_wear
        elif vb_current < 0.25:
            # Stage 2: Steady-state
            rate = self.C2 * speed_factor * thermal_factor
            rate *= (max(fz_mm, 0.01) / 0.05) ** 0.3
            rate *= (max(ap_mm, 0.1) / 1.0) ** 0.1
            rate *= coolant_wear
            vb = vb_current + rate * dt_min
        else:
            # Stage 3: Accelerated
            c3 = 0.1 * speed_factor * thermal_factor
            c3 *= coolant_wear
            vb = vb_current + vb_current * c3 * dt_min

        return min(vb, 0.50), cutting_time_min

    def taylor_life_prediction(
        self, v_mpm: float, fz_mm: float, ap_mm: float
    ) -> float:
        """Predict tool life in minutes using Taylor equation."""
        denom = (
            v_mpm
            * (max(fz_mm, 0.001) ** self.TAYLOR_A)
            * (max(ap_mm, 0.01) ** self.TAYLOR_B)
        )
        if denom <= 0:
            return float('inf')
        return (self.TAYLOR_C / denom) ** (1.0 / self.TAYLOR_N)

    def simulate_arc_block(
        self,
        block: GCodeBlock,
        tool: ToolState,
        vb: float,
        cutting_time: float,
        overrides: Optional[Dict[str, float]] = None,
        n_segments: int = 8,
    ) -> tuple:
        """Simulate an arc block by splitting into linear chord segments.

        Computes varying chip thickness around the arc (thicker on inside of
        arc, thinner on outside) and returns an averaged BlockPrediction.

        Args:
            block: GCodeBlock with arc data populated.
            tool: Current tool state.
            vb: Current flank wear in mm.
            cutting_time: Accumulated cutting time in minutes.
            overrides: Optional parameter overrides.
            n_segments: Number of chord segments for arc approximation.

        Returns:
            Tuple of (BlockPrediction, updated_vb, updated_cutting_time).
        """
        overrides = overrides or {}
        rpm = overrides.get('spindle_rpm', block.spindle_rpm)
        feed = overrides.get('feed_rate', block.feed_rate_mmpm)
        ap = overrides.get('axial_depth', block.axial_depth_mm)
        ae = overrides.get('radial_depth', block.radial_depth_mm)
        helix_rad = math.radians(tool.helix_angle_deg)

        if rpm <= 0 or feed <= 0:
            return BlockPrediction(), vb, cutting_time

        # Compute arc geometry
        cx = block.start_x + (block.arc_center_i or 0.0)
        cy = block.start_y + (block.arc_center_j or 0.0)
        radius = math.sqrt((block.start_x - cx) ** 2 + (block.start_y - cy) ** 2)
        if radius <= 0.0:
            return BlockPrediction(), vb, cutting_time

        # Compute start and end angles
        start_angle = math.atan2(block.start_y - cy, block.start_x - cx)
        end_angle = math.atan2(block.end_y - cy, block.end_x - cx)

        # Compute sweep angle based on direction
        if block.arc_direction == 'CW':
            sweep = start_angle - end_angle
            if sweep <= 0:
                sweep += 2.0 * math.pi
        else:  # CCW
            sweep = end_angle - start_angle
            if sweep <= 0:
                sweep += 2.0 * math.pi

        arc_length = radius * sweep
        seg_length = arc_length / n_segments
        fz_base = feed / (rpm * tool.flute_count)
        v_mpm = math.pi * tool.diameter_mm * rpm / 1000.0

        # Accumulate predictions across segments
        total_peak = 0.0
        total_avg = 0.0
        total_power = 0.0
        total_torque = 0.0
        total_mrr = 0.0
        total_temp = 0.0
        total_time = 0.0

        for i in range(n_segments):
            # Chip thickness varies with arc: inside cuts thicker
            # Scale factor based on curvature relative to tool diameter
            curvature_ratio = tool.diameter_mm / (2.0 * radius)
            if block.arc_direction == 'CW':
                # Climb milling on outside: slightly thinner chips
                thickness_scale = 1.0 - 0.5 * curvature_ratio * math.cos(
                    sweep * (i + 0.5) / n_segments
                )
            else:
                # Conventional: slightly thicker chips on inside
                thickness_scale = 1.0 + 0.5 * curvature_ratio * math.cos(
                    sweep * (i + 0.5) / n_segments
                )
            fz_seg = fz_base * max(0.2, min(2.0, thickness_scale))

            forces = self.calculate_forces(
                spindle_rpm=rpm,
                feed_per_tooth=fz_seg,
                axial_depth=ap,
                radial_depth=ae,
                tool_diameter=tool.diameter_mm,
                flute_count=tool.flute_count,
                helix_angle_rad=helix_rad,
                flank_wear_vb=vb,
            )

            seg_time = seg_length / feed if feed > 0 else 0.0
            total_time += seg_time
            vb, cutting_time = self.update_wear(
                vb, cutting_time, v_mpm, fz_seg, ap, seg_time
            )

            peak = math.sqrt(
                forces['fx_peak'] ** 2 + forces['fy_peak'] ** 2 + forces['fz_peak'] ** 2
            )
            avg = math.sqrt(
                forces['fx_avg'] ** 2 + forces['fy_avg'] ** 2 + forces['fz_avg'] ** 2
            )
            total_peak = max(total_peak, peak)
            total_avg += avg
            total_power += forces['power_w']
            total_torque += forces['torque_nm']
            total_mrr += forces['mrr']
            total_temp += forces['power_w'] * 0.05 * self._coolant.thermal_factor

        pred = BlockPrediction(
            peak_force_n=total_peak,
            avg_force_n=total_avg / n_segments if n_segments > 0 else 0.0,
            power_w=total_power / n_segments if n_segments > 0 else 0.0,
            torque_nm=total_torque / n_segments if n_segments > 0 else 0.0,
            mrr_mm3pm=total_mrr / n_segments if n_segments > 0 else 0.0,
            wear_after_block_mm=vb,
            temperature_rise_c=total_temp / n_segments if n_segments > 0 else 0.0,
        )
        return pred, vb, cutting_time

    def simulate_program(
        self,
        gcode_blocks: List[GCodeBlock],
        tool_state: Optional[ToolState] = None,
        parameter_overrides: Optional[Dict[str, float]] = None,
        tool_id: Optional[str] = None,
        coolant: Optional['CoolantConfig'] = None,
    ) -> SimulationResult:
        """Simulate an entire G-code program and predict forces, wear, and RUL.

        Args:
            gcode_blocks: List of G-code blocks to simulate.
            tool_state: Current tool state (wear, cutting time).
            parameter_overrides: Optional dict to override spindle_rpm, feed_rate, etc.
            tool_id: Optional tool library ID. If provided, the tool's cutting
                coefficients, Taylor parameters, and geometry are applied before
                simulation begins.
            coolant: Optional CoolantConfig to use for this simulation. If
                provided, temporarily overrides the instance coolant setting.

        Returns:
            SimulationResult with per-block predictions and overall RUL.
        """
        # Temporarily apply coolant override if provided
        prev_coolant = self._coolant
        if coolant is not None:
            self._coolant = coolant

        if tool_id is not None:
            self.set_tool(tool_id)

        tool = tool_state or ToolState()
        # If an active tool definition is loaded, sync geometry into ToolState
        if self._active_tool_def is not None:
            tool.diameter_mm = self._active_tool_def.diameter_mm
            tool.flute_count = self._active_tool_def.flute_count
            tool.helix_angle_deg = self._active_tool_def.helix_angle_deg
        overrides = parameter_overrides or {}

        vb = tool.flank_wear_vb
        cutting_time = tool.cutting_time_min
        helix_rad = math.radians(tool.helix_angle_deg)

        # Deflection / surface properties from active tool definition
        elastic_modulus = 200.0  # default HSS
        overhang = 30.0
        nose_radius = 0.4
        if self._active_tool_def is not None:
            elastic_modulus = self._active_tool_def.elastic_modulus_gpa
            overhang = self._active_tool_def.tool_overhang_mm
            nose_radius = self._active_tool_def.nose_radius_mm

        block_predictions = []
        health_trend = []
        total_time = 0.0

        for block in gcode_blocks:
            # Handle arc blocks via dedicated arc simulation
            if block.is_arc:
                pred, vb, cutting_time = self.simulate_arc_block(
                    block, tool, vb, cutting_time, overrides
                )
                block_time = block.length_mm / max(
                    overrides.get('feed_rate', block.feed_rate_mmpm), 1e-9
                )
                total_time += block_time
                block_predictions.append(pred)
                health_trend.append(max(0.0, 1.0 - vb / self.VBMAX))
                continue

            rpm = overrides.get('spindle_rpm', block.spindle_rpm)
            feed = overrides.get('feed_rate', block.feed_rate_mmpm)
            ap = overrides.get('axial_depth', block.axial_depth_mm)
            ae = overrides.get('radial_depth', block.radial_depth_mm)

            if rpm <= 0 or feed <= 0:
                block_predictions.append(BlockPrediction())
                continue

            fz = feed / (rpm * tool.flute_count)
            v_mpm = math.pi * tool.diameter_mm * rpm / 1000.0

            # Calculate forces
            forces = self.calculate_forces(
                spindle_rpm=rpm,
                feed_per_tooth=fz,
                axial_depth=ap,
                radial_depth=ae,
                tool_diameter=tool.diameter_mm,
                flute_count=tool.flute_count,
                helix_angle_rad=helix_rad,
                flank_wear_vb=vb,
            )

            # Time for this block
            block_time_min = block.length_mm / feed if feed > 0 else 0.0
            total_time += block_time_min

            # Update wear
            vb, cutting_time = self.update_wear(
                vb, cutting_time, v_mpm, fz, ap, block_time_min
            )

            # Thermal estimate modulated by coolant
            temp_rise = forces['power_w'] * 0.05 * self._coolant.thermal_factor

            # Chip recutting force penalty for poor chip evacuation
            chip_penalty = 1.0
            if self._coolant.chip_evacuation_factor > 0.7:
                # Up to 10% force increase when evacuation is poor
                chip_penalty = 1.0 + 0.1 * (
                    (self._coolant.chip_evacuation_factor - 0.7) / 0.3
                )

            peak_force = math.sqrt(
                forces['fx_peak'] ** 2
                + forces['fy_peak'] ** 2
                + forces['fz_peak'] ** 2
            ) * chip_penalty
            avg_force = math.sqrt(
                forces['fx_avg'] ** 2
                + forces['fy_avg'] ** 2
                + forces['fz_avg'] ** 2
            ) * chip_penalty

            # Tool deflection from radial force component
            radial_force = abs(forces['fy_peak']) * chip_penalty
            block_deflection = ToolDeflectionModel.compute_deflection(
                radial_force, tool.diameter_mm, overhang, elastic_modulus,
            )
            block_dim_error = ToolDeflectionModel.compute_dimensional_error(
                radial_force, tool.diameter_mm, overhang, elastic_modulus,
            )

            # Surface roughness prediction
            block_ra = SurfaceRoughnessModel.compute_ra(
                feed_per_tooth_mm=fz,
                nose_radius_mm=nose_radius,
                vibration_amplitude_mm=0.0,
                flank_wear_vb_mm=vb,
                depth_of_cut_mm=ap,
            )

            pred = BlockPrediction(
                peak_force_n=peak_force,
                avg_force_n=avg_force,
                power_w=forces['power_w'],
                torque_nm=forces['torque_nm'],
                mrr_mm3pm=forces['mrr'],
                wear_after_block_mm=vb,
                temperature_rise_c=temp_rise,
                deflection_mm=block_deflection,
                dimensional_error_mm=block_dim_error,
                surface_ra_um=block_ra,
            )
            block_predictions.append(pred)
            health_trend.append(max(0.0, 1.0 - vb / self.VBMAX))

        # Calculate RUL
        if len(gcode_blocks) > 0 and total_time > 0:
            last_block = gcode_blocks[-1]
            rpm = overrides.get('spindle_rpm', last_block.spindle_rpm)
            feed = overrides.get('feed_rate', last_block.feed_rate_mmpm)
            ap = overrides.get('axial_depth', last_block.axial_depth_mm)

            if rpm > 0 and feed > 0:
                fz = feed / (rpm * tool.flute_count)
                v_mpm = math.pi * tool.diameter_mm * rpm / 1000.0
                taylor_life = self.taylor_life_prediction(v_mpm, fz, ap)
                remaining_life_min = max(0.0, taylor_life - cutting_time)
                rul_hours = remaining_life_min / 60.0
            else:
                rul_hours = 1000.0
        else:
            rul_hours = 1000.0

        health_index = max(0.0, 1.0 - vb / self.VBMAX)

        # Confidence based on data quality
        confidence = min(0.95, 0.6 + 0.01 * len(gcode_blocks))

        # Recommended action
        if vb >= 0.25:
            action = 'Replace tool immediately'
        elif vb >= 0.15:
            action = 'Schedule tool replacement soon'
        elif vb >= 0.10:
            action = 'Monitor tool wear closely'
        elif rul_hours < 24:
            action = 'Schedule maintenance within 24 hours'
        elif rul_hours < 168:
            action = 'Plan maintenance within one week'
        else:
            action = 'Continue normal operation'

        # Deflection and surface roughness summaries
        deflections = [bp.deflection_mm for bp in block_predictions]
        dim_errors = [bp.dimensional_error_mm for bp in block_predictions]
        surface_ras = [bp.surface_ra_um for bp in block_predictions if bp.surface_ra_um > 0]
        max_defl = max(deflections) if deflections else 0.0
        max_dim_err = max(dim_errors) if dim_errors else 0.0
        avg_ra = (sum(surface_ras) / len(surface_ras)) if surface_ras else 0.0
        max_ra = max(surface_ras) if surface_ras else 0.0

        # Restore previous coolant setting
        self._coolant = prev_coolant

        return SimulationResult(
            block_predictions=block_predictions,
            total_cutting_time_min=total_time,
            final_wear_mm=vb,
            remaining_useful_life_hours=rul_hours,
            confidence=confidence,
            health_index=health_index,
            trend_data=health_trend[-10:] if health_trend else [health_index],
            recommended_action=action,
            max_deflection_mm=max_defl,
            max_dimensional_error_mm=max_dim_err,
            avg_surface_ra_um=round(avg_ra, 3),
            max_surface_ra_um=max_ra,
        )

    def what_if_compare(
        self,
        blocks: List[GCodeBlock],
        tool_state: Optional[ToolState] = None,
        baseline_params: Optional[Dict[str, float]] = None,
        override_params: Optional[Dict[str, float]] = None,
        baseline_coolant: Optional['CoolantConfig'] = None,
        override_coolant: Optional['CoolantConfig'] = None,
    ) -> WhatIfComparison:
        """Run simulation with baseline and override parameters, return comparison.

        Computes both scenarios using the full mechanistic force model and Taylor
        wear equations, then returns a structured comparison of key metrics.

        Args:
            blocks: G-code blocks to simulate.
            tool_state: Current tool state (wear, cutting time).
            baseline_params: Parameter overrides for baseline (None = programmed).
            override_params: Parameter overrides for the what-if scenario.
            baseline_coolant: CoolantConfig for baseline scenario.
            override_coolant: CoolantConfig for what-if scenario.

        Returns:
            WhatIfComparison with baseline, override, and delta dicts containing:
                peak_force, rul_minutes, max_ra, chatter_risk
        """
        baseline_result = self.simulate_program(
            blocks, tool_state=tool_state, parameter_overrides=baseline_params,
            coolant=baseline_coolant,
        )
        override_result = self.simulate_program(
            blocks, tool_state=tool_state, parameter_overrides=override_params,
            coolant=override_coolant,
        )

        # Extract peak force across all blocks
        baseline_peak = max(
            (bp.peak_force_n for bp in baseline_result.block_predictions),
            default=0.0,
        )
        override_peak = max(
            (bp.peak_force_n for bp in override_result.block_predictions),
            default=0.0,
        )

        # RUL in minutes
        baseline_rul_min = baseline_result.remaining_useful_life_hours * 60.0
        override_rul_min = override_result.remaining_useful_life_hours * 60.0

        # Use simulation-computed surface Ra (from SurfaceRoughnessModel)
        baseline_ra = baseline_result.max_surface_ra_um
        override_ra = override_result.max_surface_ra_um
        # Fall back to legacy estimate if simulation yielded zero
        if baseline_ra == 0.0:
            baseline_ra = self._estimate_surface_ra(blocks, baseline_params)
        if override_ra == 0.0:
            override_ra = self._estimate_surface_ra(blocks, override_params)

        # Deflection from simulation results
        baseline_defl = baseline_result.max_deflection_mm
        override_defl = override_result.max_deflection_mm

        # Chatter risk based on spindle RPM relative to stability lobes
        baseline_rpm = self._effective_rpm(blocks, baseline_params)
        override_rpm = self._effective_rpm(blocks, override_params)
        baseline_chatter = self._chatter_risk_score(baseline_rpm)
        override_chatter = self._chatter_risk_score(override_rpm)

        baseline_dict = {
            'peak_force': baseline_peak,
            'rul_minutes': baseline_rul_min,
            'max_ra': baseline_ra,
            'chatter_risk': baseline_chatter,
            'max_deflection_mm': baseline_defl,
        }
        override_dict = {
            'peak_force': override_peak,
            'rul_minutes': override_rul_min,
            'max_ra': override_ra,
            'chatter_risk': override_chatter,
            'max_deflection_mm': override_defl,
        }

        # Delta calculations
        force_pct = (
            (override_peak - baseline_peak) / baseline_peak * 100.0
            if baseline_peak > 0 else 0.0
        )
        life_pct = (
            (override_rul_min - baseline_rul_min) / baseline_rul_min * 100.0
            if baseline_rul_min > 0 else 0.0
        )
        quality_change = (
            -1.0 if override_ra > baseline_ra * 1.05
            else 1.0 if override_ra < baseline_ra * 0.95
            else 0.0
        )
        risk_change = override_chatter - baseline_chatter
        deflection_change_pct = (
            (override_defl - baseline_defl) / baseline_defl * 100.0
            if baseline_defl > 0 else 0.0
        )
        surface_ra_change_pct = (
            (override_ra - baseline_ra) / baseline_ra * 100.0
            if baseline_ra > 0 else 0.0
        )

        delta_dict = {
            'force_pct': force_pct,
            'life_pct': life_pct,
            'quality_change': quality_change,
            'risk_change': risk_change,
            'deflection_change_pct': deflection_change_pct,
            'surface_ra_change_pct': surface_ra_change_pct,
        }

        return WhatIfComparison(
            baseline=baseline_dict,
            override=override_dict,
            delta=delta_dict,
        )

    def _effective_rpm(
        self,
        blocks: List[GCodeBlock],
        overrides: Optional[Dict[str, float]] = None,
    ) -> float:
        """Get effective spindle RPM from overrides or first block."""
        if overrides and 'spindle_rpm' in overrides:
            return overrides['spindle_rpm']
        for block in blocks:
            if block.spindle_rpm > 0:
                return block.spindle_rpm
        return 0.0

    def _estimate_surface_ra(
        self,
        blocks: List[GCodeBlock],
        overrides: Optional[Dict[str, float]] = None,
    ) -> float:
        """Estimate surface roughness Ra (um) from cutting parameters.

        Uses the theoretical finish formula: Ra = f_z^2 / (32 * R)
        where f_z is feed per tooth and R is tool nose radius (approximated
        as tool_diameter / 2 for end mills).
        """
        ovr = overrides or {}
        max_ra = 0.0
        for block in blocks:
            rpm = ovr.get('spindle_rpm', block.spindle_rpm)
            feed = ovr.get('feed_rate', block.feed_rate_mmpm)
            if rpm <= 0 or feed <= 0:
                continue
            fz = feed / (rpm * 2)  # 2-flute default
            R = 6.35 / 2.0  # default tool radius mm
            # Ra in um: (fz^2 / (32 * R)) * 1000
            ra = (fz ** 2 / (32.0 * R)) * 1000.0
            max_ra = max(max_ra, ra)
        return max_ra if max_ra > 0 else 0.8  # fallback

    def _chatter_risk_score(self, rpm: float) -> float:
        """Compute chatter risk score (0.0 = low, 1.0 = high).

        Uses simplified stability lobe boundaries. Known unstable zones
        are near harmonics of the natural frequency (~250 Hz for typical
        CNC spindle-tool assembly).
        """
        if rpm <= 0:
            return 0.0

        # Natural frequency of spindle-tool system (Hz)
        fn = 250.0
        # Check proximity to stability lobe boundaries
        # Unstable RPM = 60 * fn / (k + 1) for integer k
        risk = 0.0
        for k in range(1, 10):
            unstable_rpm = 60.0 * fn / (k + 1)
            ratio = abs(rpm - unstable_rpm) / unstable_rpm
            if ratio < 0.05:
                risk = max(risk, 1.0)
            elif ratio < 0.10:
                risk = max(risk, 0.5)
            elif ratio < 0.15:
                risk = max(risk, 0.2)
        return risk

    def get_block_anomaly_indicators(
        self,
        result: Dict[str, float],
        force_threshold: float = 180.0,
        temp_threshold: float = 25.0,
    ) -> Dict[str, float]:
        """Extract anomaly-relevant metrics from a simulation result dict.

        Args:
            result: A dict with keys matching BlockPrediction fields
                (peak_force_n, temperature_rise_c, wear_after_block_mm,
                surface_ra_um) or a BlockPrediction-like object converted to dict.
            force_threshold: Force threshold in Newtons for ratio calculation.
            temp_threshold: Temperature threshold in Celsius for ratio calculation.

        Returns:
            Dict with anomaly indicator metrics:
                - peak_force_ratio: peak force vs threshold (0-1+)
                - temp_rise_ratio: temperature rise vs threshold (0-1+)
                - wear_rate: wear in mm/block (approximated from wear_after_block_mm)
                - chatter_risk_score: 0-1 score from stability analysis
                - surface_ra_um: surface roughness if available, else 0.0
        """
        peak_force = result.get('peak_force_n', 0.0)
        temp_rise = result.get('temperature_rise_c', 0.0)
        wear = result.get('wear_after_block_mm', 0.0)
        surface_ra = result.get('surface_ra_um', 0.0)

        # Chatter risk from RPM if provided, otherwise 0
        rpm = result.get('spindle_rpm', 0.0)
        chatter = self._chatter_risk_score(rpm)

        return {
            'peak_force_ratio': peak_force / force_threshold if force_threshold > 0 else 0.0,
            'temp_rise_ratio': temp_rise / temp_threshold if temp_threshold > 0 else 0.0,
            'wear_rate': wear,  # mm accumulated through this block
            'chatter_risk_score': chatter,
            'surface_ra_um': surface_ra,
        }

    # ------------------------------------------------------------------
    # Program-level optimization
    # ------------------------------------------------------------------

    def optimize_program(
        self,
        blocks: List[GCodeBlock],
        tool_state: Optional[ToolState] = None,
        constraints: Optional[Dict] = None,
    ) -> 'ProgramOptimizationResult':
        """Analyze a full program and suggest per-block optimizations.

        Strategy:
        1. Simulate program at current parameters
        2. For blocks with force < 60% of threshold: increase feed (more aggressive)
        3. For blocks with force > 85% of threshold: decrease feed (safer)
        4. For blocks near chatter boundary: shift RPM to stable pocket
        5. For blocks with high wear rate: reduce feed to extend tool life
        6. Re-simulate with optimized parameters to verify

        Args:
            blocks: List of GCodeBlock to optimize.
            tool_state: Current tool state.
            constraints: Dict with optional keys:
                - max_feed_increase_pct: max allowed feed increase (default 30%)
                - max_force_pct: maximum force as % of threshold (default 80%)
                - min_tool_life_min: minimum acceptable RUL (default 30)
                - preserve_surface_quality: bool (default True)
                - force_threshold_n: force threshold in N (default 180.0)
                - temp_threshold_c: temperature threshold in C (default 200.0)

        Returns:
            ProgramOptimizationResult with per-block suggestions.
        """
        if not blocks:
            return ProgramOptimizationResult()

        constraints = constraints or {}
        max_feed_inc_pct = constraints.get('max_feed_increase_pct', 30.0)
        max_force_pct = constraints.get('max_force_pct', 80.0)
        force_threshold = constraints.get('force_threshold_n', 180.0)
        temp_threshold = constraints.get('temp_threshold_c', 200.0)

        # Step 1: simulate at original parameters
        original_result = self.simulate_program(blocks, tool_state=tool_state)
        original_cycle_time = self._estimate_cycle_time(blocks)

        # Compute original peak force & tool life
        original_max_force = max(
            (bp.peak_force_n for bp in original_result.block_predictions), default=0.0
        )
        tool = tool_state or ToolState()
        v_mpm = math.pi * tool.diameter_mm * (blocks[0].spindle_rpm or 8000) / 1000.0
        fz = blocks[0].feed_rate_mmpm / (blocks[0].spindle_rpm * tool.flute_count) if blocks[0].spindle_rpm > 0 else 0.05
        original_tool_life = self.taylor_life_prediction(v_mpm, fz, blocks[0].axial_depth_mm)

        # Step 2-5: per-block optimization
        optimization_actions: List[BlockOptimization] = []
        optimized_blocks = []

        for i, block in enumerate(blocks):
            if block.feed_rate_mmpm <= 0 or block.spindle_rpm <= 0:
                optimized_blocks.append(block)
                continue

            pred = original_result.block_predictions[i] if i < len(original_result.block_predictions) else BlockPrediction()
            force_ratio = pred.peak_force_n / force_threshold if force_threshold > 0 else 0.0
            chatter = self._chatter_risk_score(block.spindle_rpm)

            opt_feed = block.feed_rate_mmpm
            opt_speed = block.spindle_rpm
            reason = ''

            if force_ratio < 0.60:
                # Strategy 2: headroom available, increase feed
                opt_feed = self._compute_optimal_feed(block, pred, constraints)
                reason = 'force_headroom'
            elif force_ratio > 0.85:
                # Strategy 3: too aggressive, decrease feed
                target_force = force_threshold * (max_force_pct / 100.0)
                opt_feed = self._compute_optimal_feed(
                    block, pred, {**constraints, '_target_force': target_force}
                )
                reason = 'force_reduction'

            # Strategy 4: chatter avoidance
            if chatter >= 0.5:
                fn = 250.0
                # Find nearest stable pocket: RPM = 60 * fn / (k + 0.5)
                best_rpm = opt_speed
                best_risk = chatter
                for k in range(1, 10):
                    stable_rpm = 60.0 * fn / (k + 0.5)
                    risk = self._chatter_risk_score(stable_rpm)
                    if risk < best_risk and abs(stable_rpm - block.spindle_rpm) / block.spindle_rpm < 0.2:
                        best_rpm = stable_rpm
                        best_risk = risk
                if best_rpm != opt_speed:
                    opt_speed = best_rpm
                    reason = 'chatter_avoidance' if not reason else reason

            # Strategy 5: high wear rate → reduce feed
            if pred.wear_after_block_mm > 0.20:
                wear_reduction = 0.85
                opt_feed = min(opt_feed, block.feed_rate_mmpm * wear_reduction)
                reason = 'wear_reduction' if not reason else reason

            # Clamp feed increase
            max_allowed_feed = block.feed_rate_mmpm * (1.0 + max_feed_inc_pct / 100.0)
            opt_feed = min(opt_feed, max_allowed_feed)
            # Never reduce below 50% of original
            opt_feed = max(opt_feed, block.feed_rate_mmpm * 0.5)

            if abs(opt_feed - block.feed_rate_mmpm) > 0.01 or abs(opt_speed - block.spindle_rpm) > 0.01:
                force_change = 0.0
                if pred.peak_force_n > 0:
                    # Approximate force change: F ∝ feed^0.8
                    force_change = ((opt_feed / block.feed_rate_mmpm) ** 0.8 - 1.0) * 100.0

                time_change = 0.0
                if opt_feed > 0 and block.feed_rate_mmpm > 0:
                    time_change = (block.feed_rate_mmpm / opt_feed - 1.0) * 100.0

                optimization_actions.append(BlockOptimization(
                    block_index=i,
                    original_feed=block.feed_rate_mmpm,
                    optimized_feed=opt_feed,
                    original_speed=block.spindle_rpm,
                    optimized_speed=opt_speed,
                    reason=reason,
                    force_change_pct=round(force_change, 2),
                    time_change_pct=round(time_change, 2),
                ))

                # Build optimized block copy
                new_block = GCodeBlock(
                    feed_rate_mmpm=opt_feed,
                    spindle_rpm=opt_speed,
                    axial_depth_mm=block.axial_depth_mm,
                    radial_depth_mm=block.radial_depth_mm,
                    length_mm=block.length_mm,
                )
                optimized_blocks.append(new_block)
            else:
                optimized_blocks.append(block)

        # Step 6: re-simulate optimized program
        optimized_result = self.simulate_program(optimized_blocks, tool_state=tool_state)
        optimized_cycle_time = self._estimate_cycle_time(optimized_blocks)
        optimized_max_force = max(
            (bp.peak_force_n for bp in optimized_result.block_predictions), default=0.0
        )

        # Optimized tool life
        if optimized_blocks and optimized_blocks[0].spindle_rpm > 0:
            v_opt = math.pi * tool.diameter_mm * optimized_blocks[0].spindle_rpm / 1000.0
            fz_opt = optimized_blocks[0].feed_rate_mmpm / (optimized_blocks[0].spindle_rpm * tool.flute_count)
            optimized_tool_life = self.taylor_life_prediction(v_opt, fz_opt, optimized_blocks[0].axial_depth_mm)
        else:
            optimized_tool_life = original_tool_life

        # Time savings
        time_savings_pct = 0.0
        if original_cycle_time > 0:
            time_savings_pct = (original_cycle_time - optimized_cycle_time) / original_cycle_time * 100.0

        # Risk assessment
        if optimized_max_force > force_threshold * 0.85:
            risk = 'high'
        elif optimized_max_force > force_threshold * 0.70 or len(optimization_actions) > len(blocks) * 0.5:
            risk = 'medium'
        else:
            risk = 'low'

        return ProgramOptimizationResult(
            original_cycle_time_min=original_cycle_time,
            optimized_cycle_time_min=optimized_cycle_time,
            time_savings_pct=round(time_savings_pct, 2),
            original_max_force_n=original_max_force,
            optimized_max_force_n=optimized_max_force,
            original_tool_life_min=original_tool_life,
            optimized_tool_life_min=optimized_tool_life,
            optimization_actions=optimization_actions,
            risk_assessment=risk,
        )

    def _compute_optimal_feed(
        self,
        block: GCodeBlock,
        sim_result: 'BlockPrediction',
        constraints: Dict,
    ) -> float:
        """Compute optimal feed rate for a single block.

        Using: F proportional to feed^0.8, so for target force F_target:
        feed_optimal = feed_current * (F_target / F_current) ^ (1/0.8)
        Clamped by constraints.
        """
        force_threshold = constraints.get('force_threshold_n', 180.0)
        max_force_pct = constraints.get('max_force_pct', 80.0)
        target_force = constraints.get('_target_force', force_threshold * (max_force_pct / 100.0))

        current_force = sim_result.peak_force_n
        current_feed = block.feed_rate_mmpm

        if current_force <= 0 or current_feed <= 0:
            return current_feed

        # F ∝ feed^0.8 → feed_opt = feed_cur * (F_target / F_current)^(1/0.8)
        ratio = target_force / current_force
        exponent = 1.0 / 0.8  # = 1.25
        feed_optimal = current_feed * (ratio ** exponent)

        return max(0.0, feed_optimal)

    def _estimate_cycle_time(self, blocks: List[GCodeBlock]) -> float:
        """Estimate total cycle time from block feed rates and distances.

        Returns time in minutes.
        """
        total_time = 0.0
        for block in blocks:
            if block.feed_rate_mmpm > 0 and block.length_mm > 0:
                total_time += block.length_mm / block.feed_rate_mmpm
        return total_time

    def get_program_statistics(
        self,
        blocks: List[GCodeBlock],
        sim_result: 'SimulationResult',
    ) -> Dict:
        """Get overall program statistics.

        Args:
            blocks: The G-code blocks that were simulated.
            sim_result: The SimulationResult from simulate_program().

        Returns:
            Dict with program-level statistics.
        """
        total_blocks = len(blocks)
        cutting_blocks = sum(1 for b in blocks if b.feed_rate_mmpm > 0 and b.spindle_rpm > 0)
        rapid_blocks = total_blocks - cutting_blocks

        total_distance = sum(b.length_mm for b in blocks)
        estimated_cycle_time = self._estimate_cycle_time(blocks)

        predictions = sim_result.block_predictions
        peak_force = max((p.peak_force_n for p in predictions), default=0.0)
        avg_force = (
            sum(p.avg_force_n for p in predictions) / len(predictions)
            if predictions else 0.0
        )
        total_wear = sim_result.final_wear_mm

        # Thermal hotspots: blocks where temperature > threshold
        temp_threshold = 200.0  # degrees C
        thermal_hotspots = [
            i for i, p in enumerate(predictions)
            if p.temperature_rise_c > temp_threshold
        ]

        # Chatter risk blocks
        chatter_risk_blocks = [
            i for i, b in enumerate(blocks)
            if self._chatter_risk_score(b.spindle_rpm) >= 0.5
        ]

        # Force utilization: avg force / threshold
        force_threshold = 180.0
        force_utilization = (avg_force / force_threshold * 100.0) if force_threshold > 0 else 0.0

        return {
            'total_blocks': total_blocks,
            'cutting_blocks': cutting_blocks,
            'rapid_blocks': rapid_blocks,
            'total_distance_mm': total_distance,
            'estimated_cycle_time_min': estimated_cycle_time,
            'peak_force_n': peak_force,
            'avg_force_n': avg_force,
            'total_wear_mm': total_wear,
            'thermal_hotspots': thermal_hotspots,
            'chatter_risk_blocks': chatter_risk_blocks,
            'force_utilization_pct': round(force_utilization, 2),
        }
