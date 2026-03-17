"""
Cutting Simulation Proxy — Python port of Unity's CuttingForceEngine + ToolWearModel.

Provides force prediction, wear estimation, and RUL calculation for the
PredictionRunner to replace hardcoded values with real simulation results.
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

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


@dataclass
class CoolantRecommendation:
    """Advisory recommendation for coolant strategy."""
    recommended_type: str  # 'dry', 'mist', 'flood', 'high_pressure', 'cryogenic'
    current_type: str
    reason: str
    thermal_improvement_pct: float
    wear_improvement_pct: float
    cost_factor: float  # 1.0 = baseline (dry)
    environmental_score: float  # 0-1, higher = greener


class CoolantOptimizer:
    """Recommends optimal coolant strategies based on cutting conditions and material."""

    COOLANT_TYPES = ('dry', 'mist', 'flood', 'high_pressure', 'cryogenic')

    # Relative cost model (dry = 1.0 baseline)
    COST_MODEL: Dict[str, float] = {
        'dry': 1.0,
        'mist': 1.8,
        'flood': 3.5,
        'high_pressure': 6.0,
        'cryogenic': 9.0,
    }

    # Environmental score (higher = greener)
    ENV_SCORE: Dict[str, float] = {
        'dry': 1.0,
        'mist': 0.85,
        'flood': 0.45,
        'high_pressure': 0.35,
        'cryogenic': 0.55,  # CO2/LN2 — no oil waste, but energy-intensive
    }

    # Material-specific coolant effectiveness tables
    # Each entry: {coolant_type: (thermal_reduction_pct, wear_reduction_pct)}
    MATERIAL_TABLES: Dict[str, Dict[str, tuple]] = {
        '6061-T6': {
            'dry':           (0.0,  0.0),
            'mist':          (25.0, 15.0),
            'flood':         (55.0, 35.0),
            'high_pressure': (70.0, 50.0),
            'cryogenic':     (85.0, 60.0),
        },
        '304-stainless': {
            'dry':           (0.0,  0.0),
            'mist':          (20.0, 10.0),
            'flood':         (50.0, 30.0),
            'high_pressure': (72.0, 55.0),
            'cryogenic':     (88.0, 65.0),
        },
        'Ti-6Al-4V': {
            'dry':           (0.0,  0.0),
            'mist':          (15.0, 8.0),
            'flood':         (40.0, 25.0),
            'high_pressure': (75.0, 60.0),
            'cryogenic':     (90.0, 70.0),
        },
        'Inconel-718': {
            'dry':           (0.0,  0.0),
            'mist':          (12.0, 6.0),
            'flood':         (35.0, 20.0),
            'high_pressure': (78.0, 65.0),
            'cryogenic':     (92.0, 75.0),
        },
    }

    # Materials that always require aggressive cooling
    HARD_MATERIALS = {'Ti-6Al-4V', 'Inconel-718'}

    # Thermal limit (degrees C) — above this, upgrade coolant
    THERMAL_LIMIT = 250.0

    # Wear rate threshold (mm/min) — above this, upgrade coolant to extend tool life
    WEAR_RATE_LIMIT = 0.02

    def __init__(self, material: str = '6061-T6'):
        self.material = material
        if material not in self.MATERIAL_TABLES:
            self._effectiveness = self.MATERIAL_TABLES['6061-T6']
        else:
            self._effectiveness = self.MATERIAL_TABLES[material]

    def get_coolant_cost_model(self) -> Dict[str, float]:
        """Return relative cost factors for all coolant types."""
        return dict(self.COST_MODEL)

    def recommend_coolant(
        self,
        operation_type: str,
        cutting_speed: float,
        depth: float,
        current_temp: float,
        current_wear_rate: float,
        current_coolant: str,
    ) -> CoolantRecommendation:
        """Recommend the best coolant for the given cutting conditions.

        Args:
            operation_type: 'roughing', 'finishing', 'slotting', etc.
            cutting_speed: surface speed in m/min
            depth: axial depth of cut in mm
            current_temp: current tool/workpiece temperature in degrees C
            current_wear_rate: current flank wear rate in mm/min
            current_coolant: currently active coolant type

        Returns:
            CoolantRecommendation with the best option.
        """
        # Guard: zero/negative speed or depth → dry (machine is idle)
        if cutting_speed <= 0 or depth <= 0:
            eff = self._effectiveness['dry']
            return CoolantRecommendation(
                recommended_type='dry',
                current_type=current_coolant,
                reason='Machine idle or no cutting — coolant unnecessary',
                thermal_improvement_pct=eff[0],
                wear_improvement_pct=eff[1],
                cost_factor=self.COST_MODEL['dry'],
                environmental_score=self.ENV_SCORE['dry'],
            )

        recommended = self._select_coolant(
            operation_type, cutting_speed, depth, current_temp, current_wear_rate,
        )

        eff = self._effectiveness[recommended]
        current_eff = self._effectiveness.get(current_coolant, (0.0, 0.0))
        thermal_imp = eff[0] - current_eff[0]
        wear_imp = eff[1] - current_eff[1]

        reason = self._build_reason(
            recommended, operation_type, cutting_speed, depth,
            current_temp, current_wear_rate,
        )

        return CoolantRecommendation(
            recommended_type=recommended,
            current_type=current_coolant,
            reason=reason,
            thermal_improvement_pct=round(thermal_imp, 2),
            wear_improvement_pct=round(wear_imp, 2),
            cost_factor=self.COST_MODEL[recommended],
            environmental_score=self.ENV_SCORE[recommended],
        )

    def evaluate_all_coolants(
        self,
        operation_type: str,
        cutting_speed: float,
        depth: float,
    ) -> List[CoolantRecommendation]:
        """Evaluate all coolant options and return sorted by net benefit.

        Net benefit = (thermal_improvement + wear_improvement) / cost_factor.
        """
        results: List[CoolantRecommendation] = []
        for ct in self.COOLANT_TYPES:
            eff = self._effectiveness[ct]
            results.append(CoolantRecommendation(
                recommended_type=ct,
                current_type='dry',  # baseline comparison
                reason=f'{ct} evaluation for {operation_type}',
                thermal_improvement_pct=eff[0],
                wear_improvement_pct=eff[1],
                cost_factor=self.COST_MODEL[ct],
                environmental_score=self.ENV_SCORE[ct],
            ))

        # Sort by net benefit descending
        def net_benefit(rec: CoolantRecommendation) -> float:
            return (rec.thermal_improvement_pct + rec.wear_improvement_pct) / max(rec.cost_factor, 0.01)

        results.sort(key=net_benefit, reverse=True)
        return results

    # ------------------------------------------------------------------
    # Internal decision logic
    # ------------------------------------------------------------------

    def _select_coolant(
        self,
        operation_type: str,
        cutting_speed: float,
        depth: float,
        current_temp: float,
        current_wear_rate: float,
    ) -> str:
        """Core decision logic for coolant selection."""
        # Rule 1: Hard materials always need aggressive cooling
        if self.material in self.HARD_MATERIALS:
            if current_temp > self.THERMAL_LIMIT:
                return 'cryogenic'
            return 'high_pressure'

        # Rule 2: Near thermal limit → upgrade
        if current_temp > self.THERMAL_LIMIT:
            if cutting_speed > 150:
                return 'high_pressure'
            return 'flood'

        # Rule 3: High wear rate → upgrade to extend tool life
        if current_wear_rate > self.WEAR_RATE_LIMIT:
            if cutting_speed > 150:
                return 'high_pressure'
            return 'flood'

        # Rule 4: High speed + deep cut → flood or high_pressure
        if cutting_speed > 200 and depth > 3.0:
            return 'high_pressure'
        if cutting_speed > 100 and depth > 2.0:
            return 'flood'

        # Rule 5: Low speed + shallow cut → dry or mist (save cost)
        if cutting_speed <= 60 and depth <= 1.0:
            return 'dry'
        if cutting_speed <= 100 and depth <= 1.5:
            return 'mist'

        # Default: flood for moderate conditions
        return 'flood'

    def _build_reason(
        self,
        recommended: str,
        operation_type: str,
        cutting_speed: float,
        depth: float,
        current_temp: float,
        current_wear_rate: float,
    ) -> str:
        """Build a human-readable reason string."""
        parts = []
        if self.material in self.HARD_MATERIALS:
            parts.append(f'{self.material} requires aggressive cooling')
        if current_temp > self.THERMAL_LIMIT:
            parts.append(f'temperature {current_temp:.0f}C exceeds limit')
        if current_wear_rate > self.WEAR_RATE_LIMIT:
            parts.append(f'wear rate {current_wear_rate:.4f} mm/min is high')
        if cutting_speed > 200 and depth > 3.0:
            parts.append('high speed + deep cut')
        elif cutting_speed <= 60 and depth <= 1.0:
            parts.append('low speed + shallow cut — coolant savings possible')
        if not parts:
            parts.append(f'{operation_type} at {cutting_speed:.0f} m/min, {depth:.1f}mm depth')
        return f'{recommended}: ' + '; '.join(parts)


@dataclass
class ThermalZone:
    """Represents a thermal zone in the multi-zone thermal model."""
    zone_id: str
    temperature_c: float = 25.0
    thermal_mass_j_per_c: float = 1000.0
    heat_input_w: float = 0.0
    heat_dissipation_w_per_c: float = 1.0
    max_safe_temp_c: float = 100.0
    adjacent_zones: List[str] = field(default_factory=list)


class ThermalModel:
    """Multi-zone thermal model for CNC milling simulation.

    Models heat generation, dissipation, and inter-zone conduction across
    five thermal zones: spindle, workpiece, tool_holder, coolant_reservoir,
    and ambient.
    """

    # Coolant effectiveness multiplier on dissipation coefficient
    COOLANT_DISSIPATION_MULTIPLIER: Dict[str, float] = {
        'dry': 1.0,
        'mist': 1.5,
        'flood': 2.5,
        'high_pressure': 3.5,
        'cryogenic': 5.0,
    }

    # Inter-zone thermal conductance (W/C) between adjacent zones
    ZONE_CONDUCTANCE: float = 2.0

    # Cutting power partition fractions
    POWER_PARTITION = {
        'workpiece': 0.60,
        'tool_holder': 0.25,
        'spindle': 0.10,
        'coolant_reservoir': 0.05,
    }

    AMBIENT_TEMP: float = 25.0

    def __init__(self) -> None:
        self._zones: Dict[str, ThermalZone] = {}
        self._init_zones()

    def _init_zones(self) -> None:
        """Initialize the five default thermal zones."""
        self._zones['spindle'] = ThermalZone(
            zone_id='spindle',
            temperature_c=self.AMBIENT_TEMP,
            thermal_mass_j_per_c=5000.0,
            heat_dissipation_w_per_c=5.0,
            max_safe_temp_c=80.0,
            adjacent_zones=['tool_holder', 'ambient'],
        )
        self._zones['workpiece'] = ThermalZone(
            zone_id='workpiece',
            temperature_c=self.AMBIENT_TEMP,
            thermal_mass_j_per_c=2000.0,
            heat_dissipation_w_per_c=3.0,
            max_safe_temp_c=200.0,
            adjacent_zones=['tool_holder', 'coolant_reservoir', 'ambient'],
        )
        self._zones['tool_holder'] = ThermalZone(
            zone_id='tool_holder',
            temperature_c=self.AMBIENT_TEMP,
            thermal_mass_j_per_c=500.0,
            heat_dissipation_w_per_c=4.0,
            max_safe_temp_c=150.0,
            adjacent_zones=['spindle', 'workpiece', 'ambient'],
        )
        self._zones['coolant_reservoir'] = ThermalZone(
            zone_id='coolant_reservoir',
            temperature_c=self.AMBIENT_TEMP,
            thermal_mass_j_per_c=20000.0,
            heat_dissipation_w_per_c=10.0,
            max_safe_temp_c=45.0,
            adjacent_zones=['workpiece', 'ambient'],
        )
        self._zones['ambient'] = ThermalZone(
            zone_id='ambient',
            temperature_c=self.AMBIENT_TEMP,
            thermal_mass_j_per_c=1e12,  # effectively infinite
            heat_dissipation_w_per_c=0.0,
            max_safe_temp_c=1e6,
            adjacent_zones=[],
        )

    def update(self, dt_sec: float, cutting_power_w: float,
               spindle_power_w: float = 0.0,
               coolant_type: str = 'flood') -> Dict[str, float]:
        """Advance the thermal model by dt_sec seconds.

        Args:
            dt_sec: Time step in seconds.
            cutting_power_w: Total cutting power in watts.
            spindle_power_w: Additional spindle motor heat in watts.
            coolant_type: Active coolant type for dissipation scaling.

        Returns:
            Dict mapping zone_id to current temperature.
        """
        if dt_sec <= 0:
            return self.get_thermal_state()

        coolant_mult = self.COOLANT_DISSIPATION_MULTIPLIER.get(coolant_type, 1.0)

        # Assign heat inputs based on power partition
        for zone_id, fraction in self.POWER_PARTITION.items():
            zone = self._zones[zone_id]
            zone.heat_input_w = cutting_power_w * fraction
        # Add spindle motor heat to spindle zone
        self._zones['spindle'].heat_input_w += spindle_power_w

        # Compute temperature changes for non-ambient zones
        for zone_id, zone in self._zones.items():
            if zone_id == 'ambient':
                continue

            # Net heat input
            q_in = zone.heat_input_w

            # Dissipation to ambient (convection/conduction)
            delta_t_ambient = zone.temperature_c - self.AMBIENT_TEMP
            q_dissipation = zone.heat_dissipation_w_per_c * delta_t_ambient * coolant_mult

            # Inter-zone conduction
            q_conduction = 0.0
            for adj_id in zone.adjacent_zones:
                if adj_id == 'ambient':
                    continue
                adj_zone = self._zones.get(adj_id)
                if adj_zone is None:
                    continue
                temp_diff = zone.temperature_c - adj_zone.temperature_c
                q_conduction += self.ZONE_CONDUCTANCE * temp_diff

            # Net heat balance: dT = (q_in - q_out) * dt / thermal_mass
            q_net = q_in - q_dissipation - q_conduction
            dT = (q_net * dt_sec) / zone.thermal_mass_j_per_c
            zone.temperature_c += dT

        return self.get_thermal_state()

    def get_thermal_state(self) -> Dict[str, float]:
        """Return current temperatures for all zones."""
        return {zid: z.temperature_c for zid, z in self._zones.items()}

    def get_thermal_warnings(self) -> List[tuple]:
        """Return warnings for zones approaching or exceeding safe limits.

        Returns:
            List of (zone_id, current_temp, max_safe_temp, pct_of_limit) tuples
            for zones at or above 80% of their safe limit (relative to ambient).
        """
        warnings = []
        for zid, zone in self._zones.items():
            if zid == 'ambient':
                continue
            temp_range = zone.max_safe_temp_c - self.AMBIENT_TEMP
            if temp_range <= 0:
                continue
            current_rise = zone.temperature_c - self.AMBIENT_TEMP
            pct = (current_rise / temp_range) * 100.0
            if pct >= 80.0:
                warnings.append((
                    zid,
                    zone.temperature_c,
                    zone.max_safe_temp_c,
                    round(pct, 2),
                ))
        return warnings

    def predict_thermal_trajectory(self, power_w: float, duration_sec: float,
                                   steps: int = 10,
                                   coolant_type: str = 'flood') -> List[Dict[str, float]]:
        """Predict temperature evolution over a future time period.

        Creates a copy of current state and simulates forward without
        modifying the actual model state.

        Args:
            power_w: Constant cutting power in watts.
            duration_sec: Total prediction horizon in seconds.
            steps: Number of time steps to simulate.
            coolant_type: Coolant type for the prediction.

        Returns:
            List of dicts, each mapping zone_id to temperature at that step.
        """
        # Save current state
        saved_temps = {zid: z.temperature_c for zid, z in self._zones.items()}

        trajectory = []
        dt = duration_sec / max(steps, 1)
        for _ in range(steps):
            state = self.update(dt, power_w, coolant_type=coolant_type)
            trajectory.append(dict(state))

        # Restore original state
        for zid, temp in saved_temps.items():
            self._zones[zid].temperature_c = temp

        return trajectory

    def reset(self) -> None:
        """Reset all zones to ambient temperature."""
        for zone in self._zones.items():
            pass
        for zid, zone in self._zones.items():
            zone.temperature_c = self.AMBIENT_TEMP
            zone.heat_input_w = 0.0

    def get_time_to_limit(self, zone_id: str) -> Optional[float]:
        """Estimate seconds until a zone reaches its max safe temperature.

        Uses current heat input rate and dissipation to extrapolate linearly.
        Returns None if the zone is cooling or already at/above limit, or
        if the zone does not exist.

        Args:
            zone_id: The zone to check.

        Returns:
            Estimated seconds to reach limit, or None if not applicable.
        """
        zone = self._zones.get(zone_id)
        if zone is None:
            return None
        if zone.temperature_c >= zone.max_safe_temp_c:
            return 0.0

        # Net heat rate at current temperature
        delta_t = zone.temperature_c - self.AMBIENT_TEMP
        q_dissipation = zone.heat_dissipation_w_per_c * delta_t
        q_net = zone.heat_input_w - q_dissipation
        if q_net <= 0:
            return None  # zone is cooling or stable

        # dT/dt = q_net / thermal_mass
        dT_dt = q_net / zone.thermal_mass_j_per_c
        remaining_temp = zone.max_safe_temp_c - zone.temperature_c
        return remaining_temp / dT_dt


@dataclass
class WorkholdingSetup:
    """Workholding configuration for force analysis."""
    setup_type: str  # VISE, CHUCK_3JAW, CHUCK_4JAW, VACUUM, FIXTURE_PLATE, MAGNETIC
    clamping_force_n: float
    friction_coefficient: float  # steel-on-steel ~0.15, soft jaws ~0.3
    num_clamp_points: int
    workpiece_mass_kg: float
    workpiece_dimensions_mm: tuple  # (L, W, H)
    safety_factor: float = 2.0

    VALID_TYPES = {'VISE', 'CHUCK_3JAW', 'CHUCK_4JAW', 'VACUUM', 'FIXTURE_PLATE', 'MAGNETIC'}


@dataclass
class WorkholdingAnalysis:
    """Result of workholding force analysis."""
    is_secure: bool
    safety_margin: float  # ratio of holding force to cutting force
    required_clamping_force_n: float
    actual_holding_force_n: float
    critical_direction: str  # X, Y, or Z
    max_cutting_force_n: float
    lift_off_risk: bool
    rotation_risk: bool
    recommendations: List[str] = field(default_factory=list)


class WorkholdingAnalyzer:
    """Analyzes workholding adequacy against cutting forces."""

    GRAVITY = 9.81  # m/s^2

    def analyze(self, setup: WorkholdingSetup, cutting_forces: dict) -> WorkholdingAnalysis:
        """Analyze whether workholding is adequate for given cutting forces.

        Args:
            setup: Workholding configuration.
            cutting_forces: dict with keys 'Fx', 'Fy', 'Fz' (Newtons).
                Fz is axial (vertical), Fx/Fy are lateral.

        Returns:
            WorkholdingAnalysis with security assessment.
        """
        fx = abs(cutting_forces.get('Fx', 0.0))
        fy = abs(cutting_forces.get('Fy', 0.0))
        fz = abs(cutting_forces.get('Fz', 0.0))

        # Gravity contribution (holds workpiece down)
        gravity_force = setup.workpiece_mass_kg * self.GRAVITY

        # Effective holding force from clamps (friction-based lateral restraint)
        holding_force_lateral = (
            setup.clamping_force_n * setup.friction_coefficient * setup.num_clamp_points
        )

        # Vertical holding: clamping force directly opposes lift + gravity helps
        holding_force_vertical = (
            setup.clamping_force_n * setup.num_clamp_points + gravity_force
        )

        # For vacuum and magnetic, holding is purely normal (vertical)
        # and lateral resistance comes from friction
        if setup.setup_type in ('VACUUM', 'MAGNETIC'):
            holding_force_vertical = (
                setup.clamping_force_n * setup.num_clamp_points + gravity_force
            )
            holding_force_lateral = (
                setup.clamping_force_n * setup.friction_coefficient
                * setup.num_clamp_points
            )

        # Find critical direction
        force_vs_holding = {
            'X': fx / max(holding_force_lateral, 1e-9),
            'Y': fy / max(holding_force_lateral, 1e-9),
            'Z': fz / max(holding_force_vertical, 1e-9),
        }
        critical_direction = max(force_vs_holding, key=force_vs_holding.get)
        max_cutting_force = max(fx, fy, fz)

        # Effective holding force in critical direction
        if critical_direction == 'Z':
            actual_holding = holding_force_vertical
        else:
            actual_holding = holding_force_lateral

        # Safety margin
        if max_cutting_force < 1e-9:
            safety_margin = float('inf')
        else:
            safety_margin = actual_holding / max_cutting_force

        is_secure = safety_margin >= setup.safety_factor

        # Lift-off risk: axial force exceeds vertical hold
        lift_off_risk = fz > holding_force_vertical

        # Rotation risk: torque from lateral forces exceeds friction torque
        # Torque from cutting = max(Fx, Fy) * moment_arm (half workpiece dimension)
        moment_arm_m = max(
            setup.workpiece_dimensions_mm[0],
            setup.workpiece_dimensions_mm[1],
        ) / 2000.0  # mm → m, half-length
        cutting_torque = max(fx, fy) * moment_arm_m

        # Friction torque from clamps: clamping_force * friction * clamp_spread
        clamp_spread_m = min(
            setup.workpiece_dimensions_mm[0],
            setup.workpiece_dimensions_mm[1],
        ) / 2000.0  # mm → m
        friction_torque = (
            setup.clamping_force_n * setup.friction_coefficient
            * setup.num_clamp_points * clamp_spread_m
        )
        rotation_risk = cutting_torque > friction_torque

        # Required clamping force (to achieve safety_factor margin)
        if max_cutting_force < 1e-9:
            required_clamping = 0.0
        else:
            # In the critical direction, solve for clamping_force
            if critical_direction == 'Z':
                # clamping_force * num_clamps + gravity >= max_force * safety_factor
                required_total = max_cutting_force * setup.safety_factor - gravity_force
                required_clamping = max(0.0, required_total / max(setup.num_clamp_points, 1))
            else:
                # clamping_force * friction * num_clamps >= max_force * safety_factor
                denom = setup.friction_coefficient * max(setup.num_clamp_points, 1)
                required_clamping = max_cutting_force * setup.safety_factor / max(denom, 1e-9)

        # Recommendations
        recommendations: List[str] = []
        if not is_secure:
            recommendations.append(
                f"Increase clamping force to at least {required_clamping:.0f} N"
            )
        if lift_off_risk:
            recommendations.append(
                "Lift-off risk detected — add top clamps or reduce axial depth of cut"
            )
        if rotation_risk:
            recommendations.append(
                "Rotation risk detected — add additional clamp points or use fixture plate"
            )
        if setup.setup_type == 'VACUUM' and max_cutting_force > 500:
            recommendations.append(
                "Vacuum holding may be insufficient for high cutting forces — consider vise"
            )
        if safety_margin < 1.5 and is_secure:
            recommendations.append(
                "Safety margin is low — consider increasing clamping force"
            )

        return WorkholdingAnalysis(
            is_secure=is_secure,
            safety_margin=safety_margin,
            required_clamping_force_n=required_clamping,
            actual_holding_force_n=actual_holding,
            critical_direction=critical_direction,
            max_cutting_force_n=max_cutting_force,
            lift_off_risk=lift_off_risk,
            rotation_risk=rotation_risk,
            recommendations=recommendations,
        )

    def recommend_setup(
        self, cutting_forces: dict, workpiece_mass_kg: float,
        workpiece_dimensions_mm: tuple = (100.0, 100.0, 50.0),
    ) -> WorkholdingSetup:
        """Suggest a workholding setup for given cutting forces.

        Args:
            cutting_forces: dict with 'Fx', 'Fy', 'Fz' in Newtons.
            workpiece_mass_kg: workpiece mass.
            workpiece_dimensions_mm: (L, W, H) tuple.

        Returns:
            Recommended WorkholdingSetup.
        """
        fx = abs(cutting_forces.get('Fx', 0.0))
        fy = abs(cutting_forces.get('Fy', 0.0))
        fz = abs(cutting_forces.get('Fz', 0.0))
        max_force = max(fx, fy, fz, 1e-9)
        safety_factor = 2.5  # conservative default

        # Choose setup type based on force magnitude
        if max_force < 200:
            setup_type = 'VACUUM'
            friction = 0.4
            num_clamps = 1
        elif max_force < 2000:
            setup_type = 'VISE'
            friction = 0.15
            num_clamps = 2
        else:
            setup_type = 'FIXTURE_PLATE'
            friction = 0.25
            num_clamps = 4

        # Compute minimum clamping force for lateral security
        required_lateral = max_force * safety_factor / (friction * num_clamps)
        # Compute minimum for vertical security
        gravity = workpiece_mass_kg * self.GRAVITY
        required_vertical = max(0.0, (fz * safety_factor - gravity) / num_clamps)

        clamping_force = max(required_lateral, required_vertical)

        return WorkholdingSetup(
            setup_type=setup_type,
            clamping_force_n=clamping_force,
            friction_coefficient=friction,
            num_clamp_points=num_clamps,
            workpiece_mass_kg=workpiece_mass_kg,
            workpiece_dimensions_mm=workpiece_dimensions_mm,
            safety_factor=safety_factor,
        )

    def get_max_safe_depth(
        self, setup: WorkholdingSetup, feed_mm_per_tooth: float,
        speed_rpm: float, material_kc: float = 2000.0,
    ) -> float:
        """Compute maximum depth of cut before workpiece becomes insecure.

        Uses a simplified Kienzle force model: F = kc * ap * f
        where kc = specific cutting force (N/mm^2), ap = depth (mm), f = feed (mm).

        Args:
            setup: Workholding configuration.
            feed_mm_per_tooth: feed per tooth in mm.
            speed_rpm: spindle speed (used for context, not directly in simplified model).
            material_kc: specific cutting force in N/mm^2 (default 2000 for steel).

        Returns:
            Maximum safe depth of cut in mm.
        """
        # Holding force (lateral, which is typically the limiting case)
        holding_force = (
            setup.clamping_force_n * setup.friction_coefficient * setup.num_clamp_points
        )
        safe_holding = holding_force / setup.safety_factor

        # F = kc * ap * f  →  ap = F / (kc * f)
        if feed_mm_per_tooth < 1e-9 or material_kc < 1e-9:
            return 0.0

        max_depth = safe_holding / (material_kc * feed_mm_per_tooth)
        return max(0.0, max_depth)


@dataclass
class ChipLoadMetrics:
    """Metrics from chip load analysis for a single cutting condition."""
    chip_load_mm: float = 0.0              # feed per tooth (mm/tooth)
    chip_thinning_factor: float = 1.0      # radial engagement correction
    effective_chip_load_mm: float = 0.0    # actual chip thickness after thinning
    chip_volume_rate_mm3_per_min: float = 0.0
    mrr_cm3_per_min: float = 0.0          # material removal rate
    specific_energy_j_per_mm3: float = 0.0
    is_optimal: bool = False
    deviation_from_optimal_pct: float = 0.0
    recommendation: str = ''


class ChipLoadMonitor:
    """Monitors and optimizes chip load for CNC milling operations.

    Tracks feed-per-tooth against material-specific optimal ranges,
    computes chip thinning corrections for partial radial engagement,
    and provides trend analysis for process stability monitoring.
    """

    # Optimal chip load ranges (mm/tooth) by material and tool diameter category.
    # Keys: material name -> (min_chip_load, max_chip_load) for standard diameters.
    OPTIMAL_RANGES: Dict[str, tuple] = {
        '6061-T6':  (0.05, 0.15),
        '7075-T6':  (0.04, 0.12),
        '304-SS':   (0.02, 0.08),
        'Ti-6Al-4V': (0.01, 0.05),
    }

    # Specific cutting energy (J/mm^3) by material.
    SPECIFIC_ENERGY: Dict[str, float] = {
        '6061-T6':  0.9,
        '7075-T6':  1.1,
        '304-SS':   2.8,
        'Ti-6Al-4V': 3.5,
    }

    def __init__(
        self,
        material: str = '6061-T6',
        num_flutes: int = 2,
        tool_diameter_mm: float = 6.35,
    ):
        self.material = material
        self.num_flutes = num_flutes
        self.tool_diameter_mm = tool_diameter_mm
        # Default radial and axial engagement (can be overridden per call)
        self._default_ae_mm = tool_diameter_mm * 0.5  # 50% radial engagement
        self._default_ap_mm = 1.5

    # ----- helpers -----

    def _get_range(self) -> tuple:
        """Return (min, max) optimal chip load for current material."""
        return self.OPTIMAL_RANGES.get(self.material, (0.05, 0.15))

    def _get_specific_energy(self) -> float:
        return self.SPECIFIC_ENERGY.get(self.material, 1.0)

    # ----- core API -----

    def compute_chip_load(
        self,
        feed_rate_mmpm: float,
        rpm: float,
        ae_mm: Optional[float] = None,
        ap_mm: Optional[float] = None,
    ) -> ChipLoadMetrics:
        """Compute chip load metrics for given cutting conditions.

        Args:
            feed_rate_mmpm: Table feed rate in mm/min.
            rpm: Spindle speed in rev/min.
            ae_mm: Radial depth of cut (width). Defaults to 50% of tool diameter.
            ap_mm: Axial depth of cut. Defaults to 1.5 mm.

        Returns:
            ChipLoadMetrics with all computed values.
        """
        if rpm <= 0 or self.num_flutes <= 0:
            return ChipLoadMetrics(
                recommendation='Invalid parameters: RPM and flutes must be > 0',
            )

        ae = ae_mm if ae_mm is not None else self._default_ae_mm
        ap = ap_mm if ap_mm is not None else self._default_ap_mm
        d = self.tool_diameter_mm

        # Basic chip load (feed per tooth)
        chip_load = feed_rate_mmpm / (rpm * self.num_flutes)

        # Chip thinning factor for partial radial engagement
        # When ae < d, the actual chip is thinner than the programmed feed/tooth.
        # thinning = sqrt(1 - (1 - 2*ae/d)^2)
        ratio_ae_d = min(ae / d, 1.0) if d > 0 else 1.0
        inner_term = 1.0 - 2.0 * ratio_ae_d
        thinning_arg = 1.0 - inner_term ** 2
        chip_thinning_factor = math.sqrt(max(0.0, thinning_arg))
        # Guard against zero thinning (full slotting gives thinning = 1.0)
        if chip_thinning_factor < 1e-9:
            chip_thinning_factor = 1.0

        effective_chip_load = chip_load / chip_thinning_factor

        # Material removal rate
        mrr_mm3 = ap * ae * feed_rate_mmpm   # mm^3/min
        mrr_cm3 = mrr_mm3 / 1000.0           # cm^3/min

        # Chip volume rate (per tooth)
        chip_volume_rate = mrr_mm3  # total volumetric rate

        # Specific energy
        se = self._get_specific_energy()

        # Optimal range check
        rng_min, rng_max = self._get_range()
        mid_optimal = (rng_min + rng_max) / 2.0
        is_optimal = rng_min <= chip_load <= rng_max

        if mid_optimal > 0:
            deviation_pct = ((chip_load - mid_optimal) / mid_optimal) * 100.0
        else:
            deviation_pct = 0.0

        # Recommendation
        if chip_load < rng_min:
            recommendation = (
                f'Chip load {chip_load:.4f} mm is below optimal range '
                f'[{rng_min:.3f}, {rng_max:.3f}]. Increase feed rate to avoid '
                f'recutting and accelerated wear.'
            )
        elif chip_load > rng_max:
            recommendation = (
                f'Chip load {chip_load:.4f} mm exceeds optimal range '
                f'[{rng_min:.3f}, {rng_max:.3f}]. Reduce feed rate to prevent '
                f'excessive tool loading.'
            )
        else:
            recommendation = (
                f'Chip load {chip_load:.4f} mm is within optimal range '
                f'[{rng_min:.3f}, {rng_max:.3f}].'
            )

        return ChipLoadMetrics(
            chip_load_mm=chip_load,
            chip_thinning_factor=chip_thinning_factor,
            effective_chip_load_mm=effective_chip_load,
            chip_volume_rate_mm3_per_min=chip_volume_rate,
            mrr_cm3_per_min=mrr_cm3,
            specific_energy_j_per_mm3=se,
            is_optimal=is_optimal,
            deviation_from_optimal_pct=deviation_pct,
            recommendation=recommendation,
        )

    def get_optimal_feed(
        self,
        rpm: float,
        depth_mm: float = 1.5,
        width_mm: Optional[float] = None,
    ) -> float:
        """Compute the feed rate (mm/min) that achieves the midpoint optimal chip load.

        Args:
            rpm: Spindle speed in rev/min.
            depth_mm: Axial depth of cut (not used in feed calc, kept for API symmetry).
            width_mm: Radial engagement. Defaults to 50% of tool diameter.

        Returns:
            Optimal feed rate in mm/min, or 0.0 if parameters are invalid.
        """
        if rpm <= 0 or self.num_flutes <= 0:
            return 0.0

        rng_min, rng_max = self._get_range()
        target_chip_load = (rng_min + rng_max) / 2.0

        # feed = chip_load * rpm * flutes
        return target_chip_load * rpm * self.num_flutes

    def analyze_trend(self, history: List[ChipLoadMetrics]) -> dict:
        """Analyze a history of chip load metrics for trends and stability.

        Args:
            history: Ordered list of ChipLoadMetrics (oldest first).

        Returns:
            Dict with keys: trend ('stable', 'increasing', 'decreasing'),
            avg_chip_load, std_chip_load, stability_score (0-1, higher = more stable).
        """
        if not history:
            return {
                'trend': 'stable',
                'avg_chip_load': 0.0,
                'std_chip_load': 0.0,
                'stability_score': 1.0,
            }

        loads = [m.chip_load_mm for m in history]
        n = len(loads)
        avg = sum(loads) / n
        variance = sum((x - avg) ** 2 for x in loads) / n if n > 0 else 0.0
        std = math.sqrt(variance)

        # Trend detection via simple linear regression slope
        if n >= 2:
            x_mean = (n - 1) / 2.0
            y_mean = avg
            num = sum((i - x_mean) * (loads[i] - y_mean) for i in range(n))
            den = sum((i - x_mean) ** 2 for i in range(n))
            slope = num / den if den > 0 else 0.0

            # Normalise slope relative to mean
            rel_slope = slope / avg if avg > 0 else 0.0
            if rel_slope > 0.02:
                trend = 'increasing'
            elif rel_slope < -0.02:
                trend = 'decreasing'
            else:
                trend = 'stable'
        else:
            trend = 'stable'

        # Stability score: 1.0 when std is 0, decreasing as variation grows
        cv = std / avg if avg > 0 else 0.0
        stability_score = max(0.0, min(1.0, 1.0 - cv * 5.0))

        return {
            'trend': trend,
            'avg_chip_load': avg,
            'std_chip_load': std,
            'stability_score': stability_score,
        }

    def check_recutting_risk(self, chip_load_mm: float) -> bool:
        """Check whether the chip load is too thin, risking chip recutting.

        Recutting occurs when chips are too thin to curl away from the cutter
        and instead get re-cut, accelerating flank wear.

        Args:
            chip_load_mm: Current feed per tooth in mm.

        Returns:
            True if recutting risk is present (chip load below minimum).
        """
        rng_min, _ = self._get_range()
        return chip_load_mm < rng_min


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
        self._thermal_model: ThermalModel = ThermalModel()

    @property
    def coolant(self) -> 'CoolantConfig':
        """Return the current coolant configuration."""
        return self._coolant

    def set_coolant(self, config: 'CoolantConfig') -> None:
        """Set coolant/lubrication configuration."""
        self._coolant = config

    def update_thermal_state(self, block: 'GCodeBlock', dt_sec: float) -> Dict[str, float]:
        """Update the thermal model based on a G-code block's cutting power.

        Calculates cutting power from the block parameters and feeds it
        into the multi-zone ThermalModel.

        Args:
            block: The G-code block with current cutting parameters.
            dt_sec: Time step in seconds.

        Returns:
            Dict mapping zone_id to current temperature.
        """
        cutting_power_w = 0.0
        spindle_power_w = 0.0

        if block.spindle_rpm > 0 and block.feed_rate_mmpm > 0:
            flute_count = 2
            fz = block.feed_rate_mmpm / (block.spindle_rpm * flute_count)
            forces = self.calculate_forces(
                spindle_rpm=block.spindle_rpm,
                feed_per_tooth=fz,
                axial_depth=block.axial_depth_mm,
                radial_depth=block.radial_depth_mm,
            )
            cutting_power_w = forces.get('power_w', 0.0)
            # Estimate spindle motor heat as 5% of cutting power
            spindle_power_w = cutting_power_w * 0.05

        return self._thermal_model.update(
            dt_sec=dt_sec,
            cutting_power_w=cutting_power_w,
            spindle_power_w=spindle_power_w,
            coolant_type=self._coolant.coolant_type,
        )

    def get_coolant_recommendation(
        self,
        block: 'GCodeBlock',
        tool_state: 'ToolState',
        material: str = '6061-T6',
    ) -> 'CoolantRecommendation':
        """Get a coolant recommendation for the given block and tool state.

        Uses CoolantOptimizer to analyze cutting conditions and recommend
        the optimal coolant strategy.

        Args:
            block: The G-code block describing the current cut.
            tool_state: Current tool wear/geometry state.
            material: Workpiece material identifier.

        Returns:
            CoolantRecommendation with the best coolant option.
        """
        optimizer = CoolantOptimizer(material=material)

        # Derive cutting speed (m/min) from spindle RPM and tool diameter
        cutting_speed = (math.pi * tool_state.diameter_mm * block.spindle_rpm) / 1000.0

        # Estimate temperature via force calculation + thermal model
        current_temp = 0.0
        if block.spindle_rpm > 0 and block.feed_rate_mmpm > 0:
            fz = block.feed_rate_mmpm / (block.spindle_rpm * tool_state.flute_count)
            forces = self.calculate_forces(
                spindle_rpm=block.spindle_rpm,
                feed_per_tooth=fz,
                axial_depth=block.axial_depth_mm,
                radial_depth=block.radial_depth_mm,
                tool_diameter=tool_state.diameter_mm,
                flute_count=tool_state.flute_count,
                helix_angle_rad=math.radians(tool_state.helix_angle_deg),
                flank_wear_vb=tool_state.flank_wear_vb,
            )
            current_temp = forces['power_w'] * 0.05 * self._coolant.thermal_factor

        # Estimate wear rate from tool state
        if tool_state.cutting_time_min > 0:
            wear_rate = tool_state.flank_wear_vb / tool_state.cutting_time_min
        else:
            wear_rate = 0.0

        # Determine operation type from depth ratio
        if tool_state.diameter_mm > 0:
            depth_ratio = block.axial_depth_mm / tool_state.diameter_mm
        else:
            depth_ratio = 0.0
        if depth_ratio > 0.5:
            operation_type = 'roughing'
        elif depth_ratio < 0.15:
            operation_type = 'finishing'
        else:
            operation_type = 'general'

        return optimizer.recommend_coolant(
            operation_type=operation_type,
            cutting_speed=cutting_speed,
            depth=block.axial_depth_mm,
            current_temp=current_temp,
            current_wear_rate=wear_rate,
            current_coolant=self._coolant.coolant_type,
        )

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

    def get_chip_load_analysis(
        self,
        block: 'GCodeBlock',
        tool_state: Optional['ToolState'] = None,
    ) -> 'ChipLoadMetrics':
        """Analyze chip load for a G-code block.

        Creates a ChipLoadMonitor using the current tool state (or defaults)
        and computes chip load metrics for the block's cutting conditions.

        Args:
            block: G-code block with feed, RPM, depths.
            tool_state: Optional tool state for diameter/flute info.

        Returns:
            ChipLoadMetrics with chip load analysis.
        """
        tool = tool_state or ToolState()
        monitor = ChipLoadMonitor(
            material='6061-T6',
            num_flutes=tool.flute_count,
            tool_diameter_mm=tool.diameter_mm,
        )
        return monitor.compute_chip_load(
            feed_rate_mmpm=block.feed_rate_mmpm,
            rpm=block.spindle_rpm,
            ae_mm=block.radial_depth_mm,
            ap_mm=block.axial_depth_mm,
        )

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


# ---------------------------------------------------------------------------
# Adaptive Feed Override Controller
# ---------------------------------------------------------------------------

class FeedControlMode(Enum):
    """Control modes for adaptive feed override."""
    FORCE_CONSTANT = 'force_constant'
    POWER_CONSTANT = 'power_constant'
    MRR_CONSTANT = 'mrr_constant'


@dataclass
class FeedControlState:
    """Internal state of the PID controller."""
    current_override_pct: float = 100.0
    previous_error: float = 0.0
    integral_error: float = 0.0
    total_adjustments: int = 0
    mode: FeedControlMode = FeedControlMode.FORCE_CONSTANT
    target_value: float = 500.0  # target force N, power W, or MRR mm³/min
    timestamp: float = field(default_factory=time.time)


@dataclass
class FeedOverrideReport:
    """Summary report of feed override controller performance."""
    current_override: float
    average_override: float
    min_override: float
    max_override: float
    mode: str
    total_adjustments: int
    override_histogram: Dict[str, int] = field(default_factory=dict)


class AdaptiveFeedController:
    """PID-based adaptive feed override controller.

    Monitors cutting forces/power/MRR and adjusts the feed override percentage
    to maintain a target value.

    Parameters:
        kp: Proportional gain (default 0.5)
        ki: Integral gain (default 0.05)
        kd: Derivative gain (default 0.1)
        min_override: Minimum override percentage (default 10%)
        max_override: Maximum override percentage (default 150%)
        max_rate: Maximum override change per cycle (default 5%)
    """

    def __init__(self, kp: float = 0.5, ki: float = 0.05, kd: float = 0.1,
                 min_override: float = 10.0, max_override: float = 150.0,
                 max_rate: float = 5.0) -> None:
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._min_override = min_override
        self._max_override = max_override
        self._max_rate = max_rate
        self._state = FeedControlState()
        self._history: List[Tuple[float, float]] = []  # (timestamp, override_pct)

    @property
    def current_override(self) -> float:
        return self._state.current_override_pct

    @property
    def mode(self) -> FeedControlMode:
        return self._state.mode

    def set_mode(self, mode: FeedControlMode, target_value: float) -> None:
        """Set control mode and target value."""
        self._state.mode = mode
        self._state.target_value = target_value
        self._state.previous_error = 0.0
        self._state.integral_error = 0.0

    def reset(self) -> None:
        """Reset controller to defaults."""
        self._state = FeedControlState()
        self._history.clear()

    def update(self, measured_value: float) -> float:
        """Update the controller with a new measurement.

        Args:
            measured_value: Current measured force (N), power (W), or MRR (mm³/min)
                depending on the active control mode.

        Returns:
            The new feed override percentage.
        """
        target = self._state.target_value
        if target <= 0:
            return self._state.current_override_pct

        # Error: positive means we're below target (need more feed)
        # negative means above target (need less feed)
        error = (target - measured_value) / target * 100.0  # normalised %

        # PID calculation
        self._state.integral_error += error
        # Anti-windup: clamp integral
        self._state.integral_error = max(-200.0, min(200.0, self._state.integral_error))

        derivative = error - self._state.previous_error
        self._state.previous_error = error

        adjustment = (
            self._kp * error +
            self._ki * self._state.integral_error +
            self._kd * derivative
        )

        # Rate limiting
        adjustment = max(-self._max_rate, min(self._max_rate, adjustment))

        # Apply adjustment
        new_override = self._state.current_override_pct + adjustment

        # Safety clamping
        new_override = max(self._min_override, min(self._max_override, new_override))

        self._state.current_override_pct = new_override
        self._state.total_adjustments += 1
        self._state.timestamp = time.time()

        self._history.append((self._state.timestamp, new_override))

        return new_override

    def report(self) -> FeedOverrideReport:
        """Generate a report of controller performance."""
        if not self._history:
            return FeedOverrideReport(
                current_override=self._state.current_override_pct,
                average_override=self._state.current_override_pct,
                min_override=self._state.current_override_pct,
                max_override=self._state.current_override_pct,
                mode=self._state.mode.value,
                total_adjustments=0,
            )

        overrides = [o for _, o in self._history]
        avg = sum(overrides) / len(overrides)

        # Build histogram in 10% buckets
        histogram: Dict[str, int] = {}
        for o in overrides:
            bucket = int(o // 10) * 10
            key = f'{bucket}-{bucket + 10}%'
            histogram[key] = histogram.get(key, 0) + 1

        return FeedOverrideReport(
            current_override=self._state.current_override_pct,
            average_override=round(avg, 2),
            min_override=round(min(overrides), 2),
            max_override=round(max(overrides), 2),
            mode=self._state.mode.value,
            total_adjustments=self._state.total_adjustments,
            override_histogram=histogram,
        )


# ---------------------------------------------------------------------------
# Spindle Warmup Manager
# ---------------------------------------------------------------------------

@dataclass
class WarmupStage:
    """A single stage in a spindle warmup sequence."""
    rpm: float
    duration_min: float
    description: str


@dataclass
class WarmupProfile:
    """Complete warmup profile for a spindle warmup sequence."""
    stages: List[WarmupStage]
    total_duration_min: float
    target_rpm: float
    machine_id: str = 'default'


@dataclass
class WarmupStatus:
    """Current status of a warmup sequence in progress."""
    current_stage_idx: int
    elapsed_min: float
    current_rpm: float
    thermal_stability_pct: float  # 0-100, 100 = fully stable
    is_complete: bool


class SpindleWarmupManager:
    """Manages spindle warmup sequences to prevent thermal shock and ensure
    dimensional stability.

    Spindle bearings and the spindle shaft expand as they reach operating
    temperature.  Running a controlled warmup sequence reduces thermal
    gradients, prevents premature bearing wear, and improves dimensional
    accuracy of the first cuts.

    Pre-built profiles:
        COLD_START   — 5 stages, 500 -> 1000 -> 2000 -> 4000 -> target
        WARM_RESTART — 2 stages, quick ramp for machines idle < 2 h
        HIGH_SPEED   — 6 stages, gradual ramp to 24 000 rpm
    """

    # Thresholds (hours since last run) for profile selection
    COLD_THRESHOLD_HOURS: float = 4.0
    WARM_THRESHOLD_HOURS: float = 2.0

    # Thermal stability evaluation constants
    TEMP_CONVERGENCE_TOLERANCE_C: float = 2.0  # degrees C
    MIN_STABILITY_PCT: float = 0.0
    MAX_STABILITY_PCT: float = 100.0

    # --------------- pre-built profiles ---------------

    @staticmethod
    def _cold_start_profile(target_rpm: float, machine_id: str = 'default') -> WarmupProfile:
        """5-stage warmup for a cold machine (idle > 4 h)."""
        stages = [
            WarmupStage(rpm=500, duration_min=3.0,
                        description='Initial bearing lubrication at low speed'),
            WarmupStage(rpm=1000, duration_min=3.0,
                        description='Gentle thermal soak — inner race warming'),
            WarmupStage(rpm=2000, duration_min=4.0,
                        description='Mid-speed stabilisation'),
            WarmupStage(rpm=4000, duration_min=4.0,
                        description='High-speed bearing preload settling'),
            WarmupStage(rpm=target_rpm, duration_min=6.0,
                        description='Final target RPM — thermal equilibrium'),
        ]
        total = sum(s.duration_min for s in stages)
        return WarmupProfile(
            stages=stages,
            total_duration_min=total,
            target_rpm=target_rpm,
            machine_id=machine_id,
        )

    @staticmethod
    def _warm_restart_profile(target_rpm: float, machine_id: str = 'default') -> WarmupProfile:
        """2-stage warmup for a warm machine (idle < 2 h)."""
        mid_rpm = min(target_rpm * 0.5, 4000.0)
        stages = [
            WarmupStage(rpm=mid_rpm, duration_min=2.0,
                        description='Quick mid-speed check'),
            WarmupStage(rpm=target_rpm, duration_min=3.0,
                        description='Ramp to target RPM'),
        ]
        total = sum(s.duration_min for s in stages)
        return WarmupProfile(
            stages=stages,
            total_duration_min=total,
            target_rpm=target_rpm,
            machine_id=machine_id,
        )

    @staticmethod
    def _high_speed_profile(machine_id: str = 'default') -> WarmupProfile:
        """6-stage warmup for high-speed spindles (up to 24 000 rpm)."""
        stages = [
            WarmupStage(rpm=500, duration_min=2.0,
                        description='Lubrication distribution'),
            WarmupStage(rpm=2000, duration_min=3.0,
                        description='Low-speed thermal soak'),
            WarmupStage(rpm=6000, duration_min=4.0,
                        description='Mid-range bearing warmup'),
            WarmupStage(rpm=12000, duration_min=5.0,
                        description='High-speed preload adjustment zone'),
            WarmupStage(rpm=18000, duration_min=5.0,
                        description='Near-max thermal stabilisation'),
            WarmupStage(rpm=24000, duration_min=6.0,
                        description='Full speed — final equilibrium'),
        ]
        total = sum(s.duration_min for s in stages)
        return WarmupProfile(
            stages=stages,
            total_duration_min=total,
            target_rpm=24000.0,
            machine_id=machine_id,
        )

    # --------------- public API ---------------

    def generate_profile(
        self,
        target_rpm: float,
        machine_state: str = 'cold',
        machine_id: str = 'default',
    ) -> WarmupProfile:
        """Create an appropriate warmup profile based on target speed and
        current thermal state.

        Args:
            target_rpm: Desired operating spindle speed.
            machine_state: One of 'cold', 'warm', or 'hot'.
            machine_id: Identifier for the machine.

        Returns:
            WarmupProfile tailored to the current conditions.
        """
        if target_rpm >= 20000:
            return self._high_speed_profile(machine_id=machine_id)

        if machine_state == 'warm':
            return self._warm_restart_profile(target_rpm, machine_id=machine_id)

        if machine_state == 'hot':
            # Hot machine: single stage, short dwell at target
            stages = [
                WarmupStage(rpm=target_rpm, duration_min=2.0,
                            description='Verification run at target RPM'),
            ]
            return WarmupProfile(
                stages=stages,
                total_duration_min=2.0,
                target_rpm=target_rpm,
                machine_id=machine_id,
            )

        # Default: cold start
        return self._cold_start_profile(target_rpm, machine_id=machine_id)

    def evaluate_stability(
        self,
        bearing_temps: List[float],
        spindle_temps: List[float],
    ) -> float:
        """Evaluate thermal stability based on temperature convergence.

        Stability is 100% when all bearing and spindle temperatures are
        within ``TEMP_CONVERGENCE_TOLERANCE_C`` of each other, and
        decreases linearly with the maximum spread.

        Args:
            bearing_temps: List of bearing temperature readings (C).
            spindle_temps: List of spindle shaft temperature readings (C).

        Returns:
            thermal_stability_pct in range [0, 100].
        """
        all_temps = bearing_temps + spindle_temps
        if not all_temps:
            return self.MIN_STABILITY_PCT

        temp_min = min(all_temps)
        temp_max = max(all_temps)
        spread = temp_max - temp_min

        if spread <= self.TEMP_CONVERGENCE_TOLERANCE_C:
            return self.MAX_STABILITY_PCT

        # Linear decay: 100 % at tolerance, 0 % at 10x tolerance
        max_spread = self.TEMP_CONVERGENCE_TOLERANCE_C * 10.0
        stability = max(
            self.MIN_STABILITY_PCT,
            self.MAX_STABILITY_PCT * (1.0 - (spread - self.TEMP_CONVERGENCE_TOLERANCE_C)
                                      / (max_spread - self.TEMP_CONVERGENCE_TOLERANCE_C)),
        )
        return round(stability, 2)

    def recommend_warmup(
        self,
        target_rpm: float,
        hours_since_last_run: float,
    ) -> Tuple[bool, str]:
        """Recommend whether a warmup cycle is needed.

        Args:
            target_rpm: Desired operating RPM.
            hours_since_last_run: Hours since the spindle was last run.

        Returns:
            Tuple of (warmup_recommended: bool, reason: str).
        """
        if hours_since_last_run >= self.COLD_THRESHOLD_HOURS:
            return (
                True,
                f"Machine has been idle for {hours_since_last_run:.1f} h "
                f"(>= {self.COLD_THRESHOLD_HOURS} h). "
                "Full cold-start warmup recommended to avoid thermal shock.",
            )

        if hours_since_last_run >= self.WARM_THRESHOLD_HOURS:
            return (
                True,
                f"Machine has been idle for {hours_since_last_run:.1f} h "
                f"(>= {self.WARM_THRESHOLD_HOURS} h). "
                "Short warm-restart warmup recommended.",
            )

        if target_rpm >= 20000:
            return (
                True,
                f"Target RPM ({target_rpm}) is in the high-speed range. "
                "High-speed warmup recommended regardless of idle time.",
            )

        return (
            False,
            f"Machine was recently active ({hours_since_last_run:.1f} h ago) "
            f"and target RPM ({target_rpm}) is moderate. No warmup needed.",
        )

    def get_status(
        self,
        profile: WarmupProfile,
        elapsed_min: float,
        bearing_temps: Optional[List[float]] = None,
        spindle_temps: Optional[List[float]] = None,
    ) -> WarmupStatus:
        """Compute the current warmup status given elapsed time.

        Args:
            profile: The active warmup profile.
            elapsed_min: Minutes elapsed since warmup started.
            bearing_temps: Optional current bearing temps for stability calc.
            spindle_temps: Optional current spindle temps for stability calc.

        Returns:
            WarmupStatus snapshot.
        """
        cumulative = 0.0
        current_stage_idx = len(profile.stages) - 1
        current_rpm = profile.target_rpm

        for i, stage in enumerate(profile.stages):
            cumulative += stage.duration_min
            if elapsed_min < cumulative:
                current_stage_idx = i
                current_rpm = stage.rpm
                break

        is_complete = elapsed_min >= profile.total_duration_min

        if bearing_temps and spindle_temps:
            stability = self.evaluate_stability(bearing_temps, spindle_temps)
        else:
            # Estimate stability from elapsed fraction
            fraction = min(1.0, elapsed_min / max(profile.total_duration_min, 1e-9))
            stability = round(fraction * 100.0, 2)

        return WarmupStatus(
            current_stage_idx=current_stage_idx,
            elapsed_min=elapsed_min,
            current_rpm=current_rpm,
            thermal_stability_pct=stability,
            is_complete=is_complete,
        )
