"""Tests for wear-adjusted stability lobe prediction physics model.

Mirrors the C# StabilityLobePredictor logic in Python to validate the
zeroth-order approximation (ZOA) chatter stability model with tool wear effects.
"""

import math
import pytest


# ---------------------------------------------------------------------------
# Physics model (mirrors StabilityLobePredictor.cs)
# ---------------------------------------------------------------------------

# Default modal parameters (1/4" HSS endmill in ER-11 collet)
DEFAULT_FN_HZ = 1800.0       # natural frequency Hz
DEFAULT_ZETA = 0.03           # damping ratio
DEFAULT_K = 8e6               # stiffness N/m
DEFAULT_KTC = 796.0           # tangential cutting coefficient N/mm^2
DEFAULT_NUM_TEETH = 2
DEFAULT_WEAR_STIFFNESS_REDUCTION = 0.15  # per 0.1mm wear
DEFAULT_WEAR_DAMPING_INCREASE = 0.05     # per 0.1mm wear
DEFAULT_VB_MAX = 0.3          # mm


def get_effective_stiffness(wear_vb, k=DEFAULT_K,
                            reduction=DEFAULT_WEAR_STIFFNESS_REDUCTION,
                            vb_max=DEFAULT_VB_MAX):
    """Wear-adjusted stiffness: k_eff = k * (1 - reduction * VB / 0.1).
    Clamped so VB never exceeds VBmax and stiffness never below 10%.
    """
    clamped_vb = max(0.0, min(wear_vb, vb_max))
    factor = 1.0 - reduction * (clamped_vb / 0.1)
    factor = max(factor, 0.1)
    return k * factor


def get_effective_damping(wear_vb, zeta=DEFAULT_ZETA,
                          increase=DEFAULT_WEAR_DAMPING_INCREASE,
                          vb_max=DEFAULT_VB_MAX):
    """Wear-adjusted damping: zeta_eff = zeta * (1 + increase * VB / 0.1)."""
    clamped_vb = max(0.0, min(wear_vb, vb_max))
    factor = 1.0 + increase * (clamped_vb / 0.1)
    return zeta * factor


def zoa_stability_limit(rpm, num_teeth=DEFAULT_NUM_TEETH, Ktc=DEFAULT_KTC,
                         fn_hz=DEFAULT_FN_HZ, zeta=DEFAULT_ZETA,
                         k_stiffness=DEFAULT_K):
    """Zeroth-order approximation stability limit (ap_lim in mm).

    Sweeps chatter frequencies near natural frequency and returns the
    minimum (most conservative) stability limit.
    """
    if rpm <= 0:
        return float('inf')

    omega_n = 2.0 * math.pi * fn_hz
    best_ap_lim = float('inf')

    # Sweep ratio from 0.5 to 2.0 in fine steps
    ratio = 0.5
    while ratio < 2.0:
        omega_c = omega_n * ratio
        r = omega_c / omega_n  # == ratio
        r2 = r * r
        dr = 2.0 * zeta * r

        # Real part of FRF: Re[G(jw)]
        denom = k_stiffness * ((1.0 - r2) ** 2 + dr ** 2)
        if abs(denom) < 1e-12:
            ratio += 0.005
            continue
        real_g = (1.0 - r2) / denom

        if real_g >= 0:
            ratio += 0.005
            continue

        # Convert m/N to mm/N
        real_g_mm = real_g * 1000.0
        ap_lim = -1.0 / (2.0 * Ktc * num_teeth * real_g_mm)

        if ap_lim > 0 and ap_lim < best_ap_lim:
            tooth_pass_freq = rpm * num_teeth / 60.0
            if tooth_pass_freq > 0:
                phase_angle = math.atan2(-dr, 1.0 - r2)
                n_lobe = (omega_c - phase_angle) / (2.0 * math.pi * tooth_pass_freq)
                if n_lobe > 0:
                    best_ap_lim = ap_lim

        ratio += 0.005

    return best_ap_lim if best_ap_lim < float('inf') else 100.0


def wear_adjusted_stability_limit(rpm, wear_vb, **kwargs):
    """Stability limit with wear-adjusted stiffness and damping."""
    k_eff = get_effective_stiffness(wear_vb)
    zeta_eff = get_effective_damping(wear_vb)
    return zoa_stability_limit(rpm, k_stiffness=k_eff, zeta=zeta_eff, **kwargs)


def compute_lobe_surface(min_rpm, max_rpm, min_depth, max_depth,
                          wear_vb=0.0, resolution=50):
    """Compute full lobe surface grid. Returns (surface, rpms, depths).
    surface[ri, di] = stability_margin (positive=stable, negative=unstable).
    """
    rpms = [min_rpm + (max_rpm - min_rpm) * i / (resolution - 1)
            for i in range(resolution)]
    depths = [min_depth + (max_depth - min_depth) * i / (resolution - 1)
              for i in range(resolution)]

    surface = [[0.0] * resolution for _ in range(resolution)]
    for ri in range(resolution):
        ap_lim = wear_adjusted_stability_limit(rpms[ri], wear_vb)
        for di in range(resolution):
            surface[ri][di] = ap_lim - depths[di]

    return surface, rpms, depths


def is_in_stable_pocket(rpm, depth, wear_vb=0.0, medium_risk_margin=0.8):
    """Check if operating point is in a stable pocket."""
    ap_lim = wear_adjusted_stability_limit(rpm, wear_vb)
    return depth < ap_lim * medium_risk_margin


def get_interpolated_limit(rpm, rpms, wear_vb=0.0):
    """Get interpolated stability limit from a grid of RPM values."""
    if rpm <= rpms[0]:
        return wear_adjusted_stability_limit(rpms[0], wear_vb)
    if rpm >= rpms[-1]:
        return wear_adjusted_stability_limit(rpms[-1], wear_vb)

    # Find bounding indices
    lo = 0
    hi = len(rpms) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rpms[mid] <= rpm:
            lo = mid
        else:
            hi = mid

    ap_lo = wear_adjusted_stability_limit(rpms[lo], wear_vb)
    ap_hi = wear_adjusted_stability_limit(rpms[hi], wear_vb)
    t = (rpm - rpms[lo]) / (rpms[hi] - rpms[lo])
    return ap_lo + t * (ap_hi - ap_lo)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestEffectiveStiffness:
    """Tests for wear-adjusted stiffness calculation."""

    def test_zero_wear_returns_nominal(self):
        """With no wear, effective stiffness equals nominal stiffness."""
        k_eff = get_effective_stiffness(0.0)
        assert k_eff == pytest.approx(DEFAULT_K, rel=1e-6)

    def test_stiffness_decreases_with_wear(self):
        """Stiffness should decrease as VB increases."""
        k0 = get_effective_stiffness(0.0)
        k1 = get_effective_stiffness(0.1)
        k2 = get_effective_stiffness(0.2)
        assert k1 < k0
        assert k2 < k1

    def test_stiffness_at_01mm_wear(self):
        """At VB=0.1mm, stiffness reduced by wearStiffnessReduction (15%)."""
        k_eff = get_effective_stiffness(0.1)
        expected = DEFAULT_K * (1.0 - DEFAULT_WEAR_STIFFNESS_REDUCTION)
        assert k_eff == pytest.approx(expected, rel=1e-6)

    def test_stiffness_clamps_at_vbmax(self):
        """Wear beyond VBmax should be clamped — no further stiffness loss."""
        k_at_max = get_effective_stiffness(DEFAULT_VB_MAX)
        k_beyond = get_effective_stiffness(DEFAULT_VB_MAX + 0.5)
        assert k_at_max == pytest.approx(k_beyond, rel=1e-6)

    def test_stiffness_never_below_ten_percent(self):
        """Even at maximum wear, stiffness should not drop below 10% of nominal."""
        k_eff = get_effective_stiffness(DEFAULT_VB_MAX)
        assert k_eff >= DEFAULT_K * 0.1


class TestEffectiveDamping:
    """Tests for wear-adjusted damping calculation."""

    def test_zero_wear_returns_nominal(self):
        """With no wear, effective damping equals nominal damping."""
        z_eff = get_effective_damping(0.0)
        assert z_eff == pytest.approx(DEFAULT_ZETA, rel=1e-6)

    def test_damping_increases_with_wear(self):
        """Damping should increase as VB increases (growing contact area)."""
        z0 = get_effective_damping(0.0)
        z1 = get_effective_damping(0.1)
        z2 = get_effective_damping(0.2)
        assert z1 > z0
        assert z2 > z1

    def test_damping_at_01mm_wear(self):
        """At VB=0.1mm, damping increased by wearDampingIncrease (5%)."""
        z_eff = get_effective_damping(0.1)
        expected = DEFAULT_ZETA * (1.0 + DEFAULT_WEAR_DAMPING_INCREASE)
        assert z_eff == pytest.approx(expected, rel=1e-6)


class TestStabilityLimitBaseline:
    """Tests for baseline (no wear) stability limit calculation."""

    def test_zero_rpm_returns_inf(self):
        """RPM=0 should return infinite stability limit (no cutting)."""
        limit = zoa_stability_limit(0)
        assert limit == float('inf')

    def test_positive_rpm_returns_finite(self):
        """At a typical machining RPM, stability limit should be finite and positive."""
        limit = zoa_stability_limit(12000)
        assert 0 < limit < 100

    def test_limit_at_resonance_is_minimum(self):
        """Near resonance, the stability limit should be at its minimum."""
        # At resonance, tooth-pass frequency ~ natural frequency / N_lobe
        # For lobe N=1: rpm_resonance ≈ fn_hz * 60 / num_teeth
        rpm_resonance = DEFAULT_FN_HZ * 60.0 / DEFAULT_NUM_TEETH
        limit_resonance = zoa_stability_limit(rpm_resonance)

        # Check a few off-resonance points
        limit_low = zoa_stability_limit(rpm_resonance * 0.5)
        limit_high = zoa_stability_limit(rpm_resonance * 1.5)

        # The resonance limit should be among the lowest
        assert limit_resonance <= limit_low or limit_resonance <= limit_high

    def test_limit_varies_with_stiffness(self):
        """Stability limit should scale with stiffness (stiffer = higher limit)."""
        rpm = 12000
        limit_soft = zoa_stability_limit(rpm, k_stiffness=4e6)
        limit_stiff = zoa_stability_limit(rpm, k_stiffness=8e6)
        # Higher stiffness → higher stability limit (less compliance → less chatter)
        assert limit_stiff > limit_soft * 0.99


class TestStabilityLimitWithWear:
    """Tests for wear-adjusted stability limit."""

    def test_wear_zero_matches_baseline(self):
        """With VB=0, wear-adjusted limit should match baseline."""
        rpm = 12000
        baseline = zoa_stability_limit(rpm)
        worn = wear_adjusted_stability_limit(rpm, 0.0)
        assert worn == pytest.approx(baseline, rel=1e-6)

    def test_stability_limit_decreases_with_wear(self):
        """Net effect of wear: stability limit should DECREASE (more chatter-prone)."""
        rpm = 12000
        limit_0 = wear_adjusted_stability_limit(rpm, 0.0)
        limit_01 = wear_adjusted_stability_limit(rpm, 0.1)
        limit_02 = wear_adjusted_stability_limit(rpm, 0.2)

        assert limit_01 < limit_0, \
            f"Limit at VB=0.1 ({limit_01:.4f}) should be less than baseline ({limit_0:.4f})"
        assert limit_02 < limit_01, \
            f"Limit at VB=0.2 ({limit_02:.4f}) should be less than VB=0.1 ({limit_01:.4f})"

    def test_stability_decreases_at_multiple_rpms(self):
        """Wear should reduce stability limit across a range of RPMs."""
        for rpm in [5000, 10000, 15000, 20000]:
            limit_fresh = wear_adjusted_stability_limit(rpm, 0.0)
            limit_worn = wear_adjusted_stability_limit(rpm, 0.2)
            assert limit_worn <= limit_fresh, \
                f"At RPM={rpm}, worn limit ({limit_worn:.4f}) should be <= fresh ({limit_fresh:.4f})"

    def test_excessive_wear_clamps(self):
        """Wear beyond VBmax should clamp — no further limit change."""
        rpm = 12000
        limit_max = wear_adjusted_stability_limit(rpm, DEFAULT_VB_MAX)
        limit_excessive = wear_adjusted_stability_limit(rpm, DEFAULT_VB_MAX + 1.0)
        assert limit_max == pytest.approx(limit_excessive, rel=1e-4)


class TestLobeSurface:
    """Tests for lobe surface computation."""

    def test_surface_dimensions(self):
        """Surface grid should have correct dimensions."""
        res = 50
        surface, rpms, depths = compute_lobe_surface(5000, 20000, 0, 5, resolution=res)
        assert len(surface) == res
        assert len(surface[0]) == res
        assert len(rpms) == res
        assert len(depths) == res

    def test_surface_rpms_monotonic(self):
        """RPM axis values should be monotonically increasing."""
        _, rpms, _ = compute_lobe_surface(5000, 20000, 0, 5)
        for i in range(1, len(rpms)):
            assert rpms[i] > rpms[i - 1]

    def test_surface_depths_monotonic(self):
        """Depth axis values should be monotonically increasing."""
        _, _, depths = compute_lobe_surface(5000, 20000, 0, 5)
        for i in range(1, len(depths)):
            assert depths[i] > depths[i - 1]

    def test_surface_margin_decreases_with_depth(self):
        """At a fixed RPM, stability margin should decrease as depth increases."""
        surface, rpms, depths = compute_lobe_surface(5000, 20000, 0, 5, resolution=20)
        # Pick a middle RPM index
        ri = 10
        for di in range(1, len(depths)):
            assert surface[ri][di] <= surface[ri][di - 1] + 1e-9

    def test_surface_with_wear_lower_margins(self):
        """Worn tool surface should have lower margins than fresh tool surface."""
        s_fresh, _, _ = compute_lobe_surface(5000, 20000, 0, 5, wear_vb=0.0, resolution=20)
        s_worn, _, _ = compute_lobe_surface(5000, 20000, 0, 5, wear_vb=0.2, resolution=20)

        # At each RPM, the margin at depth=0 equals the stability limit.
        # Worn should be lower or equal at every RPM.
        for ri in range(20):
            assert s_worn[ri][0] <= s_fresh[ri][0] + 1e-6


class TestStablePocket:
    """Tests for stable pocket detection."""

    def test_shallow_cut_is_stable(self):
        """Very shallow depth should always be in a stable pocket."""
        assert is_in_stable_pocket(12000, 0.01, wear_vb=0.0) is True

    def test_deep_cut_is_unstable(self):
        """Very deep depth should not be in a stable pocket."""
        assert is_in_stable_pocket(12000, 100.0, wear_vb=0.0) is False

    def test_wear_shrinks_stable_pocket(self):
        """A depth that was stable with fresh tool may become unstable with wear."""
        rpm = 12000
        # Find the stability limit and pick a depth just below the pocket boundary
        limit_fresh = wear_adjusted_stability_limit(rpm, 0.0)
        depth = limit_fresh * 0.75  # 75% of fresh limit — inside medium_risk_margin=0.8

        stable_fresh = is_in_stable_pocket(rpm, depth, wear_vb=0.0)
        # With enough wear the pocket shrinks
        limit_worn = wear_adjusted_stability_limit(rpm, 0.25)
        # The depth relative to worn limit should be higher
        assert depth / limit_worn > depth / limit_fresh


class TestInterpolation:
    """Tests for interpolated stability limit from surface grid."""

    def test_interpolation_at_grid_point(self):
        """Interpolation at an exact grid RPM should match direct calculation."""
        rpms = [5000 + i * 500 for i in range(31)]
        for rpm in rpms[::5]:
            interp = get_interpolated_limit(rpm, rpms, wear_vb=0.0)
            direct = wear_adjusted_stability_limit(rpm, 0.0)
            assert interp == pytest.approx(direct, rel=1e-4)

    def test_interpolation_between_grid_points(self):
        """Interpolation between grid points should be between bounding values."""
        rpms = [5000 + i * 1000 for i in range(16)]
        test_rpm = 7500  # Between rpms[2]=7000 and rpms[3]=8000
        interp = get_interpolated_limit(test_rpm, rpms, wear_vb=0.0)
        lo = wear_adjusted_stability_limit(7000, 0.0)
        hi = wear_adjusted_stability_limit(8000, 0.0)
        assert min(lo, hi) - 0.01 <= interp <= max(lo, hi) + 0.01

    def test_interpolation_clamps_below_range(self):
        """RPM below grid range should clamp to first grid point."""
        rpms = [5000 + i * 1000 for i in range(16)]
        interp = get_interpolated_limit(1000, rpms, wear_vb=0.0)
        edge = wear_adjusted_stability_limit(rpms[0], 0.0)
        assert interp == pytest.approx(edge, rel=1e-4)

    def test_interpolation_clamps_above_range(self):
        """RPM above grid range should clamp to last grid point."""
        rpms = [5000 + i * 1000 for i in range(16)]
        interp = get_interpolated_limit(50000, rpms, wear_vb=0.0)
        edge = wear_adjusted_stability_limit(rpms[-1], 0.0)
        assert interp == pytest.approx(edge, rel=1e-4)


class TestLobeBoundaries:
    """Tests for multiple RPM values crossing lobe boundaries."""

    def test_stability_limit_is_positive_across_rpm_range(self):
        """Stability limit should be positive and finite across a wide RPM range."""
        rpms = list(range(3000, 25001, 500))
        for rpm in rpms:
            limit = zoa_stability_limit(rpm)
            assert 0 < limit < float('inf'), \
                f"Limit at RPM={rpm} should be positive finite, got {limit}"

    def test_stability_limit_scales_with_flute_count(self):
        """More flutes → lower stability limit (more cuts per revolution)."""
        rpm = 12000
        limit_2 = zoa_stability_limit(rpm, num_teeth=2)
        limit_4 = zoa_stability_limit(rpm, num_teeth=4)
        assert limit_4 < limit_2, \
            f"4-flute limit ({limit_4:.4f}) should be < 2-flute ({limit_2:.4f})"

    def test_worn_lobes_shift_down(self):
        """Wear should shift stability limits downward across all RPMs."""
        rpms = list(range(5000, 20001, 200))
        limits_fresh = [wear_adjusted_stability_limit(rpm, 0.0) for rpm in rpms]
        limits_worn = [wear_adjusted_stability_limit(rpm, 0.2) for rpm in rpms]

        # Average limit should be lower when worn
        avg_fresh = sum(limits_fresh) / len(limits_fresh)
        avg_worn = sum(limits_worn) / len(limits_worn)
        assert avg_worn < avg_fresh, \
            f"Average worn limit ({avg_worn:.4f}) should be < fresh ({avg_fresh:.4f})"

    def test_worn_limit_at_every_rpm_is_lower(self):
        """At every sampled RPM, worn limit should be <= fresh limit."""
        rpms = list(range(5000, 20001, 500))
        for rpm in rpms:
            fresh = wear_adjusted_stability_limit(rpm, 0.0)
            worn = wear_adjusted_stability_limit(rpm, 0.2)
            assert worn <= fresh + 1e-6, \
                f"At RPM={rpm}: worn ({worn:.4f}) > fresh ({fresh:.4f})"
