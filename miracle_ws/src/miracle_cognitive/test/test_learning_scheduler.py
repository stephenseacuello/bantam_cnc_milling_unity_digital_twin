"""Tests for AdaptiveLearningScheduler in the reasoning engine.

Covers warmup, plateau decay, cosine annealing, reset, history tracking,
best-episode retrieval, and early-stopping logic.
"""

import sys
from unittest.mock import MagicMock

for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_cognitive.knowledge.reasoning_engine import (
    AdaptiveLearningScheduler,
    LearningEpisode,
    SchedulerConfig,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def default_scheduler():
    """Scheduler with default config."""
    return AdaptiveLearningScheduler()


@pytest.fixture
def custom_scheduler():
    """Scheduler with a small warmup and tight patience for fast tests."""
    cfg = SchedulerConfig(
        initial_lr=0.01,
        min_lr=0.0001,
        max_lr=0.1,
        patience=3,
        decay_factor=0.5,
        warmup_episodes=3,
    )
    return AdaptiveLearningScheduler(config=cfg, model_name='test_model')


@pytest.fixture
def cosine_scheduler():
    """Scheduler with cosine annealing enabled."""
    cfg = SchedulerConfig(
        initial_lr=0.01,
        min_lr=0.001,
        warmup_episodes=2,
        patience=100,  # effectively disable plateau
    )
    return AdaptiveLearningScheduler(
        config=cfg,
        model_name='cosine_model',
        use_cosine_annealing=True,
        cosine_cycle_length=10,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestInitialState:
    """Verify the scheduler starts in the expected state."""

    def test_initial_learning_rate(self, default_scheduler):
        assert default_scheduler.get_learning_rate() == 0.01

    def test_empty_history(self, default_scheduler):
        assert default_scheduler.get_history() == []

    def test_best_episode_none_when_empty(self, default_scheduler):
        assert default_scheduler.get_best_episode() is None


class TestWarmup:
    """Verify linear warmup from min_lr to initial_lr."""

    def test_warmup_linear_ramp(self, custom_scheduler):
        cfg = SchedulerConfig(
            initial_lr=0.01, min_lr=0.0001, warmup_episodes=3, patience=3,
            decay_factor=0.5,
        )
        sched = AdaptiveLearningScheduler(config=cfg)

        lr1 = sched.step(loss=1.0)
        # After 1 episode of 3 warmup: progress = 1/3
        expected = 0.0001 + (0.01 - 0.0001) * (1 / 3)
        assert abs(lr1 - expected) < 1e-9

        lr2 = sched.step(loss=0.9)
        expected2 = 0.0001 + (0.01 - 0.0001) * (2 / 3)
        assert abs(lr2 - expected2) < 1e-9

        lr3 = sched.step(loss=0.8)
        expected3 = 0.0001 + (0.01 - 0.0001) * (3 / 3)
        assert abs(lr3 - expected3) < 1e-9
        assert abs(lr3 - 0.01) < 1e-9  # should equal initial_lr

    def test_warmup_does_not_exceed_initial_lr(self, custom_scheduler):
        for _ in range(3):
            custom_scheduler.step(loss=1.0)
        assert custom_scheduler.get_learning_rate() <= 0.01 + 1e-9


class TestPlateauDecay:
    """Verify LR is reduced after patience episodes without improvement."""

    def test_decay_after_patience(self):
        cfg = SchedulerConfig(
            initial_lr=0.01, min_lr=0.0001, patience=3,
            decay_factor=0.5, warmup_episodes=0,
        )
        sched = AdaptiveLearningScheduler(config=cfg)

        # First step sets best_loss
        sched.step(loss=1.0)
        # 3 more steps with non-improving loss (patience = 3)
        sched.step(loss=1.1)
        sched.step(loss=1.2)
        sched.step(loss=1.3)

        # After patience=3 non-improving episodes, LR should have decayed
        lr = sched.get_learning_rate()
        assert lr == pytest.approx(0.01 * 0.5, abs=1e-9)

    def test_no_decay_if_improving(self):
        cfg = SchedulerConfig(
            initial_lr=0.01, min_lr=0.0001, patience=3,
            decay_factor=0.5, warmup_episodes=0,
        )
        sched = AdaptiveLearningScheduler(config=cfg)

        # Continuously improving
        for loss in [1.0, 0.9, 0.8, 0.7, 0.6, 0.5]:
            sched.step(loss=loss)

        assert sched.get_learning_rate() == pytest.approx(0.01, abs=1e-9)

    def test_lr_does_not_drop_below_min(self):
        cfg = SchedulerConfig(
            initial_lr=0.01, min_lr=0.001, patience=1,
            decay_factor=0.1, warmup_episodes=0,
        )
        sched = AdaptiveLearningScheduler(config=cfg)

        # Many non-improving steps to push LR down
        for _ in range(50):
            sched.step(loss=1.0)

        assert sched.get_learning_rate() >= cfg.min_lr


class TestCosineAnnealing:
    """Verify cosine annealing oscillation."""

    def test_cosine_varies_lr(self, cosine_scheduler):
        # Warmup first
        cosine_scheduler.step(loss=1.0)
        cosine_scheduler.step(loss=0.9)

        # Collect LRs over a cosine cycle (post-warmup)
        lrs = []
        for i in range(10):
            cosine_scheduler.step(loss=0.8)  # stable loss, no plateau decay
            lrs.append(cosine_scheduler.get_learning_rate())

        # LR should not be constant (cosine modulation)
        assert len(set(round(lr, 10) for lr in lrs)) > 1

    def test_cosine_stays_above_min(self, cosine_scheduler):
        for _ in range(30):
            cosine_scheduler.step(loss=0.5)
        assert cosine_scheduler.get_learning_rate() >= 0.001


class TestReset:
    """Verify reset restores initial state."""

    def test_reset_clears_history(self, custom_scheduler):
        for i in range(5):
            custom_scheduler.step(loss=float(i))

        assert len(custom_scheduler.get_history()) == 5

        custom_scheduler.reset()

        assert custom_scheduler.get_history() == []
        assert custom_scheduler.get_learning_rate() == 0.01
        assert custom_scheduler.get_best_episode() is None


class TestHistory:
    """Verify episode history tracking."""

    def test_history_records_all_episodes(self, default_scheduler):
        for i in range(4):
            default_scheduler.step(loss=float(i), metric_value=float(i * 10))

        history = default_scheduler.get_history()
        assert len(history) == 4
        assert all(isinstance(ep, LearningEpisode) for ep in history)
        assert history[0].episode_id == 0
        assert history[3].episode_id == 3

    def test_history_stores_model_name(self):
        sched = AdaptiveLearningScheduler(model_name='spindle_model')
        sched.step(loss=0.5, metric_value=0.9)
        assert sched.get_history()[0].model_name == 'spindle_model'


class TestBestEpisode:
    """Verify best-episode retrieval."""

    def test_best_episode_lowest_loss(self, custom_scheduler):
        losses = [0.5, 0.3, 0.8, 0.2, 0.6]
        for loss in losses:
            custom_scheduler.step(loss=loss, metric_value=0.0)

        best = custom_scheduler.get_best_episode()
        assert best is not None
        assert best.loss == pytest.approx(0.2)
        assert best.episode_id == 3  # 0-indexed, fourth step


class TestEarlyStopping:
    """Verify early stopping logic."""

    def test_no_early_stop_below_min_episodes(self):
        cfg = SchedulerConfig(warmup_episodes=0, patience=2)
        sched = AdaptiveLearningScheduler(config=cfg)

        for _ in range(5):
            sched.step(loss=1.0)

        # Only 5 episodes < 20 (default min_episodes)
        assert sched.should_stop_early(min_episodes=20, no_improve_limit=3) is False

    def test_early_stop_triggered(self):
        cfg = SchedulerConfig(
            warmup_episodes=0, patience=100,  # large patience so plateau doesn't reset counter
        )
        sched = AdaptiveLearningScheduler(config=cfg)

        # One improving step, then many non-improving
        sched.step(loss=0.1)
        for _ in range(25):
            sched.step(loss=0.5)

        # 26 episodes total >= 10, and 25 without improvement >= 10
        assert sched.should_stop_early(min_episodes=10, no_improve_limit=10) is True

    def test_early_stop_not_triggered_with_improvement(self):
        cfg = SchedulerConfig(warmup_episodes=0, patience=100)
        sched = AdaptiveLearningScheduler(config=cfg)

        # Steady improvement
        for i in range(30):
            sched.step(loss=1.0 - i * 0.01)

        assert sched.should_stop_early(min_episodes=10, no_improve_limit=5) is False


class TestSchedulerConfig:
    """Verify SchedulerConfig defaults."""

    def test_defaults(self):
        cfg = SchedulerConfig()
        assert cfg.initial_lr == 0.01
        assert cfg.min_lr == 0.0001
        assert cfg.max_lr == 0.1
        assert cfg.patience == 5
        assert cfg.decay_factor == 0.5
        assert cfg.warmup_episodes == 10
