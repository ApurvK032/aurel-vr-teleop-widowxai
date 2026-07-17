from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter, perf_counter_ns, sleep

from .types import QuestSample


def wait_for_cycle_period(cycle_started_s: float, period_s: float, *, clock=perf_counter, sleeper=sleep) -> None:
    """Wait until one full cycle has elapsed, without scheduling catch-up bursts."""
    if period_s <= 0.0:
        raise ValueError("period_s must be positive")
    deadline = float(cycle_started_s) + float(period_s)
    while (remaining := deadline - clock()) > 1e-9:
        sleeper(remaining)


@dataclass(frozen=True)
class FreshnessStatus:
    fresh: bool
    unique_samples: int
    repeated_samples: int
    age_s: float
    reconnect_generation: int


class FreshSequenceWatchdog:
    def __init__(self, timeout_s: float, fresh_samples_to_recover: int = 3) -> None:
        self.timeout_ns = int(float(timeout_s) * 1e9)
        self.fresh_samples_to_recover = max(1, int(fresh_samples_to_recover))
        self.last_sequence: int | None = None
        self.last_unique_arrival_ns: int | None = None
        self.last_reconnect_generation = -1
        self.unique_samples = 0
        self.repeated_samples = 0
        self._recovery_streak = 0
        self._fresh = False

    def observe(self, sample: QuestSample) -> FreshnessStatus:
        generation_changed = sample.reconnect_generation != self.last_reconnect_generation
        unique = generation_changed or sample.sequence != self.last_sequence
        if generation_changed:
            self._fresh = False
            self._recovery_streak = 0
            self.last_reconnect_generation = sample.reconnect_generation
        if unique:
            self.unique_samples += 1
            self._recovery_streak += 1
            self.last_sequence = sample.sequence
            self.last_unique_arrival_ns = sample.pc_arrival_monotonic_ns
            if self._recovery_streak >= self.fresh_samples_to_recover:
                self._fresh = True
        else:
            self.repeated_samples += 1
        return self.poll(sample.pc_arrival_monotonic_ns)

    def poll(self, now_ns: int | None = None) -> FreshnessStatus:
        now = perf_counter_ns() if now_ns is None else int(now_ns)
        age_ns = self.timeout_ns + 1 if self.last_unique_arrival_ns is None else now - self.last_unique_arrival_ns
        if age_ns > self.timeout_ns:
            self._fresh = False
            self._recovery_streak = 0
        return FreshnessStatus(
            fresh=self._fresh,
            unique_samples=self.unique_samples,
            repeated_samples=self.repeated_samples,
            age_s=max(0.0, age_ns / 1e9),
            reconnect_generation=self.last_reconnect_generation,
        )
