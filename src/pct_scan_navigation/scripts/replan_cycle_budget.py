#!/usr/bin/env python3
"""Bound SCAN-triggered global-replan handoffs for one user goal."""


class ReplanCycleBudget:
    """Tracks accepted global-replan cycles; Action retries are separate."""

    def __init__(self, max_cycles):
        self.max_cycles = max(1, int(max_cycles))
        self.used_cycles = 0
        self.exhausted = False

    def reset(self):
        self.used_cycles = 0
        self.exhausted = False

    def try_begin_cycle(self):
        """Reserve one SCAN-to-PCT handoff, returning False when exhausted."""
        if self.exhausted or self.used_cycles >= self.max_cycles:
            self.exhausted = True
            return False
        self.used_cycles += 1
        return True
