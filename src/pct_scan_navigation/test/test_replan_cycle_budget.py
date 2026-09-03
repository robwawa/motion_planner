#!/usr/bin/env python3

import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from replan_cycle_budget import ReplanCycleBudget


class ReplanCycleBudgetTest(unittest.TestCase):

    def test_successful_handoffs_are_bounded_per_goal(self):
        budget = ReplanCycleBudget(3)

        self.assertTrue(budget.try_begin_cycle())
        self.assertTrue(budget.try_begin_cycle())
        self.assertTrue(budget.try_begin_cycle())
        self.assertEqual(budget.used_cycles, 3)
        self.assertFalse(budget.try_begin_cycle())
        self.assertTrue(budget.exhausted)

    def test_new_goal_resets_exhausted_budget(self):
        budget = ReplanCycleBudget(1)

        self.assertTrue(budget.try_begin_cycle())
        self.assertFalse(budget.try_begin_cycle())
        budget.reset()
        self.assertFalse(budget.exhausted)
        self.assertEqual(budget.used_cycles, 0)
        self.assertTrue(budget.try_begin_cycle())

    def test_invalid_limit_has_a_safe_minimum(self):
        budget = ReplanCycleBudget(0)

        self.assertEqual(budget.max_cycles, 1)
        self.assertTrue(budget.try_begin_cycle())
        self.assertFalse(budget.try_begin_cycle())


if __name__ == '__main__':
    unittest.main()
