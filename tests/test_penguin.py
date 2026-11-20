"""Feedback must use available observations and report unfinished motion honestly."""
import unittest
from dataclasses import asdict
from reachy_locomotion.primitives import Primitive
from reachy_locomotion.penguin import PenguinController


class PenguinFeedbackTests(unittest.TestCase):
    def controller(self, **options):
        recipe=asdict(Primitive())
        feedback=dict(return_x_after_catch_s=.2)
        return PenguinController(recipe,feedback,cycles=1,min_step_s=3.,target_step_turn_deg=15,
                                 pivot_only=True,**options)

    def launch_and_settle(self, controller, end_frame=157, heading=0):
        for frame in range(end_frame+1):
            t=frame*.02
            controller.observe_heading(t,heading)
            controller.target(t,12 if .8<t<1.0 else 0)

    def test_settling_without_requested_turn_is_not_success(self):
        c=self.controller(max_repeats=1)
        self.launch_and_settle(c)
        self.assertTrue(c.finished)
        self.assertFalse(c.report()['completed'])
        self.assertFalse(c.report()['steps'][0]['turn_goal_met'])

    def test_fallback_uses_delayed_progress_not_current_ground_truth(self):
        fallback=dict(recipe=asdict(Primitive(roll_deg=25)),feedback=dict(return_x_after_catch_s=.2))
        c=self.controller(max_repeats=3,fallback_policy=fallback)
        self.launch_and_settle(c,end_frame=156)
        # A new heading observation cannot change the delayed feedback yet.
        c.observe_heading(3.14,-20)
        c.target(3.14,0)
        self.assertEqual(c.sensed_heading,0)
        self.assertTrue(c.fallback_active)
        self.assertEqual(c.report()['completed_steps'],0)
        self.assertEqual(c.child.p.roll_deg,25)
        c.observe_heading(3.16,-20)
        self.assertEqual(c.sensed_heading,-20)

    def test_recovered_successful_turn_needs_no_fallback(self):
        fallback=dict(recipe=asdict(Primitive(roll_deg=25)),feedback=dict(return_x_after_catch_s=.2))
        c=self.controller(max_repeats=3,fallback_policy=fallback)
        self.launch_and_settle(c,heading=-16)
        self.assertTrue(c.report()['completed'])
        self.assertFalse(c.fallback_active)
        self.assertEqual(len(c.bursts),1)

    def test_high_roll_prevents_sequencing_even_with_heading_goal(self):
        c=self.controller(max_repeats=3)
        for frame in range(180):
            t=frame*.02;c.observe_heading(t,-16);c.target(t,12 if t>.8 else 0)
        self.assertEqual(c.report()['completed_steps'],0)
        self.assertFalse(c.report()['completed'])


if __name__=='__main__':unittest.main()
