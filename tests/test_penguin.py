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

    def test_shorter_pauses_still_require_low_roll_rate(self):
        c=PenguinController(asdict(Primitive()),dict(return_x_after_catch_s=.2),cycles=1,
                            min_step_s=1.2,settle_s=.04,post_catch_s=.6,head_return_margin_s=0,
                            target_step_turn_deg=15,max_repeats=3,pivot_only=True)
        for frame in range(180):
            t=frame*.02;c.observe_heading(t,-16)
            roll=12 if .8<t<1.0 else (1 if frame%2 else -1)
            c.target(t,roll)
        self.assertLess(abs(c.child.sensed),2)
        self.assertGreater(abs(c.child.rate),10)
        self.assertEqual(c.report()['completed_steps'],0)

    def test_measured_head_gate_waits_after_body_has_settled(self):
        c=PenguinController(asdict(Primitive()),dict(return_x_after_catch_s=.2),cycles=1,
                            min_step_s=1.2,settle_s=.04,post_catch_s=.6,head_return_margin_s=0,
                            target_step_turn_deg=15,max_repeats=3,pivot_only=True,head_settle_yaw_deg=2)
        for frame in range(176):
            t=frame*.02;c.observe_heading(t,-16);c.observe_head(t,12)
            c.target(t,12 if .8<t<1 else 0)
        self.assertLess(abs(c.child.rate),10)
        self.assertEqual(c.report()['completed_steps'],0)
        for frame in range(176,226):
            t=frame*.02;c.observe_heading(t,-16);c.observe_head(t,0);c.target(t,0)
        self.assertTrue(c.report()['completed'])

    def test_rescue_requires_observed_weakness_in_regular_fallback(self):
        feedback=dict(return_x_after_catch_s=.2,sensor_delay_frames=2)
        regular=dict(recipe=asdict(Primitive(roll_deg=25)),feedback=feedback)
        rescue=dict(recipe=asdict(Primitive(roll_deg=20)),feedback=feedback)
        c=PenguinController(asdict(Primitive()),feedback,cycles=1,min_step_s=3.,target_step_turn_deg=15,
                            max_repeats=4,pivot_only=True,fallback_policy=regular,delayed_sensor_fallback=rescue,
                            rescue_after_weak_fallback_deg=5)
        for frame in range(166):
            t=frame*.02;local=t-c.started;c.observe_heading(t,0)
            c.target(t,12 if .8<local<1 else 0)
        self.assertEqual(c.active_policy,'fallback')
        self.assertEqual(c.child.p.roll_deg,25)
        for frame in range(166,331):
            t=frame*.02;local=t-c.started;c.observe_heading(t,0)
            c.target(t,12 if .8<local<1 else 0)
        self.assertEqual(c.active_policy,'fallback_delayed_sensor')
        self.assertEqual(c.child.p.roll_deg,20)

    def test_fallback_learning_can_be_independent_for_each_side(self):
        regular=dict(recipe=asdict(Primitive(roll_deg=25)),feedback=dict(return_x_after_catch_s=.2))
        c=PenguinController(asdict(Primitive()),dict(return_x_after_catch_s=.2),cycles=2,
                            min_step_s=3.,target_step_turn_deg=15,max_repeats=4,
                            fallback_policy=regular,per_side_fallback=True)
        for frame in range(500):
            t=frame*.02;local=t-c.started
            # First right burst is weak; its retry succeeds. The left side
            # must retain the primary recipe and its own observed history.
            heading=0 if not c.bursts else (-16 if not c.steps else 0)
            c.observe_heading(t,heading);c.target(t,12 if .8<local<1 else 0)
            if len(c.steps)==1:
                self.assertEqual(c.active_policy,'primary')
                self.assertFalse(c.fallback_active)
                self.assertTrue(c.report()['fallback_activated'])
            if len(c.steps)==2:break
        self.assertEqual(len(c.steps),2)
        self.assertEqual(c.side,1)
        self.assertTrue(c.fallback_active)
        self.assertEqual(c.active_policy,'fallback')
        self.assertEqual(c.side_fallback,{1:True,-1:False})

    def test_sustained_cycle_budget_is_bounded(self):
        c=PenguinController(asdict(Primitive()),{},cycles=30)
        self.assertEqual(c.report()['requested_steps'],60)
        self.assertFalse(c.report()['completed'])
        with self.assertRaises(ValueError):PenguinController(asdict(Primitive()),{},cycles=31)

    def test_twenty_cycles_keep_the_same_heading_reference(self):
        c=PenguinController(asdict(Primitive()),dict(return_x_after_catch_s=.2),cycles=20,
                            min_step_s=3.,target_step_turn_deg=15,max_repeats=1)
        for frame in range(7000):
            t=frame*.02;local=t-c.started
            c.observe_heading(t,-16 if c.side==1 else 0)
            c.target(t,12 if .8<local<1 else 0)
            if c.finished:break
        r=c.report()
        self.assertTrue(r['completed'])
        self.assertEqual(len(r['cycle_end_headings_deg']),20)
        self.assertEqual([s['target_heading_deg'] for s in r['steps']],[-15,0]*20)

    def test_budget_tolerance_requires_a_progressing_final_retry(self):
        for tolerance,expected in [(2,False),(4,True)]:
            c=self.controller(max_repeats=2,exhaustion_tolerance_deg=tolerance)
            for frame in range(350):
                t=frame*.02;local=t-c.started
                c.observe_heading(t,-6 if not c.bursts else -12)
                c.target(t,12 if .8<local<1 else 0)
                if c.finished:break
            self.assertEqual(len(c.bursts),2)
            self.assertEqual(c.report()['completed'],expected)
        c=self.controller(max_repeats=2,exhaustion_tolerance_deg=4)
        for frame in range(350):
            t=frame*.02;local=t-c.started;c.observe_heading(t,-12)
            c.target(t,12 if .8<local<1 else 0)
            if c.finished:break
        self.assertFalse(c.report()['completed'])


if __name__=='__main__':unittest.main()
