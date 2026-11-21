import unittest

from reachy_locomotion.landing_pivot import LandingPivot


def action(name):
    segment = dict(turn=-12., torso_edge_deg=-10., base_edge_deg=-10., measured_swings=3,
                   dx=1., dy=0., drift=1., roll=15.)
    return dict(id=name, responses=[dict(delay=2, valid=True, first_completed=True,
                                         segments=[segment, segment], slip=2.)])


class LandingGuidanceTests(unittest.TestCase):
    def policy(self, goal=-90):
        path = [dict(action='probe', error=-90., turn=-20., roll=16.),
                dict(action='power', error=-70., turn=-22., roll=20.)]
        guide = dict(delay=2, target_deg=-90., target_tolerance_deg=3., opening='probe', paths=[path])
        return LandingPivot([action('legacy')], goal, None, delay=2,
                            guidance=dict(extra_actions=[dict(id='probe'), dict(id='power')], groups=[guide]))

    def test_selects_power_after_observed_landing(self):
        p = self.policy()
        self.assertEqual(p.choose(-90)['id'], 'probe')
        p.strokes.append(dict(option='probe', measured_turn_deg=-19., peak_sensed_roll_deg=17., residual_deg=-90.))
        self.assertEqual(p.choose(-71)['id'], 'power')
        self.assertEqual(p.choose(-60)['id'], 'legacy')

    def test_fitted_radius_rejects_an_ambiguous_landing(self):
        p = self.policy()
        p.guide['match_radius'] = .18
        p.strokes.append(dict(option='probe', measured_turn_deg=-19., peak_sensed_roll_deg=16., residual_deg=-90.))
        self.assertEqual(p.choose(-71)['id'], 'legacy')

    def test_unfamiliar_roll_or_untrained_goal_uses_prior_policy(self):
        p = self.policy()
        p.strokes.append(dict(option='probe', measured_turn_deg=-20., peak_sensed_roll_deg=22., residual_deg=-90.))
        self.assertEqual(p.choose(-70)['id'], 'legacy')
        self.assertEqual(self.policy(-45).choose(-45)['id'], 'legacy')


if __name__ == '__main__':
    unittest.main()
