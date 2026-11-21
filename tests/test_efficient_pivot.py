import unittest

from reachy_locomotion.balance import BalanceController
from reachy_locomotion.efficient_pivot import EfficientPivot
from reachy_locomotion.geometry import euler
from reachy_locomotion.primitives import Primitive


def option(name, turn, edge, *, valid=True):
    segment = dict(turn=turn, torso_edge_deg=edge, base_edge_deg=edge,
                   measured_swings=4, dx=1., dy=0., drift=1., roll=10.)
    response = dict(delay=1, valid=valid, first_completed=valid, segments=[segment, segment], slip=2.)
    return dict(id=name, responses=[response])


class EfficientTurnTests(unittest.TestCase):
    def test_prefers_loaded_turn_and_qualified_repeat(self):
        bank = [option('flat',18,1),option('edge',16,12),option('joint stop',28,22,valid=False)]
        policy = EfficientPivot(bank,90,None)
        self.assertEqual(policy.choose(90)['id'],'edge')

    def test_first_only_motion_cannot_be_repeated(self):
        launch=option('launch',22,18,valid=False)
        launch['responses'][0]['first_completed']=True
        bank=[launch,option('repeat',12,9)]
        p=EfficientPivot(bank,-90,None)
        self.assertEqual(p.choose(90)['id'],'launch')
        p.strokes.append(dict(option='launch',residual_deg=90,measured_turn_deg=22,peak_sensed_roll_deg=10))
        self.assertEqual(p.choose(68)['id'],'repeat')

    def test_failed_transition_uses_observed_landing(self):
        strong=option('strong',18,14)
        strong['blocked_after']=[dict(delay=1,previous='strong',turn=18.,roll=10.)]
        other=option('other',10,7)
        p=EfficientPivot([strong,other],90,None)
        self.assertEqual(p.choose(90)['id'],'strong')
        p.strokes.append(dict(option='strong',residual_deg=90,measured_turn_deg=17.,peak_sensed_roll_deg=12.))
        self.assertEqual(p.choose(73)['id'],'other')

    def test_small_turn_opening_does_not_override_quarter_turn(self):
        a,b=option('large',18,14),option('small',12,10)
        b['trim_for']=['1/1']
        self.assertEqual(EfficientPivot([a,b],17,None).choose(17)['id'],'small')
        self.assertEqual(EfficientPivot([a,b],90,None).choose(90)['id'],'large')

    def test_finish_rule_requires_matching_sensor_history(self):
        large,small=option('large',18,14),option('small',8,6)
        settings={'late_choices':[dict(previous='large',turn=18.,roll=10.,error=18.,action='small')]}
        p=EfficientPivot([large,small],90,None,settings=settings)
        p.strokes.append(dict(option='large',residual_deg=36.,measured_turn_deg=18.,peak_sensed_roll_deg=10.))
        self.assertEqual(p.choose(18)['id'],'small')
        self.assertEqual(p.choose(40)['id'],'large')

    def test_no_qualified_launch_holds_neutral(self):
        bank = [option('joint stop',28,22,valid=False)]
        p = EfficientPivot(bank,90,None)
        pose,stage,_ = p.target(0,0)
        self.assertEqual(stage,'no qualified stroke')
        self.assertTrue(p.finished)
        self.assertEqual(pose[2,3],.177)

    def test_yaw_windup_mirrors_and_releases_without_edge(self):
        for sign in (-1,1):
            p=Primitive(yaw_deg=40*sign,yaw_at_roll_deg=10)
            c=BalanceController(p,dict(state_dependent_yaw=True,yaw_windup_deg=30))
            pose,_,_=c.target(.8,0)
            self.assertAlmostEqual(euler(pose)[2],-sign*.5235987756,places=6)
            pose,_,_=c.target(3.5,0)
            self.assertAlmostEqual(euler(pose)[2],0.)


if __name__=='__main__':
    unittest.main()
