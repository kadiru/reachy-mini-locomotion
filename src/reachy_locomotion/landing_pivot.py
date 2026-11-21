"""Choose a pivot continuation using relative heading and measured peak roll."""
from .efficient_pivot import EfficientPivot


class LandingPivot(EfficientPivot):
    def __init__(self, bank, target_deg, checked_ik, *, guidance, **kwargs):
        super().__init__(bank, target_deg, checked_ik, **kwargs)
        self.actions = {o['id']: o for o in [*bank, *guidance['extra_actions']]}
        # Unmodelled past actions supply no evidence to the legacy response tables.
        for extra in guidance['extra_actions']:
            self.responses[extra['id']] = [dict(segments=[]) for _ in bank[0]['responses']]
        self.guide = next((group for group in guidance['groups']
                           if group['delay'] == self.delay and
                           abs(group['target_deg'] - target_deg) <= group['target_tolerance_deg']), None)

    def choose(self, error):
        if self.guide is None:
            return super().choose(error)
        if not self.strokes:
            return self.actions[self.guide['opening']]
        # Match the same action history. Use every landing to distinguish surfaces
        # whose last heading alone is ambiguous. Reject out-of-range observations.
        step = len(self.strokes)
        candidates = []
        for trajectory in self.guide['paths']:
            if len(trajectory) <= step:
                continue
            distance = 0.
            for actual, fitted in zip(self.strokes, trajectory):
                if actual['option'] != fitted['action']:
                    break
                yaw_error = abs(actual['measured_turn_deg'] - fitted['turn']) / 3.
                roll_error = abs(actual['peak_sensed_roll_deg'] - fitted['roll']) / 2.
                distance = max(distance, yaw_error, roll_error)
            else:
                state = trajectory[step]
                distance = max(distance, abs(error - state['error']) / 4.)
                if distance <= self.guide.get('match_radius', 1.):
                    candidates.append((distance, state['action']))
        if candidates:
            return self.actions[min(candidates)[1]]
        return super().choose(error)
