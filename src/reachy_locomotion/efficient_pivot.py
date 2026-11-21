"""Select efficient rim strokes from measured single and repeated launches."""
import math

import numpy as np

from .target_pivot import TargetPivot


class EfficientPivot(TargetPivot):
    def __init__(self, bank, target_deg, checked_ik, *, settings=None, **kwargs):
        super().__init__(bank,target_deg,checked_ik,**kwargs)
        key=f'{self.delay}/{int(np.sign(target_deg))}'
        saved=bank[0].get('selection',{}).get(key,{}) if bank and abs(target_deg)>=60 else {}
        self.settings=settings if settings is not None else saved

    def choose(self, error):
        # Learn which response tables match the recent yaw and roll observations.
        tables = self.bank[0]['responses']
        indices = [i for i, profile in enumerate(tables) if profile['delay'] == self.delay]
        if not indices:
            raise ValueError('No trained response for the observed sensing delay')
        losses = np.zeros(len(indices))
        probe_losses=None
        for n, stroke in enumerate(self.strokes):
            responses = self.responses[stroke['option']]
            for k, i in enumerate(indices):
                segments = responses[i]['segments']
                if not segments:
                    losses[k] += 4.
                    continue
                seg = segments[min(n,1,len(segments)-1)]
                losses[k] += min(4.,abs(stroke['measured_turn_deg']-seg['turn'])/2.8)
                losses[k] += min(3.,abs(stroke['peak_sensed_roll_deg']-seg['roll'])/2.5)
            if n==0 and self.settings.get('plans'):
                probe_losses=losses.copy()
                losses+=2*probe_losses
        weights = np.exp(-(losses-losses.min())/self.settings.get('temperature',1.))
        weights /= weights.sum()
        direction = np.sign(error)
        total = abs(error)
        def rotation(deg, segment):
            a = math.radians(deg)
            dx, dy = segment['dx'], segment['dy']
            return np.array([math.cos(a)*dx-math.sin(a)*dy, math.sin(a)*dx+math.cos(a)*dy])

        locations = np.zeros((len(indices),2))
        for n, past in enumerate(self.strokes):
            for k,i in enumerate(indices):
                segments = self.responses[past['option']][i]['segments']
                if segments:
                    locations[k] += rotation(self.target_deg-past['residual_deg'],segments[min(n,1,len(segments)-1)])

        guard = 'valid' if self.strokes else 'first_completed'

        def cost(option):
            for rule in option.get('blocked_after', []):
                if self.delay != rule['delay']:
                    continue
                if not self.strokes:
                    if rule['previous'] is None:
                        return math.inf
                    continue
                previous = self.strokes[-1]
                if (previous['option'] == rule['previous']
                        and abs(previous['measured_turn_deg']-rule['turn']) <= 5
                        and abs(previous['peak_sensed_roll_deg']-rule['roll']) <= 5):
                    return math.inf
            outcomes = []
            for k,i in enumerate(indices):
                response = option['responses'][i]
                segments = response['segments']
                if not response[guard] or response['slip']>15:
                    outcomes.append(180.)
                    continue
                segment = segments[min(len(self.strokes),1)]
                turn = segment['turn']
                measured = self.measured_turns.get(option['id'])
                if measured is not None:
                    turn = .45*turn + .55*measured
                progress = total-abs(error-turn)
                edge = min(direction*segment['base_edge_deg'], direction*segment['torso_edge_deg'])
                distance = np.linalg.norm(locations[k]+rotation(self.heading,segment))
                benefit = progress + .6*min(progress,edge) - .85*max(0.,.55*direction*turn-edge)
                benefit -= .35*segment['measured_swings'] + .16*segment['drift'] + self.settings.get('travel_cost',.65)*max(0.,distance-self.settings.get('travel_start',43.))
                if abs(error-turn)<=3.5:
                    benefit += 9.
                outcomes.append(-benefit)
            # Exclude a failed repeated launch while its learned response is
            # compatible with the sensed landings. At startup all are plausible.
            if any(w > .025 and not option['responses'][i][guard] for w,i in zip(weights,indices)):
                return math.inf
            return float(weights@outcomes)

        if not self.strokes and 14 <= total <= 20:
            key=f'{self.delay}/{int(direction)}'
            for option in self.bank:
                if key in option.get('trim_for',()) and math.isfinite(cost(option)):
                    return option
        if not self.strokes and total >= 60:
            forced=self.settings.get('opening')
            if forced:
                option=next(o for o in self.bank if o['id']==forced)
                if math.isfinite(cost(option)):
                    return option
            key = f'{self.delay}/{int(direction)}'
            for option in self.bank:
                if key in option.get('opening_for', ()) and math.isfinite(cost(option)):
                    return option
        if self.strokes:
            last=self.strokes[-1]
            for rule in self.settings.get('late_choices',()):
                if (last['option']==rule['previous'] and abs(last['measured_turn_deg']-rule['turn'])<=5.
                        and abs(last['peak_sensed_roll_deg']-rule['roll'])<=5.
                        and abs(error-rule['error'])<=5.):
                    alternative=next(o for o in self.bank if o['id']==rule['action'])
                    if math.isfinite(cost(alternative)):
                        return alternative
        if probe_losses is not None and len(indices)>=2 and np.sign(self.target_deg)==direction:
            ranked=np.argsort(probe_losses)
            j=ranked[0]
            if probe_losses[j] <= 2. and probe_losses[ranked[1]]-probe_losses[j] >= 1.:
                profile=indices[j]
                plan=self.settings['plans'].get(str(profile),[])
                if plan and total>self.settings.get('trim_deg',14.):
                    chosen=plan[(len(self.strokes)-1)%len(plan)]
                    candidate=next(o for o in self.bank if o['id']==chosen)
                    predicted=candidate['responses'][profile]['segments'][-1]['turn']
                    if abs(error-predicted)<total and math.isfinite(cost(candidate)):
                        return candidate
        best_score, _, best = min((cost(o),i,o) for i,o in enumerate(self.bank))
        return best if math.isfinite(best_score) else None
