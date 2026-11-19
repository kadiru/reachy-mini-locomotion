"""Observational cycle metrics and the separate large-rocking gait objective.

No forces or commands are written here. Heading is relative to startup, sampled
at the same phase after every full left/right pair, including startup cycles.
"""
import math
import statistics


class CycleMonitor:
    def __init__(self, frames_per_cycle, warmup_frames, startup_yaw):
        self.period = frames_per_cycle
        self.warmup = warmup_frames
        self.forward = (math.cos(startup_yaw), math.sin(startup_yaw))
        self.rows, self.cycles, self.boundaries = [], [], []

    def observe(self, frame, position, roll_rad, heading_rad, side_forces):
        heading = math.degrees(heading_rad)
        if frame and frame % self.period == 0:
            self.boundaries.append(dict(cycle=frame//self.period, heading_deg=heading,
                                        warmup=frame <= self.warmup))
        if frame < self.warmup:
            return
        right, left = side_forces
        support = 'left' if right < .1 and left > .1 else 'right' if left < .1 and right > .1 else 'other'
        row = dict(x=float(position[0]), y=float(position[1]), roll_deg=math.degrees(roll_rad),
                   heading_deg=heading, support=support)
        self.rows.append(row)
        if len(self.rows) == self.period+1:
            start, end = self.rows[0], self.rows[-1]
            rolls = [x['roll_deg'] for x in self.rows]
            headings = [x['heading_deg'] for x in self.rows]
            left_fraction = sum(x['support']=='left' for x in self.rows[:-1])/self.period
            right_fraction = sum(x['support']=='right' for x in self.rows[:-1])/self.period
            self.cycles.append(dict(
                forward_mm=1000*((end['x']-start['x'])*self.forward[0]+(end['y']-start['y'])*self.forward[1]),
                roll_min_deg=min(rolls), roll_max_deg=max(rolls),
                bilateral_roll_deg=max(0.,min(max(rolls),-min(rolls))),
                yaw_excursion_deg=max(headings)-min(headings), heading_end_deg=end['heading_deg'],
                left_support_fraction=left_fraction,right_support_fraction=right_fraction,
                both_edges_loaded=left_fraction>=.1 and right_fraction>=.1))
            self.rows = [end]

    def report(self):
        c = self.cycles
        return dict(heading_rule='absolute startup-relative heading at every complete left/right cycle',
                    max_completed_cycle_heading_deg=max((abs(x['heading_deg']) for x in self.boundaries),default=0.),
                    completed_cycle_headings=self.boundaries, measured_cycles=c,
                    mean_forward_mm_per_cycle=statistics.mean(x['forward_mm'] for x in c) if c else 0.,
                    min_forward_mm_per_cycle=min((x['forward_mm'] for x in c),default=0.),
                    median_bilateral_roll_deg=statistics.median(x['bilateral_roll_deg'] for x in c) if c else 0.,
                    minimum_bilateral_roll_deg=min((x['bilateral_roll_deg'] for x in c),default=0.),
                    both_edges_cycle_fraction=sum(x['both_edges_loaded'] for x in c)/len(c) if c else 0.,
                    mean_yaw_excursion_deg=statistics.mean(x['yaw_excursion_deg'] for x in c) if c else 0.,
                    sample_hz=50)


def rocking_metrics(result):
    """Predeclared style targets distinguish this mode from the fast shuffle.

    Six degrees each side and 15 mm/cycle are simulation style targets, not
    manufacturer limits. The existing 35-degree/no-contact fall guard remains.
    """
    c = result['rocking_cycles']
    stable = bool(result['valid']) and result.get('min_motor_joint_limit_margin_rad') is not None
    stable = stable and result['min_motor_joint_limit_margin_rad'] >= -.005
    stable = stable and len(c['measured_cycles']) == result['requested_cycles']
    heading = c['max_completed_cycle_heading_deg']
    stride = c['mean_forward_mm_per_cycle']
    rock = c['median_bilateral_roll_deg']
    alternate = c['both_edges_cycle_fraction']
    speed = 1000*result['measurement_displacement_in_startup_frame_m'][0]/max(result['measured_seconds'],1e-9)
    course_ok = stable and heading <= 20 and stride > 0
    style_ok = rock >= 6 and alternate >= .8 and stride >= 15
    feasible = course_ok and style_ok
    # A continuous exploration score can reach the style region without making
    # big tilt itself unbounded reward or substituting yaw for translation.
    score = (1000 if feasible else 0)+stride+3*min(rock,10)-4*max(0.,heading-20)-40*max(0.,.8-alternate)
    if not stable:
        score = -1e6
    return dict(stable=stable,course_ok=course_ok,style_ok=style_ok,feasible=feasible,
                speed_mm_s=speed,stride_mm=stride,bilateral_roll_deg=rock,
                both_edges_cycle_fraction=alternate,cycle_heading_deg=heading,
                instantaneous_heading_deg=math.degrees(result['max_heading_from_startup_rad']),score=score)
