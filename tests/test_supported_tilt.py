import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from experiments.supported_tilt import compare, daemon_info
from reachy_locomotion.recording import Recorder


class SupportedTiltTests(unittest.TestCase):
    def test_physical_capture_checks_reported_simulator_flag(self):
        with patch('experiments.supported_tilt.request_json', return_value={'simulation_enabled':True}) as read:
            with self.assertRaisesRegex(ValueError, 'simulated backend'):
                daemon_info('http://reachy-mini.local:8000')
            read.assert_called_once_with('http://reachy-mini.local:8000','/api/daemon/status',timeout=2.)

    def test_static_reference_and_missing_imu(self):
        with tempfile.TemporaryDirectory() as directory:
            files = []
            for label, tilt in (('upright', 0), ('left', -32), ('right', 32), ('unknown', None)):
                file = Path(directory)/(label+'.jsonl')
                files.append(file)
                pose = {'m': np.eye(4).flatten().tolist()}
                state = {'head_pose': pose, 'target_head_pose': pose}
                if tilt is not None:
                    rad = math.radians(tilt)
                    state['imu'] = dict(accelerometer=[0., 9.81*math.sin(rad), 9.81*math.cos(rad)],
                                        gyroscope=[0., 0., 0.])
                meta = dict(session_id=label, robot={}, evidence='synthetic', control_mode='teleoperation',
                            setup={'condition_id': label}, measurement={})
                with Recorder(file, meta) as recorder:
                    recorder.write('state', dict(sdk_state=state, request_t_ns=0), 'fixture')
            results = compare(files)
            for row in results[1:3]:
                self.assertAlmostEqual(row['gravity_change_from_upright_deg'], 32.)
                self.assertEqual(row['target_present_angle_p95_deg'], 0.)
                self.assertEqual(row['evidence'], 'synthetic')
            self.assertIsNone(results[-1]['gravity_change_from_upright_deg'])
            self.assertEqual(results[-1]['imu_samples'], 0)
