import copy
import math
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from reachy_locomotion.__main__ import synthetic_fixture
from reachy_locomotion.analysis import analyze, replay_plan
from reachy_locomotion.recording import validate, Recorder, load
from reachy_locomotion.capture import parse_emcee, capture_emcee, poll_state


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.rows = synthetic_fixture()

    def test_known_displacement_and_repeatability(self):
        r = analyze(self.rows)
        self.assertEqual(r["evidence"], "synthetic")
        self.assertEqual(r["measurable_cycles"], 3)
        self.assertAlmostEqual(r["mean_forward_m_per_cycle"], .01)
        self.assertAlmostEqual(r["sample_sd_forward_m"], 0)

    def test_sideways_is_not_forward(self):
        for r in self.rows[1:]:
            if r["kind"] == "base_pose":
                r["data"]["yaw_rad"] = math.pi/2
        report = analyze(self.rows)
        self.assertAlmostEqual(report["mean_forward_m_per_cycle"], 0)
        self.assertAlmostEqual(report["cycles"][0]["net_m"], .01)

    def test_closed_wobble_has_no_net_travel(self):
        for r in self.rows[1:]:
            if r["kind"] == "base_pose":
                r["data"]["x_m"] = .02*math.sin(2*math.pi*r["t_ns"]/1e9)
        self.assertAlmostEqual(analyze(self.rows)["mean_forward_m_per_cycle"], 0)

    def test_tracking_loss_is_not_interpolated_over(self):
        rows = [r for r in self.rows if not (r["kind"] == "base_pose" and 1e8 < r["t_ns"] < 8e8)]
        report = analyze(rows)
        self.assertFalse(report["cycles"][0]["valid_displacement"])
        self.assertEqual(report["measurable_cycles"], 2)

    def test_no_extrapolation(self):
        rows = [r for r in self.rows if not (r["kind"] == "base_pose" and r["t_ns"] == 0)]
        self.assertFalse(analyze(rows)["cycles"][0]["valid_displacement"])

    def test_head_pose_cannot_masquerade_as_base(self):
        next(r for r in self.rows if r["kind"] == "base_pose")["data"]["tracked_body"] = "head"
        with self.assertRaises(ValueError):
            analyze(self.rows)

    def test_fall_is_counted_and_excluded_from_clean_summary(self):
        self.rows.append({"kind": "event", "source": "test", "t_ns": 500000000, "data": {"name": "fall"}})
        self.rows = [self.rows[0]] + sorted(self.rows[1:], key=lambda r: r["t_ns"])
        report = analyze(self.rows)
        self.assertEqual(report["fall_fraction_complete_cycles"], 1/3)
        self.assertEqual(report["completed_cycles_without_flagged_incidents"], 2)
        with self.assertRaises(ValueError):
            replay_plan(self.rows)

    def test_missing_fall_review_is_unknown(self):
        self.rows[0]["review"]["falls_reviewed"] = False
        self.assertIsNone(analyze(self.rows)["fall_fraction_complete_cycles"])

    def test_missing_cycle_end_remains_in_denominator(self):
        rows = [r for r in self.rows if not (r["kind"] == "event" and r["data"] == {"name": "cycle_end", "cycle_id": "2"})]
        report = analyze(rows)
        self.assertEqual(report["total_cycles"], 3)
        self.assertEqual(report["measurable_cycles"], 2)

    def test_nonfinite_and_time_reversal_rejected(self):
        bad = copy.deepcopy(self.rows)
        bad[-1]["data"]["bad"] = float("nan")
        with self.assertRaises(ValueError):
            validate(bad)
        self.rows[-1]["t_ns"] = -1
        with self.assertRaises(ValueError):
            validate(self.rows)

    def test_replay_only_uses_submitted_targets(self):
        p = replay_plan(self.rows)
        self.assertFalse(p["hardware_ready"])
        self.assertEqual(p["commands"][-1]["offset_ns"], 3000000000)
        for row in self.rows:
            if row["kind"] == "command":
                row["data"]["stage"] = "requested"
        with self.assertRaises(ValueError):
            replay_plan(self.rows)

    def test_emcee_packet_is_input_not_target(self):
        packet = dict(number=1, teleop_engaged=True, a_temi_engaged=False, b_reachy_engaged=True, funny_engaged=False, pitch_deg=-45., roll_deg=0., yaw_deg=0., stick_x=0., stick_y=0.)
        self.assertEqual(parse_emcee(packet)["pitch_deg"], -45)
        packet["teleop_engaged"] = "false"
        with self.assertRaises(ValueError):
            parse_emcee(packet)

    def test_record_and_read_state_with_mock_transport(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"synthetic.jsonl"
            with Recorder(path, self.rows[0]) as recorder:
                with patch("urllib.request.urlopen", return_value=io.StringIO('{"body_yaw": 0.1}')) as get:
                    result = poll_state(recorder, "http://localhost:8000", duration_s=.001, hz=50)
                    self.assertIn("with_imu=true", get.call_args.args[0])
                self.assertEqual(result["state_samples"], 1)
                self.assertEqual(result["read_errors"], 0)
            stored = load(path)
            self.assertEqual(stored[1]["kind"], "state")
            self.assertLessEqual(stored[1]["data"]["request_t_ns"], stored[1]["t_ns"])
            with self.assertRaises(FileExistsError):
                Recorder(path, self.rows[0])

    def test_state_failure_does_not_invent_sample(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"synthetic.jsonl"
            with Recorder(path, self.rows[0]) as recorder:
                with patch("urllib.request.urlopen", side_effect=OSError("mock unavailable")):
                    result = poll_state(recorder, "http://localhost:8000", duration_s=.001, hz=50)
            self.assertEqual(result["state_samples"], 0)
            self.assertEqual(load(path)[1]["data"]["name"], "state_read_error")


if __name__ == "__main__":
    unittest.main()
