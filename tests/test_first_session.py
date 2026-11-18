import copy
import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from reachy_locomotion.profile import Profile, QuestMapper
from reachy_locomotion.protocol import QuestGate, encode, decode
from reachy_locomotion.geometry import rotation, transform, angle
from reachy_locomotion.guardian import Guardian
from reachy_locomotion.robot_io import MockRobot, LocalReachy
from reachy_locomotion.recording import Recorder, load
from reachy_locomotion.first_session import metadata
from reachy_locomotion.session import RecordingSession, PreviewSink, align_recording
from reachy_locomotion.camera import Tracker, estimate_marker, corners_3d, calibrate_intrinsics, calibrate_mount
from reachy_locomotion.smoke import synthetic_camera_config, render_frame, quest_message

ROOT = Path(__file__).resolve().parents[1]


class Clock:
    def __init__(self):
        self.ns = time.monotonic_ns()
    def __call__(self):
        return self.ns
    def advance(self, seconds):
        self.ns += int(seconds*1e9)


class FirstSessionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.profile_data = json.loads((ROOT/"configs/first-session-preview.json").read_text())
        self.profile = Profile(self.profile_data)
        self.rec = Recorder(self.path/"raw.jsonl", metadata("synthetic", "test", self.profile, "SYNTHETIC-test"))
        self.addCleanup(self.rec.close)
        self.key = b"a"*64

    def guardian(self):
        clock = Clock()
        robot = MockRobot(self.profile)
        guardian = Guardian(robot, self.profile, self.rec, clock)
        lease = guardian.arm(self.profile.hash)["lease"]
        clock.advance(.02)
        return guardian, robot, clock, lease

    def command(self, guardian, clock, lease, seq=0, target=None):
        target = target or {"head": np.eye(4).tolist(), "body_yaw": 0.}
        return guardian.command({"lease": lease, "sequence": seq, "deadline_ns": clock()+100_000_000, "target": target})

    def packet(self, gate, seq=1, **kwargs):
        challenge = gate.challenge()["challenge"]
        return encode(quest_message(seq, challenge, **kwargs), self.key)

    def test_preview_profile_cannot_enable_hardware(self):
        with self.assertRaisesRegex(ValueError, "hardware_verified"):
            Profile(self.profile_data, execute=True)

    def test_missing_limit_and_nonfinite_limit_rejected(self):
        data = copy.deepcopy(self.profile_data)
        data["max_base_tilt_rad"] = float("nan")
        with self.assertRaises(ValueError):
            Profile(data)
        data["max_base_tilt_rad"] = 0
        with self.assertRaises(ValueError):
            Profile(data)

    def test_quest_authentication_and_tampering(self):
        wire = encode({"number": 1}, self.key)
        self.assertEqual(decode(wire, self.key), {"number": 1})
        with self.assertRaises(ValueError):
            decode(wire.replace(b'1}', b'2}'), self.key)
        with self.assertRaises(ValueError):
            decode(wire, b"b"*64)

    def test_unpatched_legacy_input_cannot_actuate(self):
        gate = QuestGate(self.key)
        legacy = quest_message(1, "")["packet"]
        with self.assertRaises(ValueError):
            gate.accept(encode(legacy, self.key))

    def test_expired_challenge_and_rearm_nonce_rejected(self):
        clock = [1.]
        gate = QuestGate(self.key, clock=lambda: clock[0])
        raw = self.packet(gate)
        clock[0] += .3
        with self.assertRaises(ValueError):
            gate.accept(raw)
        raw = self.packet(gate, 2)
        gate.reset()
        with self.assertRaises(ValueError):
            gate.accept(raw)

    def test_duplicate_sequence_and_sender_change_rejected(self):
        gate = QuestGate(self.key)
        raw = self.packet(gate)
        gate.accept(raw)
        with self.assertRaises(ValueError):
            gate.accept(raw)
        data = quest_message(2, gate.challenge()["challenge"])
        data["sender_id"] = "different_sender"
        with self.assertRaises(ValueError):
            gate.accept(encode(data, self.key))

    def test_tracking_valid_must_be_boolean(self):
        gate = QuestGate(self.key)
        data = quest_message(1, gate.challenge()["challenge"])
        data["tracking_valid"] = "true"
        with self.assertRaises(ValueError):
            gate.accept(encode(data, self.key))

    def test_clutch_removes_emcee_pitch_offset(self):
        mapper = QuestMapper(self.profile)
        packet = quest_message(1, "")["packet"]
        mapper.clutch(packet)
        target, _ = mapper.map(packet, .02)
        np.testing.assert_allclose(target["head"], np.eye(4))

    def test_mapping_wrap_and_rate_envelope(self):
        mapper = QuestMapper(self.profile)
        packet = quest_message(1, "")["packet"]
        packet["yaw_deg"] = 179
        mapper.clutch(packet)
        packet["yaw_deg"] = -179
        previous = np.eye(4)
        for _ in range(20):
            target, detail = mapper.map(packet, .02)
            self.assertAlmostEqual(detail["requested_rpy_rad"][2], math.radians(.2))
            current = transform(target["head"])
            self.assertLessEqual(angle(previous, current), self.profile.data["max_head_speed_rad_s"]*.02+1e-7)
            previous = current

    def test_translation_and_nonrotation_target_rejected(self):
        target = {"head": np.eye(4).tolist(), "body_yaw": 0.}
        target["head"][0][3] = .01
        with self.assertRaises(ValueError):
            self.profile.check_target(target)
        target["head"][0][3] = 0
        target["head"][0][0] = 2
        with self.assertRaises(ValueError):
            self.profile.check_target(target)

    def test_guardian_does_not_enable_or_move_on_arm(self):
        guardian, robot, clock, lease = self.guardian()
        self.assertEqual(robot.submissions, [])
        self.assertEqual(guardian.snapshot()["mode"], "ARMED")
        with self.assertRaisesRegex(ValueError, "active lease"):
            guardian.arm(self.profile.hash)

    def test_expired_command_never_submitted(self):
        guardian, robot, clock, lease = self.guardian()
        clock.advance(.5)
        with self.assertRaises(ValueError):
            self.command(guardian, clock, lease)
        self.assertEqual(guardian.snapshot()["mode"], "STOPPED")
        self.assertEqual(len(robot.submissions), 1)  # measured-pose hold only

    def test_watchdog_revokes_lease_and_cannot_auto_resume(self):
        guardian, robot, clock, lease = self.guardian()
        self.command(guardian, clock, lease)
        clock.advance(.11)
        guardian.tick()
        n = len(robot.submissions)
        with self.assertRaises(ValueError):
            self.command(guardian, clock, lease, seq=1)
        self.assertEqual(len(robot.submissions), n)
        self.assertEqual(guardian.reason, "watchdog_timeout")

    def test_duplicate_command_latches_stop(self):
        guardian, robot, clock, lease = self.guardian()
        self.command(guardian, clock, lease)
        clock.advance(.01)
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.command(guardian, clock, lease)
        self.assertIsNone(guardian.lease)

    def test_unreachable_ik_latches_stop_before_target(self):
        guardian, robot, clock, lease = self.guardian()
        robot.ik = lambda target: np.full(7, float("nan"))
        with self.assertRaises(ValueError):
            self.command(guardian, clock, lease)
        self.assertIsNone(guardian.lease)

    def test_outside_rotation_and_high_rate_rejected(self):
        guardian, robot, clock, lease = self.guardian()
        head = np.eye(4)
        head[:3, :3] = rotation([.05, 0, 0])  # inside pose envelope, too fast
        with self.assertRaisesRegex(ValueError, "rate"):
            self.command(guardian, clock, lease, target={"head": head.tolist(), "body_yaw": 0.})
        self.assertIsNone(guardian.lease)

    def test_competing_translation_writer_detected(self):
        guardian, robot, clock, lease = self.guardian()
        robot.target["head"][0][3] = .01
        with self.assertRaisesRegex(ValueError, "outside this writer"):
            self.command(guardian, clock, lease)

    def test_stop_holds_measured_pose_not_neutral(self):
        guardian, robot, clock, lease = self.guardian()
        head = np.eye(4)
        head[:3, :3] = rotation([.001, 0, 0])
        target = {"head": head.tolist(), "body_yaw": 0.}
        self.command(guardian, clock, lease, target=target)
        guardian.stop()
        np.testing.assert_allclose(robot.submissions[-1]["head"], head)
        self.assertIsNone(guardian.lease)

    def test_logging_failure_still_attempts_hold(self):
        guardian, robot, clock, lease = self.guardian()
        with patch.object(self.rec, "write", side_effect=OSError("disk full")):
            guardian.stop("disk_full")
        self.assertEqual(len(robot.submissions), 1)
        self.assertEqual(guardian.status, "FAULT")

    def test_daemon_ignored_response_is_not_success(self):
        adapter = LocalReachy.__new__(LocalReachy)
        adapter.origin, adapter.timeout = "http://127.0.0.1:8000", .1
        with patch("reachy_locomotion.robot_io.request_json", return_value={"status": "ignored"}) as rpc:
            with self.assertRaises(RuntimeError):
                adapter.submit({"head": np.eye(4).tolist(), "body_yaw": 0.})
        self.assertEqual(rpc.call_args.args[2]["target_head_pose"]["m"], np.eye(4).ravel().tolist())

    def test_session_requires_trigger_release_after_arm(self):
        session = RecordingSession(self.rec, self.profile, self.key, PreviewSink(), require_camera=False)
        session.arm()
        session.receive(self.packet(session.gate, 1, engaged=True))
        session.step()
        self.assertEqual(session.status, "WAIT_RELEASE")
        session.receive(self.packet(session.gate, 2, engaged=False))
        session.step()
        self.assertEqual(session.status, "READY")
        session.receive(self.packet(session.gate, 3, engaged=True))
        session.step()
        self.assertEqual(session.status, "ACTIVE")
        session.receive(self.packet(session.gate, 4, valid=False))
        session.step()
        self.assertEqual(session.status, "STOPPED")
        session.receive(self.packet(session.gate, 5, engaged=True))
        session.step()
        self.assertEqual(session.status, "STOPPED")

    def test_grip_stop_is_latched(self):
        session = RecordingSession(self.rec, self.profile, self.key, PreviewSink(), require_camera=False)
        data = quest_message(1, session.gate.challenge()["challenge"])
        data["packet"]["funny_engaged"] = True
        session.receive(encode(data, self.key))
        self.assertTrue(session.stop_requested.is_set())

    def test_synthetic_sender_cannot_enter_physical_recording(self):
        session = RecordingSession(self.rec, self.profile, self.key, PreviewSink(), mode="record-only", require_camera=False)
        with self.assertRaises(ValueError):
            session.receive(self.packet(session.gate))
        self.assertTrue(session.stop_requested.is_set())

    def test_camera_stale_and_tilt_prevent_arming(self):
        session = RecordingSession(self.rec, self.profile, self.key, PreviewSink())
        with self.assertRaises(ValueError):
            session.arm()
        session.camera_update({"roll_rad": .5, "pitch_rad": 0, "x_m": 0, "y_m": 0}, time.monotonic_ns())
        with self.assertRaisesRegex(ValueError, "tilt"):
            session.arm()

    def test_blocking_preparation_cannot_refresh_expired_observations(self):
        sink = PreviewSink()
        session = RecordingSession(self.rec, self.profile, self.key, sink, require_camera=False)
        session.status = "ACTIVE"
        sink.arm()
        session.start_ns = time.monotonic_ns()
        session.last_control_ns = session.start_ns-20_000_000
        packet = quest_message(1, "", engaged=True)
        session.mapper.clutch(packet["packet"])
        session.latest = (packet, time.monotonic_ns())
        sink.synced_at = -2_000_000_000
        def delayed_sync():
            # Model expired input after clock sync without adding a slow test.
            session.latest = (packet, time.monotonic_ns()-300_000_000)
            session.gate.max_age = .000000001
        with patch.object(sink, "sync", side_effect=delayed_sync), patch.object(sink, "submit") as submit:
            session.step()
        submit.assert_not_called()
        self.assertEqual(session.status, "STOPPED")

    def test_normal_stop_does_not_mark_clean_trial_as_abort(self):
        session = RecordingSession(self.rec, self.profile, self.key, PreviewSink(), require_camera=False)
        session.request_stop("Deadman released")
        session.stop()
        names = [r["data"].get("name") for r in load(self.path/"raw.jsonl")[1:]]
        self.assertIn("teleop_stopped", names)
        self.assertNotIn("abort", names)

    def test_tracker_pixel_pipeline_and_mount_compensation(self):
        config = synthetic_camera_config()
        tracker = Tracker(config)
        # Rock around a fixed foot origin; marker center moves, foot should not.
        recovered = []
        for roll in [-.12, 0, .12]:
            foot = np.eye(4)
            foot[:3, :3] = rotation([roll, .05, 0])
            foot[:3, 3] = [.15, 0, 0]
            pose = tracker.process(render_frame(config, foot))
            xyz = np.array([pose["x_m"], pose["y_m"], pose["z_m"]])
            np.testing.assert_allclose(xyz, foot[:3, 3], atol=.005)
            recovered.append(xyz)
        self.assertLess(np.linalg.norm(recovered[-1]-recovered[0]), .005)

    def test_tracker_rejects_unverified_mount_missing_tag_and_wrong_size(self):
        config = synthetic_camera_config()
        foot = np.eye(4); foot[0, 3] = .15
        image = render_frame(config, foot)
        bad = dict(config, foot_mount_verified=False)
        with self.assertRaisesRegex(ValueError, "unverified"):
            Tracker(bad).process(image)
        with self.assertRaisesRegex(ValueError, "Missing"):
            Tracker(config).process(np.full_like(image, 255))
        with self.assertRaisesRegex(ValueError, "resolution"):
            Tracker(config).process(image[:300])

    def test_small_marker_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "small"):
            estimate_marker([[1,1],[11,1],[11,11],[1,11]], .035, np.eye(3), np.zeros(5))

    def test_mount_calibration_from_measured_placement(self):
        config = synthetic_camera_config()
        foot = np.eye(4); foot[:3, 3] = [.15, 0, 0]
        image_path = self.path/"mount.png"
        cv2.imwrite(str(image_path), render_frame(config, foot))
        config["T_marker_foot"] = np.eye(4).tolist()
        config["foot_mount_verified"] = False
        calibrated = calibrate_mount(config, image_path, .15, 0, 0, self.path/"mount.json")
        pose = Tracker(calibrated).process(cv2.imread(str(image_path)))
        np.testing.assert_allclose([pose["x_m"], pose["y_m"], pose["z_m"]], [.15, 0, 0], atol=1e-8)
        self.assertTrue(calibrated["foot_mount_verified"])

    def test_intrinsic_calibration_from_generated_checkerboard_views(self):
        matrix = np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1]])
        board = np.full((420, 600), 255, dtype=np.uint8)
        for y in range(7):
            for x in range(10):
                if (x+y) % 2 == 0:
                    board[y*60:(y+1)*60, x*60:(x+1)*60] = 0
        paths = []
        objects = np.array([[-.02,-.02,0],[.18,-.02,0],[.18,.12,0],[-.02,.12,0]], dtype=float)
        for i in range(12):
            rvec = np.array([.3*math.sin(i), .35*math.cos(i), .1*math.sin(i*.4)])
            tvec = np.array([-.07+.04*math.sin(i*.7), -.04+.02*math.cos(i*.3), .5+.01*i])
            corners, _ = cv2.projectPoints(objects, rvec, tvec, matrix, np.zeros(5))
            warp = cv2.getPerspectiveTransform(np.array([[0,0],[599,0],[599,419],[0,419]], np.float32), corners.reshape(4,2).astype(np.float32))
            image = cv2.warpPerspective(board, warp, (1280,720), borderValue=255)
            path = self.path/("board-%02d.png" % i)
            cv2.imwrite(str(path), image)
            paths.append(path)
        result = calibrate_intrinsics(paths, self.path/"intrinsics.json", square_m=.02)
        self.assertLess(result["calibration_rms_px"], 1.)
        self.assertFalse(result["foot_mount_verified"])
        self.assertAlmostEqual(result["camera_matrix"][0][0]/900, 1., delta=.08)

    def test_alignment_preserves_raw_receipt_and_sorts_sample_time(self):
        self.rec.write("event", {"name": "test"}, "test")
        self.rec.write("base_pose", {"tracked_body": "foot", "frame": "world", "x_m": 0, "y_m": 0,
                                   "yaw_rad": 0, "sample_t_ns": 0}, "synthetic")
        destination = self.path/"aligned.jsonl"
        align_recording(self.path/"raw.jsonl", destination)
        rows = load(destination)
        self.assertEqual(rows[1]["kind"], "base_pose")
        self.assertEqual(rows[1]["t_ns"], 0)
        self.assertIn("raw_record_t_ns", rows[1]["data"])
        with self.assertRaises(FileExistsError):
            align_recording(self.path/"raw.jsonl", destination)


if __name__ == "__main__":
    unittest.main()
