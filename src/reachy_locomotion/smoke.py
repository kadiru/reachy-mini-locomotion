"""Synthetic end-to-end verification; never imports or contacts a real robot."""
import json
import math
import secrets
import socket
import threading
import time
from pathlib import Path
import cv2
import numpy as np
from .camera import DICTIONARY, corners_3d, Tracker, CameraRecorder
from .geometry import rotation
from .profile import Profile
from .recording import Recorder, load
from .first_session import metadata
from .guardian import Guardian, GuardianClient, make_server
from .robot_io import MockRobot
from .protocol import encode, decode
from .session import RecordingSession, udp_worker, align_recording


def synthetic_camera_config():
    mount = np.eye(4)
    mount[2, 3] = -.03
    return {"schema_version": 1, "image_size": [1280, 720],
            "camera_matrix": [[1200., 0, 640], [0, 1200., 360], [0, 0, 1]],
            "distortion_coefficients": [0., 0, 0, 0, 0], "world_marker_id": 0, "foot_marker_id": 1,
            "world_marker_size_m": .08, "foot_marker_size_m": .035,
            "T_marker_foot": mount.tolist(), "foot_mount_verified": True,
            "max_reprojection_error_px": 2., "min_pose_error_gap_px": .1,
            "synthetic": True, "measurement_verified": False}


def render_frame(config, foot):
    """Known 3D geometry rendered to pixels, including marker/foot offset."""
    image = np.full((720, 1280, 3), 255, dtype=np.uint8)
    camera_world = np.eye(4)
    camera_world[:3, :3] = rotation([math.pi-.65, .1, 0])
    camera_world[:3, 3] = [-.07, .01, .5]
    objects = [(0, .08, np.eye(4)), (1, .035, foot @ np.linalg.inv(np.asarray(config["T_marker_foot"])))]
    for marker_id, size, world_pose in objects:
        pose = camera_world @ world_pose
        rv, _ = cv2.Rodrigues(pose[:3, :3])
        projected, _ = cv2.projectPoints(corners_3d(size), rv, pose[:3, 3], np.asarray(config["camera_matrix"]), np.zeros(5))
        points = projected.reshape(4, 2).astype(np.float32)
        marker = cv2.aruco.generateImageMarker(DICTIONARY, marker_id, 400)
        homography = cv2.getPerspectiveTransform(np.array([[0, 0], [399, 0], [399, 399], [0, 399]], np.float32), points)
        warped = cv2.warpPerspective(marker, homography, (1280, 720), flags=cv2.INTER_NEAREST, borderValue=255)
        image = np.minimum(image, cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR))
    return image


def quest_message(seq, challenge, engaged=False, roll=0., valid=True):
    return {"protocol": "quest-recording-v1", "sender_id": "SYNTHETIC_TEST_SENDER", "synthetic": True,
            "challenge": challenge, "source_time_s": time.monotonic(), "tracking_valid": valid,
            "packet": {"number": seq, "teleop_engaged": engaged, "a_temi_engaged": False,
                       "b_reachy_engaged": engaged, "funny_engaged": False, "pitch_deg": -45.,
                       "roll_deg": roll, "yaw_deg": 0., "stick_x": 0., "stick_y": 0.}}


def run_smoke(directory):
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    profile = Profile(json.loads((root/"configs/first-session-preview.json").read_text()))
    camera_config = synthetic_camera_config()
    tracker = Tracker(camera_config)
    key = secrets.token_hex(32).encode()
    source_path = out/"SYNTHETIC-input.avi"
    writer = cv2.VideoWriter(str(source_path), cv2.VideoWriter_fourcc(*"MJPG"), 50, (1280, 720))
    if not writer.isOpened():
        raise RuntimeError("Synthetic video writer unavailable")
    truth = []
    for i in range(40):
        foot = np.eye(4)
        foot[:3, :3] = rotation([.12*math.sin(i*.2), .05, 0])
        foot[:3, 3] = [.15+i*.0005, 0, 0]
        truth.append(foot)
        writer.write(render_frame(camera_config, foot))
    writer.release()
    robot = MockRobot(profile)
    finished = threading.Event()
    seq = 0
    with Recorder(out/"guardian.jsonl", metadata("synthetic", "mock", profile, "SYNTHETIC-guardian")) as gr, \
            Recorder(out/"raw.jsonl", metadata("synthetic", "preview", profile, "SYNTHETIC-session")) as rec:
        guardian = Guardian(robot, profile, gr)
        server = make_server(guardian, key, ("127.0.0.1", 0))
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        def watchdog():
            while not finished.wait(.005):
                guardian.tick()
        watch = threading.Thread(target=watchdog, daemon=True)
        watch.start()
        client = GuardianClient("http://127.0.0.1:%d" % server.server_port, key, profile, rec)
        session = RecordingSession(rec, profile, key, client, mode="preview", require_camera=True)
        ready, address = threading.Event(), []
        def bound(value):
            address.append(value)
            ready.set()
        udp = threading.Thread(target=udp_worker, args=(session, key, ("127.0.0.1", 0), bound), daemon=True)
        udp.start()
        if not ready.wait(2):
            raise RuntimeError("UDP server did not start")
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.settimeout(.5)
        challenge = ""
        camera = CameraRecorder(str(source_path), tracker, rec, out)
        poses, errors = [], []
        try:
            # Each real transport exchange echoes the preceding host challenge.
            for i in range(40):
                image, pose, error, receipt = camera.step()
                session.camera_update(pose, receipt)
                if pose:
                    poses.append(pose)
                    errors.append(float(np.linalg.norm(np.array([pose["x_m"], pose["y_m"], pose["z_m"]])-truth[i][:3, 3])))
                if i == 1:
                    session.arm()
                msg = quest_message(seq, challenge, engaged=i >= 6, roll=2*math.sin(max(0, i-6)*.15))
                sender.sendto(encode(msg, key), address[0])
                challenge = decode(sender.recv(8192), key)["challenge"]
                seq += 1
                session.step()
                time.sleep(.02)
            active_before_loss = guardian.snapshot()["mode"]
            # Kill input flow: no host stop request; robot-side watchdog must act.
            time.sleep(profile.data["command_timeout_s"]+.08)
            watchdog_mode = guardian.snapshot()["mode"]
            session.request_stop("synthetic_transport_loss")
            session.stop()
        finally:
            session.quit.set()
            udp.join(timeout=1)
            sender.close()
            camera.close()
            guardian.stop("smoke_shutdown")
            finished.set()
            watch.join(timeout=1)
            server.shutdown()
            server.server_close()
            server_thread.join(timeout=1)
    count = align_recording(out/"raw.jsonl", out/"aligned.jsonl")
    rows = load(out/"aligned.jsonl")
    submitted = sum(r["kind"] == "command" and r["data"]["stage"] == "submitted" for r in rows[1:])
    report = {"evidence": "synthetic", "physical_hardware_tested": False,
              "transport": "authenticated UDP → HTTP guardian → MockRobot",
              "camera": "generated 3D markers → MJPEG → OpenCV detection/PnP",
              "frames": 40, "tracked_frames": len(poses), "max_position_error_m": max(errors) if errors else None,
              "commands_submitted": submitted, "active_before_input_loss": active_before_loss,
              "guardian_after_input_loss": watchdog_mode, "aligned_records": count,
              "passed": submitted > 10 and len(poses) >= 35 and max(errors) < .005 and active_before_loss == "ACTIVE" and watchdog_mode == "STOPPED"}
    (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    if not report["passed"]:
        raise RuntimeError("Synthetic pipeline failed; inspect " + str(out/"report.json"))
    return report
