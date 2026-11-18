"""CLI for capture, calibration, bridge and guardian. Nothing runs on import."""
import argparse
import contextlib
import json
import os
import select
import secrets
import signal
import sys
import threading
import time
from pathlib import Path
from .recording import Recorder

COMMANDS = {"record-session", "guardian", "stop", "guardian-status", "make-token", "make-markers",
            "calibration-photos", "calibrate-camera", "calibrate-mount", "align-session", "session-smoke", "inspect-robot"}


def metadata(evidence, mode, profile, session_id):
    return {"session_id": session_id, "evidence": evidence, "control_mode": "teleoperation",
            "robot": {"variant": "wireless", "profile_sha256": profile.hash,
                      "actuation": mode, "mapping": "reconstructed_clutched_rpy_v1"},
            "setup": {"condition_id": session_id, "hardware_profile": profile.data},
            "measurement": {"max_pose_gap_s": .1, "xy_endpoint_uncertainty_m": None},
            "review": {"falls_reviewed": False, "external_assistance_reviewed": False}}


@contextlib.contextmanager
def keyboard():
    if not sys.stdin.isatty():
        yield lambda: None
        return
    import termios
    import tty
    fd = sys.stdin.fileno()
    before = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        yield lambda: sys.stdin.read(1) if select.select([sys.stdin], [], [], 0)[0] else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, before)


def run_guardian(args):
    from .profile import Profile
    from .protocol import read_key
    from .robot_io import MockRobot, LocalReachy
    from .guardian import Guardian, make_server
    profile = Profile(json.loads(Path(args.profile).read_text()), execute=args.execute)
    key = read_key(args.token_file)
    # A process-local lease is insufficient if another guardian starts. This
    # advisory OS lock covers our guardians; other SDK apps still need stopping.
    lock_file = None
    if args.execute:
        import fcntl
        lock_path = "/tmp/reachy-locomotion-daemon-%d.lock" % args.daemon_port
        fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        lock_file = os.fdopen(fd, "w")
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lock_file.close()
            raise ValueError("Another guardian owns this local daemon")
    robot = LocalReachy(profile, args.daemon_port) if args.execute else MockRobot(profile)
    with Recorder(args.output, metadata(robot.evidence, "guardian_execute" if args.execute else "guardian_mock", profile, Path(args.output).stem)) as rec:
        guardian = Guardian(robot, profile, rec)
        server = make_server(guardian, key, (args.bind, args.port))
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        print("Guardian %s on %s:%s; DISARMED. Space/x stops, q quits, Ctrl+C stops and exits." % (robot.evidence, *server.server_address), flush=True)
        try:
            with keyboard() as get_key:
                while True:
                    guardian.tick()
                    keypress = get_key()
                    if keypress in ("x", " ", "q"):
                        print(guardian.stop("guardian_operator_stop"), flush=True)
                    if keypress == "q":
                        break
                    time.sleep(.01)
        except KeyboardInterrupt:
            pass
        finally:
            guardian.stop("guardian_shutdown")
            server.shutdown()
            server.server_close()
            server_thread.join(timeout=2)
            if lock_file:
                lock_file.close()


def run_session(args):
    import cv2
    from .profile import Profile
    from .protocol import read_key
    from .guardian import GuardianClient
    from .camera import Tracker, CameraRecorder
    from .session import RecordingSession, PreviewSink, udp_worker, control_worker, state_worker
    execute = args.mode == "execute"
    if execute and (not args.guardian or not args.camera_config or args.camera is None):
        raise ValueError("Execute requires guardian, camera and calibrated camera configuration")
    if execute and not sys.stdin.isatty():
        raise ValueError("Execute requires an interactive terminal for local stop controls")
    profile = Profile(json.loads(Path(args.profile).read_text()), execute=execute)
    key = read_key(args.token_file)
    tracker = Tracker(json.loads(Path(args.camera_config).read_text())) if args.camera_config else None
    if args.camera is not None and tracker is None:
        raise ValueError("Camera calibration file required")
    if execute and (tracker.config.get("foot_mount_verified") is not True or tracker.config.get("measurement_verified") is not True):
        raise ValueError("Verify mount plus stationary/known-distance tracking before execute; set measurement_verified in camera profile")
    if execute and tracker.config.get("synthetic"):
        raise ValueError("Synthetic camera calibration cannot enable physical execution")
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    meta = metadata("synthetic" if args.mode == "preview" else "physical", args.mode, profile, out.name)
    if args.metadata:
        user = json.loads(Path(args.metadata).read_text())
        for field in ("setup", "measurement", "review"):
            meta[field].update(user.get(field, {}))
    if tracker:
        meta["setup"]["camera_calibration"] = tracker.config
    meta["setup"]["timestamp_limitations"] = "Shared host monotonic receipt; camera exposure and Quest source clock not synchronized"
    (out/"profile.json").write_text(json.dumps(profile.data, indent=2)+"\n")
    if tracker:
        (out/"camera-calibration.json").write_text(json.dumps(tracker.config, indent=2)+"\n")
    with Recorder(out/"raw.jsonl", meta) as rec:
        sink = GuardianClient(args.guardian, key, profile, rec) if execute else PreviewSink()
        if execute:
            status = sink.rpc("/status")
            if status["evidence"] != "physical" or status["profile_sha256"] != profile.hash:
                raise ValueError("Guardian evidence/profile does not match physical execution")
        session = RecordingSession(rec, profile, key, sink, args.mode, require_camera=tracker is not None)
        camera = None
        threads = []
        images, image_lock = {}, threading.Lock()

        def camera_loop():
            try:
                while not session.quit.is_set():
                    image, pose, error, receipt = camera.step()
                    session.camera_update(pose, receipt)
                    with image_lock:
                        images.update(image=image, error=error)
            except Exception as error:
                session.request_stop("camera_failed: " + str(error))
                session.event("camera_failed", detail=str(error))
            finally:
                camera.close()

        def worker(fn, *params):
            def guarded():
                try:
                    fn(*params)
                except Exception as error:
                    session.request_stop("worker_failed: " + str(error))
                    session.quit.set()
            thread = threading.Thread(target=guarded, daemon=True)
            threads.append(thread)
            thread.start()

        try:
            if args.camera is not None:
                source = int(args.camera) if args.camera.isdigit() else args.camera
                camera = CameraRecorder(source, tracker, rec, out)
                worker(camera_loop)
            worker(udp_worker, session, key, (args.bind, args.port))
            worker(control_worker, session)
            if args.state_url:
                if execute:
                    raise ValueError("Execute obtains state from guardian; omit --state-url")
                worker(state_worker, session, args.state_url.rstrip("/"))
            print("%s recording → %s; Quest UDP %s:%s" % (args.mode.upper(), out, args.bind, args.port), flush=True)
            print("a arm | hold Quest B+trigger to move | release/grip stops | Space/x stop | q quit | c/v cycle start/end | f fall | h catch | m marker", flush=True)
            started, printed = time.monotonic(), 0.
            with keyboard() as get_key:
                while not session.quit.is_set():
                    keypress = get_key()
                    if camera and not args.headless:
                        with image_lock:
                            image = images.get("image")
                            shown = None if image is None else image.copy()
                            error = images.get("error")
                        if shown is not None:
                            color = (0, 0, 255) if session.status == "ACTIVE" else (0, 200, 0)
                            cv2.putText(shown, session.status + "  SPACE:STOP  A:ARM  Q:QUIT", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
                            if error:
                                cv2.putText(shown, error[:80], (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 255), 1)
                            cv2.imshow("Reachy recording session", shown)
                        k = cv2.waitKey(1) & 255
                        if k != 255:
                            keypress = chr(k)
                    if keypress:
                        try:
                            session.key(keypress.lower())
                        except ValueError as error:
                            print("Not armed: " + str(error), flush=True)
                    if time.monotonic()-printed > 1:
                        print("%s | %s" % (session.status, session.reason or "waiting for operator/input"), flush=True)
                        printed = time.monotonic()
                    if args.seconds and time.monotonic()-started >= args.seconds:
                        break
                    time.sleep(.01)
        except KeyboardInterrupt:
            session.request_stop("operator_stop")
        finally:
            session.request_stop(session.reason or "session_end")
            try:
                session.stop()
            finally:
                session.quit.set()
                for thread in threads:
                    thread.join(timeout=2)
                cv2.destroyAllWindows()
                rec.write("event", {"name": "session_closed", "worker_threads_still_alive": sum(t.is_alive() for t in threads)}, "session")
    from .session import align_recording
    count = align_recording(out/"raw.jsonl", out/"aligned.jsonl")
    print("Saved raw and receipt-aligned logs (%d records). Review falls/catches and timing before analysis." % count)


def calibration_photos(args):
    import cv2
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise ValueError("Camera not available")
    count = 0
    print("Space saves the ORIGINAL frame; move/tilt checkerboard between captures. q quits.")
    try:
        while True:
            ok, image = cap.read()
            if not ok:
                raise ValueError("Camera read failed")
            shown = image.copy()
            cv2.putText(shown, "SPACE save (%d)   Q quit" % count, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 180, 0), 2)
            cv2.imshow("Calibration photos", shown)
            key = cv2.waitKey(1) & 255
            if key == ord("q"):
                break
            if key == ord(" "):
                if not cv2.imwrite(str(out/("frame-%03d.png" % count)), image):
                    raise OSError("Could not save calibration photo")
                count += 1
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description="First recording session tools; hardware execution is explicit")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("make-token")
    p.add_argument("output")
    p = sub.add_parser("make-markers")
    p.add_argument("output")
    p = sub.add_parser("calibration-photos")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--output", required=True)
    p = sub.add_parser("calibrate-camera")
    p.add_argument("--images", required=True)
    p.add_argument("--square-m", type=float, required=True)
    p.add_argument("--output", required=True)
    p = sub.add_parser("calibrate-mount")
    p.add_argument("--config", required=True)
    p.add_argument("--image", required=True)
    p.add_argument("--foot-x-m", type=float, required=True)
    p.add_argument("--foot-y-m", type=float, required=True)
    p.add_argument("--foot-yaw-deg", type=float, required=True)
    p.add_argument("--output", required=True)
    for name in ("guardian", "record-session"):
        p = sub.add_parser(name)
        p.add_argument("--profile", required=True)
        p.add_argument("--token-file", required=True)
        p.add_argument("--output", required=True)
        p.add_argument("--bind", default="127.0.0.1")
        p.add_argument("--port", type=int, default=8766 if name == "guardian" else 8765)
        if name == "guardian":
            p.add_argument("--execute", action="store_true")
            p.add_argument("--daemon-port", type=int, default=8000)
        else:
            p.add_argument("--mode", choices=("preview", "record-only", "execute"), default="preview")
            p.add_argument("--guardian")
            p.add_argument("--camera")
            p.add_argument("--camera-config")
            p.add_argument("--metadata")
            p.add_argument("--state-url")
            p.add_argument("--seconds", type=float, default=0)
            p.add_argument("--headless", action="store_true")
    for name in ("stop", "guardian-status"):
        p = sub.add_parser(name)
        p.add_argument("--guardian", required=True)
        p.add_argument("--token-file", required=True)
    p = sub.add_parser("align-session")
    p.add_argument("input")
    p.add_argument("output")
    p = sub.add_parser("session-smoke")
    p.add_argument("--output", required=True)
    p = sub.add_parser("inspect-robot")
    p.add_argument("--url", required=True)
    p.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    if threading.current_thread() is threading.main_thread():
        def terminate(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, terminate)
    try:
        if args.command == "make-token":
            path = Path(args.output)
            path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "w") as f:
                f.write(secrets.token_hex(32)+"\n")
            print("Created private token file: " + str(path))
        elif args.command == "make-markers":
            from .camera import make_printables
            make_printables(args.output)
        elif args.command == "calibration-photos":
            calibration_photos(args)
        elif args.command == "calibrate-camera":
            from .camera import calibrate_intrinsics
            paths = sorted(Path(args.images).glob("*.png"))
            result = calibrate_intrinsics(paths, args.output, square_m=args.square_m)
            print("Calibration RMS %.3f px. Mount still unverified." % result["calibration_rms_px"])
        elif args.command == "calibrate-mount":
            from .camera import calibrate_mount
            calibrate_mount(json.loads(Path(args.config).read_text()), args.image, args.foot_x_m, args.foot_y_m, args.foot_yaw_deg, args.output)
            print("Mount saved. Still verify stationary drift and a known displacement before execution.")
        elif args.command == "guardian":
            run_guardian(args)
        elif args.command == "record-session":
            run_session(args)
        elif args.command in ("stop", "guardian-status"):
            from .protocol import read_key
            from .robot_io import request_json
            from urllib.parse import urlparse
            u = urlparse(args.guardian)
            if u.scheme != "http" or not u.hostname or u.path not in ("", "/") or u.username or u.password or u.query or u.fragment:
                raise ValueError("Expected explicit guardian origin")
            result = request_json(args.guardian.rstrip("/"), "/stop" if args.command == "stop" else "/status",
                                  {"reason": "separate_terminal_stop"} if args.command == "stop" else None,
                                  read_key(args.token_file), timeout=.5)
            print(json.dumps(result, indent=2))
        elif args.command == "align-session":
            from .session import align_recording
            print("Aligned records:", align_recording(args.input, args.output))
        elif args.command == "session-smoke":
            from .smoke import run_smoke
            print(json.dumps(run_smoke(args.output), indent=2))
        elif args.command == "inspect-robot":
            from urllib.parse import urlparse
            from .robot_io import request_json, STATE_PATH
            u = urlparse(args.url)
            if u.scheme not in {"http", "https"} or not u.hostname or u.path not in {"", "/"} or u.username or u.password or u.query or u.fragment:
                raise ValueError("Expected explicit daemon origin")
            report = {"mode": "read_only", "origin": args.url, "results": {}}
            for route in ("/api/daemon/status", "/api/kinematics/info", "/api/motors/status", "/api/move/running", "/api/apps/current-app-status", STATE_PATH):
                try:
                    report["results"][route] = {"value": request_json(args.url.rstrip("/"), route, timeout=2)}
                except Exception as error:
                    report["results"][route] = {"error": str(error)}
            with open(args.output, "x") as f:
                json.dump(report, f, indent=2, allow_nan=False)
            print("Read-only inspection saved: " + args.output)
    except (ValueError, KeyError, OSError, ImportError, RuntimeError) as error:
        parser.exit(2, "Error: " + str(error) + "\n")
