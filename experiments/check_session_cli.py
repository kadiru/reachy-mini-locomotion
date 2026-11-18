"""Exercise threaded CLI startup, video EOF stopping, and clean shutdown."""
import json
import secrets
import tempfile
from pathlib import Path
from reachy_locomotion.first_session import main
from reachy_locomotion.smoke import synthetic_camera_config
from reachy_locomotion.recording import load

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="reachy-session-cli-") as directory:
    tmp = Path(directory)
    token = tmp/"test.token"
    token.write_text(secrets.token_hex(32))
    token.chmod(0o600)
    camera = tmp/"camera.json"
    camera.write_text(json.dumps(synthetic_camera_config()))
    main(["record-session", "--mode", "preview", "--profile", str(root/"configs/first-session-preview.json"),
          "--token-file", str(token), "--port", "0", "--camera-config", str(camera),
          "--camera", str(root/"data/synthetic/first-session-smoke-04/SYNTHETIC-input.avi"),
          "--headless", "--seconds", ".8", "--output", str(tmp/"session")])
    rows = load(tmp/"session/aligned.jsonl")
    events = [r["data"] for r in rows[1:] if r["kind"] == "event"]
    integrity = next(e for e in events if e["name"] == "video_integrity")
    closed = next(e for e in events if e["name"] == "session_closed")
    commands = [r for r in rows[1:] if r["kind"] == "command"]
    assert rows[0]["evidence"] == "synthetic"
    assert integrity["complete"] and integrity["decoded_frames"] == 40
    assert closed["worker_threads_still_alive"] == 0
    assert not commands
    assert any(e["name"] == "camera_failed" for e in events)
    report = {"evidence": "synthetic", "passed": True, "threads_remaining": 0,
              "video_frames_decoded": 40, "unarmed_commands": 0,
              "camera_eof_detected": True, "physical_hardware_tested": False}
    (root/"experiments/reports/first-session-cli.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report))
