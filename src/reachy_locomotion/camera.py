"""Calibrated 3D fiducial tracking and original-frame recording.

OpenCV host receipt is NOT sensor exposure time. Per-frame timestamps are the
time base; the AVI's fixed playback FPS is only a viewing convenience.
"""
import hashlib
import json
import math
import time
from pathlib import Path
import cv2
import numpy as np
from .geometry import transform, euler, angle

DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def corners_3d(size):
    h = size/2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def estimate_marker(corners, size, matrix, distortion, max_error=2., min_gap=.1):
    image = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if abs(cv2.contourArea(image.astype(np.float32))) < 2500:
        raise ValueError("Marker too small")
    points = corners_3d(size)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(points, image, matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError("No marker pose solution")
    candidates = []
    for rv, tv in zip(rvecs, tvecs):
        r, _ = cv2.Rodrigues(rv)
        if not np.all(np.isfinite(r)) or np.any((points @ r.T + tv.reshape(3))[:, 2] <= 0):
            continue
        projection, _ = cv2.projectPoints(points, rv, tv, matrix, distortion)
        error = float(np.sqrt(np.mean(np.sum((projection.reshape(4, 2)-image)**2, axis=1))))
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = r, tv.reshape(3)
        candidates.append((error, pose))
    candidates.sort(key=lambda item: item[0])
    if not candidates or candidates[0][0] > max_error:
        raise ValueError("Marker reprojection error too large")
    if len(candidates) > 1 and candidates[1][0]-candidates[0][0] < min_gap and angle(candidates[0][1], candidates[1][1]) > math.radians(2):
        raise ValueError("Ambiguous planar marker pose; use a more oblique view")
    return candidates[0][1], candidates[0][0]


class Tracker:
    def __init__(self, config):
        self.config = config
        if config.get("schema_version") != 1:
            raise ValueError("Camera schema_version must be 1")
        self.matrix = np.asarray(config["camera_matrix"], dtype=float)
        self.distortion = np.asarray(config["distortion_coefficients"], dtype=float)
        if self.matrix.shape != (3, 3) or not np.all(np.isfinite(self.matrix)) or self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0:
            raise ValueError("Valid calibrated intrinsics required")
        if not np.all(np.isfinite(self.distortion)) or self.distortion.size not in (4, 5, 8, 12, 14):
            raise ValueError("Invalid distortion coefficients")
        if config["world_marker_id"] == config["foot_marker_id"]:
            raise ValueError("World and foot markers must differ")
        for field in ("world_marker_size_m", "foot_marker_size_m"):
            if not .005 < config[field] < .5:
                raise ValueError("Measure the black square in meters: " + field)
        self.mount = transform(config["T_marker_foot"])
        self.hash = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(DICTIONARY, params)

    def poses(self, image):
        h, w = image.shape[:2]
        if [w, h] != self.config["image_size"]:
            raise ValueError("Camera resolution differs from calibration")
        corners, ids, _ = self.detector.detectMarkers(image)
        ids = [] if ids is None else ids.flatten().tolist()
        result = {}
        for key in ("world", "foot"):
            marker_id = self.config[key+"_marker_id"]
            if ids.count(marker_id) != 1:
                raise ValueError("Missing/duplicate " + key + " marker")
            c = corners[ids.index(marker_id)].reshape(4, 2)
            if np.any(c[:, 0] < 2) or np.any(c[:, 0] > w-3) or np.any(c[:, 1] < 2) or np.any(c[:, 1] > h-3):
                raise ValueError("Marker clipped at image boundary")
            result[key] = estimate_marker(c, self.config[key+"_marker_size_m"], self.matrix, self.distortion,
                                           self.config.get("max_reprojection_error_px", 2.), self.config.get("min_pose_error_gap_px", .1))
        return result

    def process(self, image):
        poses = self.poses(image)
        world_marker = np.linalg.inv(poses["world"][0]) @ poses["foot"][0]
        foot = world_marker @ self.mount
        if self.config.get("foot_mount_verified") is not True:
            raise ValueError("Foot mount transform unverified; run calibrate-mount")
        roll, pitch, yaw = euler(foot)
        return {"frame": "world", "tracked_body": "foot", "x_m": float(foot[0, 3]),
                "y_m": float(foot[1, 3]), "z_m": float(foot[2, 3]), "roll_rad": float(roll),
                "pitch_rad": float(pitch), "yaw_rad": float(yaw), "T_world_foot": foot.tolist(),
                "world_reprojection_error_px": poses["world"][1], "foot_reprojection_error_px": poses["foot"][1],
                "calibration_sha256": self.hash, "confidence": "quality_gated_not_probability"}


class CameraRecorder:
    def __init__(self, source, tracker, recorder, directory, capture_factory=cv2.VideoCapture):
        self.tracker, self.recorder = tracker, recorder
        self.capture = capture_factory(source)
        if not self.capture.isOpened():
            raise ValueError("Cannot open camera; check OS permission and camera index")
        w, h = tracker.config["image_size"]
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.writer = None
        self.path = Path(directory)/"camera-original.avi"
        if self.path.exists():
            self.capture.release()
            raise FileExistsError(self.path)
        self.frame = 0

    def step(self):
        started = time.monotonic_ns()
        ok, image = self.capture.read()
        receipt = time.monotonic_ns()
        if not ok:
            raise RuntimeError("Camera frame unavailable")
        h, w = image.shape[:2]
        if [w, h] != self.tracker.config["image_size"]:
            raise ValueError("Live camera size differs from calibration")
        if self.writer is None:
            fps = self.capture.get(cv2.CAP_PROP_FPS)
            fps = fps if np.isfinite(fps) and 1 <= fps <= 240 else 30.
            self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))
            if not self.writer.isOpened():
                raise RuntimeError("Could not create original-frame video")
        self.writer.write(image)
        timing = {"sample_t_ns": receipt-self.recorder.origin, "read_start_t_ns": started-self.recorder.origin,
                  "timestamp_basis": "host_frame_receipt", "exposure_time_known": False,
                  "frame_index": self.frame, "stream_id": "external_camera", "path": self.path.name}
        self.recorder.write("video_frame", timing, "opencv_camera")
        self.frame += 1
        try:
            pose = self.tracker.process(image)
            pose.update(sample_t_ns=timing["sample_t_ns"], frame_index=timing["frame_index"],
                        sample_time_basis="host_frame_receipt")
            self.recorder.write("base_pose", pose, "opencv_aruco_3d")
            error = None
        except ValueError as e:
            pose, error = None, str(e)
            self.recorder.write("event", {"name": "tracking_lost", "detail": error, **timing}, "opencv_camera")
        return image, pose, error, receipt

    def close(self):
        self.capture.release()
        if self.writer:
            self.writer.release()
            self.writer = None
            check = cv2.VideoCapture(str(self.path))
            count = 0
            try:
                while check.isOpened():
                    ok, _ = check.read()
                    if not ok:
                        break
                    count += 1
            finally:
                check.release()
            self.recorder.write("event", {"name": "video_integrity", "expected_frames": self.frame,
                "decoded_frames": count, "complete": count == self.frame}, "opencv_camera")
            if count != self.frame:
                raise RuntimeError("Recorded video incomplete: %d/%d frames decode" % (count, self.frame))


def make_printables(directory):
    from PIL import Image
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=False)
    dpi = 300
    for marker_id, mm in ((0, 80), (1, 35)):
        px = round(mm*dpi/25.4)
        image = cv2.aruco.generateImageMarker(DICTIONARY, marker_id, px)
        margin = round(10*dpi/25.4)
        image = cv2.copyMakeBorder(image, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
        pil = Image.fromarray(image)
        pil.save(out/('marker-%d-%dmm.png' % (marker_id, mm)), dpi=(dpi, dpi))
        pil.save(out/('marker-%d-%dmm.pdf' % (marker_id, mm)), resolution=dpi)
    square = round(20*dpi/25.4)
    board = np.full((7*square, 10*square), 255, dtype=np.uint8)
    for y in range(7):
        for x in range(10):
            if (x+y) % 2 == 0:
                board[y*square:(y+1)*square, x*square:(x+1)*square] = 0
    board = cv2.copyMakeBorder(board, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    Image.fromarray(board).save(out/"checkerboard-9x6-20mm.pdf", resolution=dpi)
    (out/"PRINTING.txt").write_text("Print PDF at 100% actual size. Measure black marker squares: ID 0 = 80 mm, ID 1 = 35 mm.\nCheckerboard: 9x6 INNER corners, nominal squares 20 mm; measure actual printed size.\nMount ID 0 flat and fixed on the test surface; its +X points right and +Y toward the top of the unrotated marker.\nMount ID 1 rigidly to the lower foot, not the moving shell/head.\n")


def calibrate_intrinsics(paths, output, cols=9, rows=6, square_m=.02):
    if len(paths) < 10 or not .001 < square_m < .2:
        raise ValueError("At least ten varied calibration images and measured square size required")
    object_points = np.zeros((cols*rows, 3), dtype=np.float32)
    object_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)*square_m
    objects, images, used = [], [], []
    size = None
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError("Cannot read calibration image: " + str(path))
        current = image.shape[::-1]
        if size and size != current:
            raise ValueError("Calibration image resolutions differ")
        size = current
        ok, corners = cv2.findChessboardCornersSB(image, (cols, rows))
        if ok:
            objects.append(object_points.copy())
            images.append(corners.astype(np.float32))
            used.append(str(path))
    if len(used) < 10:
        raise ValueError("Fewer than ten checkerboard detections")
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(objects, images, size, None, None)
    if not np.isfinite(rms) or rms > 1.:
        raise ValueError("Calibration RMS exceeds 1 px; inspect images and recapture")
    config = {"schema_version": 1, "image_size": list(size), "camera_matrix": matrix.tolist(),
              "distortion_coefficients": distortion.ravel().tolist(), "calibration_rms_px": rms,
              "calibration_images": used, "square_size_m": square_m, "world_marker_id": 0, "foot_marker_id": 1,
              "world_marker_size_m": .08, "foot_marker_size_m": .035, "T_marker_foot": np.eye(4).tolist(),
              "foot_mount_verified": False, "max_reprojection_error_px": 2., "min_pose_error_gap_px": .1}
    with open(output, "x") as f:
        json.dump(config, f, indent=2, allow_nan=False)
    return config


def calibrate_mount(config, image_path, foot_x, foot_y, foot_yaw_deg, output):
    """Robot upright: measured foot origin at z=0, roll=pitch=0 in world-tag frame."""
    from .geometry import rotation
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError("Cannot read mount-calibration image")
    tracker = Tracker(config)
    poses = tracker.poses(image)
    world_marker = np.linalg.inv(poses["world"][0]) @ poses["foot"][0]
    world_foot = np.eye(4)
    world_foot[:3, :3] = rotation([0, 0, math.radians(foot_yaw_deg)])
    world_foot[:3, 3] = [foot_x, foot_y, 0]
    config = dict(config, T_marker_foot=(np.linalg.inv(world_marker) @ world_foot).tolist(),
                  foot_mount_verified=True, mount_calibration={"image": str(image_path),
                  "T_world_foot_measured": world_foot.tolist(), "requires_static_and_known_displacement_check": True})
    with open(output, "x") as f:
        json.dump(config, f, indent=2, allow_nan=False)
    return config
