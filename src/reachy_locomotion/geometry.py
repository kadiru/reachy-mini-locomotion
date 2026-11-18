"""Explicit right-handed transforms; XYZ extrinsic Euler convention of the SDK."""
import math
import numpy as np
from .recording import validate_target


def transform(value):
    a = np.asarray(value, dtype=float)
    validate_target({"stage": "requested", "target": {"head": a.tolist()}})
    return a


def rotation(rpy):
    x, y, z = rpy
    cx, cy, cz = math.cos(x), math.cos(y), math.cos(z)
    sx, sy, sz = math.sin(x), math.sin(y), math.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]])


def euler(matrix):
    r = np.asarray(matrix)[:3, :3]
    return np.array([math.atan2(r[2, 1], r[2, 2]),
                     math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0])),
                     math.atan2(r[1, 0], r[0, 0])])


def angle(a, b):
    return math.acos(float(np.clip((np.trace(a[:3, :3].T @ b[:3, :3])-1)/2, -1, 1)))


def wrapped(value):
    return (value + math.pi) % (2*math.pi) - math.pi
