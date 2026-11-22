"""Compare effective spin commands across versions that gained optional orbit fields."""
from dataclasses import asdict
import hashlib
import json


def effective_policy(model, policy):
    if model != 'spin':
        return policy
    from reachy_locomotion.continuous_spin import Orbit
    return dict(policy, orbit=asdict(Orbit(**policy['orbit'])))


def effective_digest(model, policy):
    payload = json.dumps(effective_policy(model, policy), sort_keys=True, allow_nan=False)
    return hashlib.sha256(payload.encode()).hexdigest()
