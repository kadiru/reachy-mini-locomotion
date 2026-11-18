"""Authenticated LAN Quest input. Never trust legacy packets for actuation.

Wire: lowercase HMAC-SHA256 hex + newline + UTF-8 JSON. Challenges bound
packet age without assuming that Unity and Python share a clock.
"""
import hashlib
import hmac
import json
import secrets
import time
from .capture import parse_emcee
from .recording import finite


def read_key(path):
    with open(path) as f:
        key = f.read().strip()
    if len(key) != 64 or any(c not in "0123456789abcdef" for c in key):
        raise ValueError("Token file must contain 64 lowercase hexadecimal characters")
    return key.encode("ascii")


def encode(data, key):
    body = json.dumps(data, separators=(",", ":"), allow_nan=False).encode()
    return hmac.new(key, body, hashlib.sha256).hexdigest().encode() + b"\n" + body


def decode(raw, key):
    if len(raw) > 8192:
        raise ValueError("Packet too large")
    sig, body = raw.split(b"\n", 1)
    expected = hmac.new(key, body, hashlib.sha256).hexdigest().encode()
    if not hmac.compare_digest(sig, expected):
        raise ValueError("Invalid packet authentication")
    value = json.loads(body)
    if not isinstance(value, dict):
        raise ValueError("Object required")
    finite(value)
    return value


class QuestGate:
    def __init__(self, key, max_age_s=.2, clock=time.monotonic):
        self.key, self.max_age, self.clock = key, max_age_s, clock
        self.sender = None
        self.sequence = -1
        self.source_time = -1.
        self.challenges = {}

    def reset(self):
        # Arm requires a packet generated after this reset/challenge exchange.
        self.challenges.clear()

    def challenge(self):
        now = self.clock()
        self.challenges = {k: v for k, v in self.challenges.items() if now-v <= self.max_age}
        nonce = secrets.token_hex(12)
        self.challenges[nonce] = now
        return {"protocol": "quest-challenge-v1", "challenge": nonce}

    def accept(self, raw):
        data = decode(raw, self.key)
        if data.get("protocol") != "quest-recording-v1":
            raise ValueError("Unsupported Quest protocol")
        if not isinstance(data.get("sender_id"), str) or not 1 <= len(data["sender_id"]) <= 64:
            raise ValueError("Sender identity required")
        if self.sender is not None and data["sender_id"] != self.sender:
            raise ValueError("Sender changed; restart recording explicitly")
        packet = parse_emcee(data["packet"])
        if type(data.get("tracking_valid")) is not bool:
            raise ValueError("Explicit tracking validity required")
        if type(data.get("synthetic", False)) is not bool:
            raise ValueError("Invalid synthetic evidence flag")
        stamp = data.get("source_time_s")
        if type(stamp) not in (int, float) or stamp < 0:
            raise ValueError("Acquisition monotonic time required")
        if packet["number"] <= self.sequence or stamp <= self.source_time:
            raise ValueError("Duplicate or reordered input")
        issued = self.challenges.get(data.get("challenge"))
        if issued is None or self.clock()-issued > self.max_age:
            raise ValueError("Missing/expired receiver challenge")
        self.sender = data["sender_id"]
        self.sequence, self.source_time = packet["number"], stamp
        return data
