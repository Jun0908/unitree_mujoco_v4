import argparse
import hashlib
import hmac
import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 3000
DEFAULT_SECRET = "local-hackathon-mock-secret"


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def point_to_segment_distance(point, seg_a, seg_b):
    segment = [b - a for a, b in zip(seg_a, seg_b)]
    point_to_a = [p - a for p, a in zip(point, seg_a)]
    denom = sum(value * value for value in segment)
    if denom < 1e-12:
        return sum(value * value for value in point_to_a) ** 0.5

    t = sum(a * b for a, b in zip(point_to_a, segment)) / denom
    t = min(max(t, 0.0), 1.0)
    closest = [a + t * value for a, value in zip(seg_a, segment)]
    return sum((p - c) ** 2 for p, c in zip(point, closest)) ** 0.5


def recompute_trace_hit(payload):
    trace = payload.get("trace") or []
    hit_distance = float(payload.get("hit_distance", 0.24))
    best_hand = None
    best_forearm = None

    for frame in trace:
        fist = frame.get("fist")
        elbow = frame.get("elbow")
        head = frame.get("head")
        if not (isinstance(fist, list) and isinstance(elbow, list) and isinstance(head, list)):
            continue
        if not (len(fist) == len(elbow) == len(head) == 3):
            continue

        hand_distance = sum((float(a) - float(b)) ** 2 for a, b in zip(fist, head)) ** 0.5
        forearm_distance = point_to_segment_distance(
            [float(value) for value in head],
            [float(value) for value in elbow],
            [float(value) for value in fist],
        )

        best_hand = hand_distance if best_hand is None else min(best_hand, hand_distance)
        best_forearm = (
            forearm_distance
            if best_forearm is None
            else min(best_forearm, forearm_distance)
        )

    if best_hand is None or best_forearm is None:
        return None

    return {
        "clean_hit": min(best_hand, best_forearm) <= hit_distance,
        "best_hand_distance": best_hand,
        "best_forearm_distance": best_forearm,
    }


def verify_payload(payload):
    claimed_clean_hit = bool(payload.get("clean_hit"))
    marker_changed = bool(payload.get("marker_color_changed"))
    hit_count_ok = int(payload.get("hit_count", 0)) > 0
    hit_distance = float(payload.get("hit_distance", 0.24))
    hand_distance = float(payload.get("hand_distance", 999.0))
    forearm_distance = float(payload.get("forearm_distance", 999.0))
    event_distance_ok = min(hand_distance, forearm_distance) <= hit_distance

    trace_check = recompute_trace_hit(payload)
    if trace_check is None:
        trace_distance_ok = True
    else:
        trace_distance_ok = trace_check["clean_hit"]

    verified = (
        payload.get("motion") == "seiken"
        and claimed_clean_hit
        and marker_changed
        and hit_count_ok
        and event_distance_ok
        and trace_distance_ok
    )

    return verified, trace_check


class NautilusMockHandler(BaseHTTPRequestHandler):
    server_version = "MockNautilus/0.1"

    def log_message(self, fmt, *args):
        print(f"mock_nautilus {self.address_string()} - {fmt % args}")

    def do_GET(self):
        if self.path == "/health":
            self.write_json(200, {"ok": True, "service": "mock-nautilus"})
            return
        self.write_json(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/process_data":
            self.write_json(404, {"error": "not found"})
            return

        try:
            length = int(self.headers.get("content-length", "0"))
            raw_body = self.rfile.read(length)
            payload = json.loads(raw_body.decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("payload must be a JSON object")

            if self.server.verbose:
                print("mock_nautilus received payload:")
                print(json.dumps(payload, indent=2, sort_keys=True))

            verified, trace_check = verify_payload(payload)
            payload_hash = hashlib.sha256(canonical_json(payload)).hexdigest()
            response = {
                "session_id": payload.get("session_id"),
                "motion": payload.get("motion"),
                "verified": verified,
                "clean_hit": bool(payload.get("clean_hit")),
                "hit_count": int(payload.get("hit_count", 0)),
                "payload_hash": payload_hash,
                "timestamp_ms": int(time.time() * 1000),
                "trace_check": trace_check,
                "message": "verified by mock Nautilus",
            }
            signature = hmac.new(
                self.server.secret.encode("utf-8"),
                canonical_json(response),
                hashlib.sha256,
            ).hexdigest()
            result = {
                "response": response,
                "signature": f"mock_hmac_sha256:{signature}",
            }

            print(
                "mock_nautilus verified="
                f"{verified} clean_hit={response['clean_hit']} "
                f"payload_hash={payload_hash[:16]}..."
            )
            self.write_json(200, result)
        except Exception as exc:
            self.write_json(400, {"error": str(exc)})

    def write_json(self, status, value):
        body = json.dumps(value, indent=2, sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class NautilusMockServer(ThreadingHTTPServer):
    def __init__(self, server_address, request_handler_class, secret, verbose=False):
        super().__init__(server_address, request_handler_class)
        self.secret = secret
        self.verbose = verbose


def parse_args():
    parser = argparse.ArgumentParser(description="Run a local mock Nautilus server.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--secret", default=DEFAULT_SECRET)
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print the full JSON payload received from the visualizer.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    server = NautilusMockServer(
        (args.host, args.port),
        NautilusMockHandler,
        secret=args.secret,
        verbose=args.verbose,
    )
    print(f"mock Nautilus listening on http://{args.host}:{args.port}/process_data")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("mock Nautilus stopped.")


if __name__ == "__main__":
    main()
