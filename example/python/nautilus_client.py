import json
import os
import threading
import time
import urllib.error
import urllib.request


DEFAULT_NAUTILUS_URL = "http://127.0.0.1:3000/process_data"
DEFAULT_TIMEOUT = 0.5


class NautilusClient:
    def __init__(self, url=None, timeout=DEFAULT_TIMEOUT, enabled=True):
        self.url = url or os.environ.get("NAUTILUS_URL", DEFAULT_NAUTILUS_URL)
        self.timeout = timeout
        self.enabled = enabled and os.environ.get("NAUTILUS_DISABLE", "0") != "1"

    def post_hit_event(self, payload):
        if not self.enabled:
            return None

        body = json.dumps(payload, sort_keys=True).encode("utf-8")
        request = urllib.request.Request(
            self.url,
            data=body,
            headers={"content-type": "application/json"},
            method="POST",
        )

        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            response_body = response.read().decode("utf-8")
        return json.loads(response_body)

    def post_hit_event_async(self, payload):
        if not self.enabled:
            return

        thread = threading.Thread(
            target=self._post_and_print,
            args=(payload,),
            name="nautilus_hit_event",
            daemon=True,
        )
        thread.start()

    def _post_and_print(self, payload):
        try:
            result = self.post_hit_event(payload)
            response = result.get("response", {}) if isinstance(result, dict) else {}
            signature = result.get("signature", "") if isinstance(result, dict) else ""
            payload_hash = response.get("payload_hash", "")
            print(
                "nautilus_mock "
                f"verified={response.get('verified')} "
                f"clean_hit={response.get('clean_hit')} "
                f"payload_hash={payload_hash[:16]}... "
                f"signature={signature[:24]}..."
            )
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            print(f"nautilus_mock unavailable: {exc}")


def now_ms():
    return int(time.time() * 1000)
