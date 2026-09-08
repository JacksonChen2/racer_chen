import json
import threading
import time

from agentic_crpo.single_gpu_pause import (
    SingleGpuPauseConfig,
    SingleGpuPauseController,
)


def _atomic_json(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload), encoding="utf-8")
    temporary.replace(path)


def test_pause_controller_waits_for_ack_and_resume(tmp_path):
    request_path = tmp_path / "pause_request.json"
    acknowledgement_path = tmp_path / "pause_ack.json"
    controller = SingleGpuPauseController(
        SingleGpuPauseConfig(
            request_path=request_path,
            acknowledgement_path=acknowledgement_path,
            timeout_s=2.0,
            poll_interval_s=0.001,
        )
    )
    observed = []

    def fake_isaac():
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not request_path.exists():
            time.sleep(0.001)
        request = json.loads(request_path.read_text(encoding="utf-8"))
        observed.append("paused")
        _atomic_json(
            acknowledgement_path,
            {"token": request["token"], "status": "paused"},
        )
        while request_path.exists():
            time.sleep(0.001)
        observed.append("resumed")
        _atomic_json(
            acknowledgement_path,
            {"token": request["token"], "status": "resumed"},
        )

    worker = threading.Thread(target=fake_isaac)
    worker.start()
    with controller.paused():
        assert observed == ["paused"]
    worker.join(timeout=1.0)

    assert observed == ["paused", "resumed"]
    assert controller.pause_count == 1
    assert not request_path.exists()


def test_pause_controller_cancel_unblocks_pending_request(tmp_path):
    request_path = tmp_path / "pause_request.json"
    controller = SingleGpuPauseController(
        SingleGpuPauseConfig(
            request_path=request_path,
            acknowledgement_path=tmp_path / "pause_ack.json",
            timeout_s=30.0,
            poll_interval_s=0.001,
        )
    )
    errors = []

    def request_pause():
        try:
            controller.acquire()
        except RuntimeError as error:
            errors.append(str(error))

    worker = threading.Thread(target=request_pause)
    worker.start()
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not request_path.exists():
        time.sleep(0.001)
    assert request_path.exists()

    controller.cancel_pending()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert errors == ["single-GPU pause request was cancelled"]
    assert not request_path.exists()
