"""Test-side ROS client over rosbridge: topics, services, recordings.

Imports ``roslibpy`` at module top, so only the ROS tests import this module (the
fixtures import it lazily).

roslibpy runs one Twisted reactor per process, which cannot be restarted: the client
calls ``Ros.close()`` and never ``terminate()``. The reactor runs in a daemon thread
started once per process (``_ensure_reactor``). Twisted is not thread-safe, so a
``Ros`` is created and closed on the reactor thread (``_on_reactor``): creating it
elsewhere while the reactor already runs queues the TCP connect without waking the
reactor, so the second and later connections of a session never start. Blocking
calls run in ``asyncio.to_thread``; topic callbacks arrive on the reactor thread.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
import time
from pathlib import Path

import roslibpy
from roslibpy.comm import RosBridgeClientFactory
from twisted.internet import reactor, threads
from twisted.python import log as twisted_log

from framework.assertions import FlightAssertionError, wait_until
from framework.config import Config

log = logging.getLogger(__name__)

RETRY_MAX_DELAY_S = 1.0   # between refused connection attempts
REACTOR_START_S = 10.0    # the reactor thread reports running within this

_reactor_lock = threading.Lock()


def _ensure_reactor() -> None:
    """Start the process's Twisted reactor in a daemon thread unless it already runs."""
    with _reactor_lock:
        if reactor.running:
            return
        running = threading.Event()
        reactor.callWhenRunning(running.set)
        twisted_log.PythonLoggingObserver().start()  # Twisted errors into ``logging``
        threading.Thread(target=reactor.run, kwargs={"installSignalHandlers": False},
                         name="twisted-reactor", daemon=True).start()
        if not running.wait(REACTOR_START_S):
            raise RuntimeError("the Twisted reactor did not start (it cannot be restarted "
                               "once stopped: never call Ros.terminate())")


def _on_reactor(fn, *args, **kwargs):
    """Run ``fn`` on the reactor thread and return its result (blocks the caller)."""
    return threads.blockingCallFromThread(reactor, fn, *args, **kwargs)


def record_name(topic: str) -> str:
    """``/survey/status`` -> ``survey__status``: the recording file name of a topic."""
    return topic.lstrip("/").replace("/", "__")


def _json_safe(value):
    """NaN and infinities become null, so every recorded line is strict JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_json_safe(v) for v in value]
    return value


class RosbridgeClient:
    """One rosbridge connection; each subscribed topic is recorded to ``record_dir``."""

    def __init__(self, cfg: Config, test_id: str, record_dir: Path):
        self.cfg = cfg
        self.test_id = test_id
        self.record_dir = Path(record_dir)
        self._ros: roslibpy.Ros | None = None
        self._topics: dict[str, roslibpy.Topic] = {}
        self._latest: dict[str, dict] = {}
        self._counts: dict[str, int] = {}
        self._files: dict = {}
        self._lock = threading.Lock()

    async def connect(self) -> None:
        """Connect within ``rosbridge_connect``.

        One ``Ros`` per connection, created on the reactor thread: its factory retries a
        refused connection by itself (every ``RETRY_MAX_DELAY_S`` at most); on timeout it
        is told to stop trying.
        """
        ros_cfg = self.cfg.ros
        timeout_s = self.cfg.timeouts_wall_s.rosbridge_connect
        self.record_dir.mkdir(parents=True, exist_ok=True)
        RosBridgeClientFactory.set_max_delay(RETRY_MAX_DELAY_S)
        await asyncio.to_thread(_ensure_reactor)
        ros = await asyncio.to_thread(_on_reactor, roslibpy.Ros,
                                      host=ros_cfg.rosbridge_address, port=ros_cfg.rosbridge_port)
        ready = threading.Event()
        await asyncio.to_thread(_on_reactor, ros.factory.on_ready, lambda _proto: ready.set())
        if not await asyncio.to_thread(ready.wait, timeout_s):
            await asyncio.to_thread(self._close_quietly, ros)
            raise FlightAssertionError(
                f"rosbridge at {ros_cfg.rosbridge_address}:{ros_cfg.rosbridge_port} "
                f"connected within {timeout_s} s", "no websocket connection opened")
        self._ros = ros

    @staticmethod
    def _close_quietly(ros) -> None:
        def close():
            ros.factory.stopTrying()  # no reconnect after a close or a failed connect
            ros.close()

        try:
            if reactor.running:
                _on_reactor(close)
            else:
                close()
        except Exception:
            log.debug("closing the rosbridge connection failed", exc_info=True)

    def _require(self) -> roslibpy.Ros:
        if self._ros is None:
            raise RuntimeError("RosbridgeClient is not connected")
        return self._ros

    def subscribe(self, topic: str, msg_type: str) -> None:
        """Subscribe, record every message and keep the latest one."""
        ros = self._require()
        path = self.record_dir / f"{record_name(topic)}.jsonl"
        handle = open(path, "a")  # noqa: SIM115 - closed in close()/reconnect()

        def on_message(message: dict) -> None:
            line = json.dumps({"wall_t": time.monotonic(), "msg": _json_safe(message)})
            with self._lock:
                self._latest[topic] = message
                self._counts[topic] = self._counts.get(topic, 0) + 1
                handle.write(line + "\n")
                handle.flush()

        listener = roslibpy.Topic(ros, topic, msg_type)
        with self._lock:
            self._files[topic] = handle
            self._topics[topic] = listener
        listener.subscribe(on_message)

    def latest(self, topic: str) -> dict | None:
        with self._lock:
            return self._latest.get(topic)

    def count(self, topic: str) -> int:
        with self._lock:
            return self._counts.get(topic, 0)

    async def call_service(self, name: str, srv_type: str, request: dict,
                           timeout_s: float) -> dict:
        """Call a service; a timeout or a failed call raises ``FlightAssertionError``."""
        ros = self._require()
        done = threading.Event()
        result: dict = {}

        def on_result(response):
            result["response"] = dict(response)
            done.set()

        def on_error(error):
            result["error"] = error
            done.set()

        service = roslibpy.Service(ros, name, srv_type)
        service.call(roslibpy.ServiceRequest(request), on_result, on_error)
        finished = await asyncio.to_thread(done.wait, timeout_s)
        if not finished:
            raise FlightAssertionError(f"{name} answered within {timeout_s} s", "no reply")
        if "error" in result:
            raise FlightAssertionError(f"{name} succeeded", f"error: {result['error']}")
        return result["response"]

    def _drop_subscriptions(self) -> None:
        with self._lock:
            topics, files = list(self._topics.values()), list(self._files.values())
            self._topics.clear()
            self._files.clear()
            self._latest.clear()
            self._counts.clear()
        for listener in topics:
            try:
                listener.unsubscribe()
            except Exception:
                log.debug("unsubscribing %s failed", listener.name, exc_info=True)
        for handle in files:
            handle.close()

    async def reconnect(self, record_dir: Path) -> None:
        """Close, clear subscriptions and latest values, and connect again (replay)."""
        await asyncio.to_thread(self.close)
        self.record_dir = Path(record_dir)
        await self.connect()

    def close(self) -> None:
        """Unsubscribe, close the recordings and the connection; never raises."""
        self._drop_subscriptions()
        if self._ros is not None:
            self._close_quietly(self._ros)
            self._ros = None


async def wait_for_topic(client: RosbridgeClient, topic: str, predicate, timeout_s: float,
                         description: str) -> dict:
    """The first latest message of ``topic`` for which ``predicate`` holds."""

    def ready():
        message = client.latest(topic)
        return message if message is not None and predicate(message) else None

    try:
        return await wait_until(ready, timeout_s, description)
    except FlightAssertionError:
        last = client.latest(topic)
        shown = json.dumps(_json_safe(last)) if last is not None else "no message received"
        raise FlightAssertionError(f"{description} within {timeout_s} s",
                                   f"latest {topic}: {shown}") from None
