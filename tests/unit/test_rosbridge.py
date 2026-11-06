"""Unit tests for RosbridgeClient's connection handling against a fake websocket server.

Regression for a CI flake: the second connection of a pytest session never opened because
``roslibpy.Ros`` was created off the reactor thread while the reactor ran (the TCP connect
was queued without waking the reactor).
"""

import asyncio
import base64
import hashlib
import socket
import struct
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip("roslibpy")

import roslibpy
from twisted.internet import reactor
from twisted.python import threadable

from framework import rosbridge

pytestmark = pytest.mark.unit

WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _recv_exact(conn, n):
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError("closed")
        data += chunk
    return data


def _serve(conn, opened):
    """Accept one websocket handshake (counted in ``opened`` before the answer), then read
    frames until a close (answered)."""
    try:
        request = b""
        while b"\r\n\r\n" not in request:
            chunk = conn.recv(4096)
            if not chunk:
                return
            request += chunk
        key = next(line.split(b":", 1)[1].strip() for line in request.split(b"\r\n")
                   if line.lower().startswith(b"sec-websocket-key:"))
        accept = base64.b64encode(hashlib.sha1(key + WS_GUID).digest())
        # Count the connection before answering: once the client has the answer it can
        # connect, close and assert before this thread runs again (seen in CI under
        # pytest-xdist -n 2, where the runner has two busy workers).
        opened.append(True)
        conn.sendall(b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                     b"Connection: Upgrade\r\nSec-WebSocket-Accept: " + accept + b"\r\n\r\n")
        while True:
            head, size = _recv_exact(conn, 2)
            size &= 0x7F
            if size == 126:
                size = struct.unpack(">H", _recv_exact(conn, 2))[0]
            elif size == 127:
                size = struct.unpack(">Q", _recv_exact(conn, 8))[0]
            mask = _recv_exact(conn, 4)  # client frames are always masked
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(_recv_exact(conn, size)))
            if head & 0x0F == 8:
                conn.sendall(bytes([0x88, len(payload)]) + payload)
                return
    except (ConnectionError, OSError):
        pass
    finally:
        conn.close()


@pytest.fixture
def fake_rosbridge():
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(4)
    opened: list = []

    def accept_loop():
        while True:
            try:
                conn, _ = server.accept()
            except OSError:
                return
            threading.Thread(target=_serve, args=(conn, opened), daemon=True).start()

    threading.Thread(target=accept_loop, daemon=True).start()
    yield server.getsockname()[1], opened
    server.close()


def _cfg(port):
    return SimpleNamespace(
        ros=SimpleNamespace(rosbridge_address="127.0.0.1", rosbridge_port=port),
        timeouts_wall_s=SimpleNamespace(rosbridge_connect=5.0))


def test_sequential_connections_are_made_on_the_reactor_thread(fake_rosbridge, tmp_path,
                                                               monkeypatch):
    port, opened = fake_rosbridge
    created_in_io_thread = []

    class RecordingRos(roslibpy.Ros):
        def __init__(self, *args, **kwargs):
            created_in_io_thread.append(threadable.isInIOThread())
            super().__init__(*args, **kwargs)

    monkeypatch.setattr(rosbridge.roslibpy, "Ros", RecordingRos)

    async def two_sessions():
        for i in range(2):
            client = rosbridge.RosbridgeClient(_cfg(port), f"t{i}", tmp_path / str(i))
            await client.connect()
            client.close()
            assert reactor.running, "close() must not stop the reactor"

    asyncio.run(two_sessions())
    assert len(opened) == 2
    assert created_in_io_thread == [True, True]


def test_connect_timeout_raises_and_leaves_the_reactor_running(tmp_path):
    with socket.socket() as probe:  # a port with no listener: every attempt is refused
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    cfg = _cfg(port)
    cfg.timeouts_wall_s.rosbridge_connect = 0.5
    client = rosbridge.RosbridgeClient(cfg, "t", tmp_path)
    with pytest.raises(rosbridge.FlightAssertionError, match="connected within 0.5 s"):
        asyncio.run(client.connect())
    assert reactor.running
