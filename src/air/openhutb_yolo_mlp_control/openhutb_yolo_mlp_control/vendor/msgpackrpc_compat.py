"""Modern lightweight MessagePack-RPC client for OpenHUTB/AirSim.

Implements the subset used by AirSim PythonClient:
    Address
    Client.call()
    Client.call_async()
    Future.join()/get()

Unlike the earlier compatibility shim, call_async() is genuinely asynchronous:
it sends the request immediately and returns a Future without waiting for the
server response. A background reader thread dispatches responses by msgid.
"""

from __future__ import annotations

import socket
import threading
from dataclasses import dataclass
from typing import Any

import msgpack


@dataclass(frozen=True)
class Address:
    host: str
    port: int


class Future:
    def __init__(self):
        self._event = threading.Event()
        self._result = None
        self._error = None

    def _set_result(self, result):
        self._result = result
        self._event.set()

    def _set_error(self, error):
        self._error = error
        self._event.set()

    def done(self):
        return self._event.is_set()

    def join(self):
        self._event.wait()
        if self._error is not None:
            raise self._error
        return self._result

    def get(self):
        return self.join()


class Client:
    def __init__(
        self,
        address: Address,
        timeout: float = 3600,
        pack_encoding: str = "utf-8",
        unpack_encoding: str = "utf-8",
        **_kwargs,
    ):
        self.address = address
        self.timeout = float(timeout)
        self.pack_encoding = pack_encoding
        self.unpack_encoding = unpack_encoding

        self._sock = None
        self._unpacker = msgpack.Unpacker(raw=False)
        self._msgid = 0

        self._state_lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._pending = {}

        self._reader_thread = None
        self._reader_stop = threading.Event()

    @staticmethod
    def _default(obj):
        if hasattr(obj, "to_msgpack"):
            return obj.to_msgpack()

        if hasattr(obj, "item") and callable(obj.item):
            try:
                return obj.item()
            except Exception:
                pass

        if hasattr(obj, "__dict__"):
            return obj.__dict__

        raise TypeError(
            f"Object of type {type(obj).__name__} "
            "is not MessagePack serializable"
        )

    def _connect(self):
        with self._state_lock:
            if self._sock is not None:
                return

            sock = socket.create_connection(
                (
                    self.address.host,
                    int(self.address.port),
                ),
                timeout=self.timeout,
            )

            # The reader uses a short timeout only so close() can stop it.
            sock.settimeout(0.5)

            self._sock = sock
            self._unpacker = msgpack.Unpacker(raw=False)
            self._reader_stop.clear()

            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                name="msgpackrpc-reader",
                daemon=True,
            )
            self._reader_thread.start()

    def _fail_all_pending(self, exc):
        with self._state_lock:
            pending = list(self._pending.values())
            self._pending.clear()

        for future in pending:
            future._set_error(exc)

    def _reset_transport(self):
        with self._state_lock:
            sock = self._sock
            self._sock = None

        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    def close(self):
        self._reader_stop.set()
        self._reset_transport()

        thread = self._reader_thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=1.0)

        self._reader_thread = None

        self._fail_all_pending(
            ConnectionError("MessagePack-RPC client closed")
        )

    def _reader_loop(self):
        try:
            while not self._reader_stop.is_set():
                with self._state_lock:
                    sock = self._sock

                if sock is None:
                    return

                try:
                    chunk = sock.recv(1024 * 1024)
                except socket.timeout:
                    continue

                if not chunk:
                    raise ConnectionError(
                        "AirSim RPC connection closed by simulator"
                    )

                self._unpacker.feed(chunk)

                for message in self._unpacker:
                    if (
                        not isinstance(message, (list, tuple))
                        or len(message) < 4
                    ):
                        continue

                    msg_type, msgid, error, result = message[:4]

                    if msg_type != 1:
                        continue

                    with self._state_lock:
                        future = self._pending.pop(
                            msgid,
                            None,
                        )

                    if future is None:
                        continue

                    if error not in (
                        None,
                        False,
                        "",
                        b"",
                    ):
                        future._set_error(
                            RuntimeError(
                                f"AirSim RPC error: {error}"
                            )
                        )
                    else:
                        future._set_result(result)

        except Exception as exc:
            self._reset_transport()
            self._fail_all_pending(exc)

    def call_async(self, method: str, *params):
        self._connect()

        future = Future()

        with self._state_lock:
            self._msgid += 1
            msgid = self._msgid
            self._pending[msgid] = future
            sock = self._sock

        packet = [
            0,
            msgid,
            method,
            list(params),
        ]

        payload = msgpack.packb(
            packet,
            default=self._default,
            use_bin_type=True,
        )

        try:
            with self._send_lock:
                if sock is None:
                    raise ConnectionError(
                        "AirSim RPC socket is not connected"
                    )
                sock.sendall(payload)
        except Exception as exc:
            with self._state_lock:
                self._pending.pop(
                    msgid,
                    None,
                )

            future._set_error(exc)
            self._reset_transport()

        return future

    def call(self, method: str, *params):
        return self.call_async(
            method,
            *params,
        ).join()
