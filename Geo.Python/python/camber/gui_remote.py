"""Optional local GUI automation bridge; standard library plus the GUI's pyglet.

Create GuiControlServer on the GUI thread. A separate process can use
GuiControlClient with its connection dictionary. Input uses GuiDriver's normal
window events; this module has no CAD dependencies.
"""

import base64
from concurrent.futures import Future
from dataclasses import asdict
import hmac
import io
import json
from pathlib import Path
from queue import Empty, Queue
import secrets
import socket
import socketserver
import threading


_INPUTS = frozenset(("click", "move", "drag", "key", "text", "scroll"))
_REQUEST_LIMIT = 65536
_RESPONSE_LIMIT = 32 * 1024 * 1024
_TIMEOUT = 120


class GuiControlServer:
    def __init__(self, driver):
        import pyglet
        self.driver = driver
        self._clock = pyglet.clock
        self._pending = Queue()
        self._closed = threading.Event()
        self._token = secrets.token_urlsafe(32)
        owner = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.request.settimeout(5)
                try:
                    line = self.rfile.readline(_REQUEST_LIMIT+1)
                    if len(line) > _REQUEST_LIMIT or not line.endswith(b"\n"):
                        raise ValueError("Invalid or oversized request")
                    request = json.loads(line)
                    token = request.get("token")
                    if not isinstance(token, str) or not hmac.compare_digest(token, owner._token):
                        raise ValueError("Invalid GUI control token")
                    command, arguments = request.get("command"), request.get("arguments", {})
                    if command not in _INPUTS | {"controls", "screenshot"} or not isinstance(arguments, dict):
                        raise ValueError("Unknown command or invalid arguments")
                    future = Future()
                    if owner._closed.is_set():
                        raise RuntimeError("GUI control server is closed")
                    owner._pending.put((command, arguments, future))
                    if owner._closed.is_set():
                        future.cancel()
                    try:
                        result = future.result(timeout=_TIMEOUT)
                    finally:
                        future.cancel()  # Do not run a timed-out command still in the queue.
                    response = {"result": result}
                except Exception as exc:
                    response = {"error": f"{type(exc).__name__}: {exc}"}
                try:
                    self.wfile.write(json.dumps(response).encode()+b"\n")
                except OSError:
                    pass  # The client may have disconnected while the GUI was busy.

        self._server = socketserver.TCPServer(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        kwargs={"poll_interval": .05}, daemon=True)
        self._clock.schedule_interval(self._poll, 1/60)
        self._thread.start()

    @property
    def connection(self):
        host, port = self._server.server_address
        return {"host": host, "port": port, "token": self._token}

    def _poll(self, dt):
        try:
            command, arguments, future = self._pending.get_nowait()
        except Empty:
            return
        if not future.set_running_or_notify_cancel():
            return
        try:
            if command in _INPUTS:
                result = getattr(self.driver, command)(**arguments)
            elif command == "controls":
                if arguments:
                    raise ValueError("controls takes no arguments")
                self.driver.frame()
                result = [asdict(control) for control in self.driver.ui.controls]
            else:
                if arguments:
                    raise ValueError("screenshot takes no arguments")
                output = io.BytesIO()
                self.driver.screenshot().save("screenshot.png", file=output)
                result = base64.b64encode(output.getvalue()).decode("ascii")
            future.set_result(result)
        except Exception as exc:
            future.set_exception(exc)

    def close(self):
        """Stop polling and listening; call on the GUI thread before closing it."""
        if self._closed.is_set():
            return
        self._closed.set()
        self._clock.unschedule(self._poll)
        while True:
            try:
                self._pending.get_nowait()[2].cancel()
            except Empty:
                break
        self._server.shutdown()
        self._server.server_close()
        self._thread.join()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class GuiControlClient:
    """Send input to a running GUI. Only screenshots are written by this client."""
    def __init__(self, *, host="127.0.0.1", port, token):
        if host != "127.0.0.1":
            raise ValueError("GUI control is restricted to the local machine")
        self.address = (host, port)
        self.token = token

    @classmethod
    def from_file(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))

    def command(self, command, **arguments):
        request = json.dumps({"token": self.token, "command": command,
                              "arguments": arguments}).encode()+b"\n"
        if len(request) > _REQUEST_LIMIT:
            raise ValueError("GUI command is too large")
        with socket.create_connection(self.address, timeout=_TIMEOUT) as connection:
            connection.sendall(request)
            with connection.makefile("rb") as response_file:
                response = response_file.readline(_RESPONSE_LIMIT+1)
        if len(response) > _RESPONSE_LIMIT or not response.endswith(b"\n"):
            raise RuntimeError("Invalid or oversized GUI response")
        result = json.loads(response)
        if "error" in result:
            raise RuntimeError(result["error"])
        return result["result"]

    def controls(self):
        return self.command("controls")

    def screenshot(self, path=None):
        png = base64.b64decode(self.command("screenshot"), validate=True)
        if path is not None:
            Path(path).write_bytes(png)
        return png

    def click(self, label=None, **arguments):
        return self.command("click", label=label, **arguments)

    def text(self, text, *, replace=False):
        return self.command("text", text=text, replace=replace)

    def key(self, symbol, *, modifiers=0):
        return self.command("key", symbol=symbol, modifiers=modifiers)

    def move(self, x, y):
        return self.command("move", x=x, y=y)

    def scroll(self, x, y, amount):
        return self.command("scroll", x=x, y=y, amount=amount)

    def drag(self, start, end, *, steps=8):
        return self.command("drag", start=start, end=end, steps=steps)
