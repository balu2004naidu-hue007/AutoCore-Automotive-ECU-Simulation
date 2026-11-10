#!/usr/bin/env python3
"""Local-only AutoCore dashboard bridge. C++ remains the simulation source of truth."""
from __future__ import annotations

import argparse
from collections import deque
import csv
import json
import subprocess
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
HTML = ROOT / "dashboard" / "index.html"

class EngineBridge:
    def __init__(self, executable: Path):
        self.executable = executable
        self.lock = threading.RLock()
        self.process: subprocess.Popen[str] | None = None
        self.last_state: dict[str, Any] = {}
        self.last_error = ""
        self.stderr_tail: deque[str] = deque(maxlen=30)
        self.stop_event = threading.Event()
        self.stderr_reader: threading.Thread | None = None
        self.ticker: threading.Thread | None = None
        try:
            self._launch()
            self._command("STATE")
            self.ticker = threading.Thread(target=self._tick_loop, name="AutoCoreTicker", daemon=True)
            self.ticker.start()
        except Exception:
            self.close()
            raise

    def _launch(self) -> None:
        if not self.executable.is_file():
            raise FileNotFoundError(f"C++ engine executable not found: {self.executable}")
        self.process = subprocess.Popen(
            [str(self.executable), "--can-matrix", str(ROOT / "config" / "can_messages.csv")],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, encoding="utf-8", errors="replace", bufsize=1,
            cwd=str(ROOT),
        )
        self.stderr_reader = threading.Thread(
            target=self._drain_stderr, name="AutoCoreStderr", daemon=True
        )
        self.stderr_reader.start()

    def _drain_stderr(self) -> None:
        process = self.process
        if process is None or process.stderr is None:
            return
        for line in process.stderr:
            text = line.rstrip()
            if text:
                with self.lock:
                    self.stderr_tail.append(text)

    def _command(self, command: str) -> dict[str, Any]:
        with self.lock:
            if self.process is None or self.process.poll() is not None:
                raise RuntimeError("AutoCore C++ engine is not running")
            assert self.process.stdin is not None and self.process.stdout is not None
            self.process.stdin.write(command + "\n")
            self.process.stdin.flush()
            line = self.process.stdout.readline()
            if not line:
                error = "\n".join(self.stderr_tail)
                detail = (" Diagnostic output: " + error) if error else ""
                self.last_error = "C++ engine closed its output stream." + detail
                raise RuntimeError(self.last_error)
            try:
                state = json.loads(line)
            except json.JSONDecodeError as exc:
                self.last_error = f"C++ engine returned invalid JSON state: {exc}"
                raise RuntimeError(self.last_error) from exc
            if not isinstance(state, dict):
                self.last_error = "C++ engine state root must be a JSON object"
                raise RuntimeError(self.last_error)
            self.last_state = state
            self.last_error = ""
            return self.last_state

    def state(self) -> dict[str, Any]:
        return self._command("STATE")

    def action(self, command: str) -> dict[str, Any]:
        return self._command(command)

    def _tick_loop(self) -> None:
        while not self.stop_event.wait(0.1):
            try:
                self._command("TICK")
            except Exception as exc:  # surface failure through health/state endpoint
                with self.lock:
                    self.last_error = str(exc)
                return

    def close(self) -> None:
        # Stop producers before closing any pipe. The stderr reader must be
        # allowed to observe EOF and exit before its stream is closed by us.
        self.stop_event.set()
        if self.ticker and self.ticker.is_alive():
            self.ticker.join(timeout=2.0)

        with self.lock:
            process = self.process
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)

        if self.stderr_reader and self.stderr_reader.is_alive():
            self.stderr_reader.join(timeout=2.0)

        with self.lock:
            process = self.process
            if process:
                for stream in (process.stdin, process.stdout, process.stderr):
                    if stream and not stream.closed:
                        try:
                            stream.close()
                        except OSError:
                            pass


def make_handler(bridge: EngineBridge):
    allowed_simple = {
        "start": "START", "pause": "PAUSE", "reset": "RESET",
        "fault_overtemp": "FAULT:OVER_TEMP",
        "fault_low_voltage": "FAULT:LOW_VOLTAGE",
        "fault_sensor_demo": "FAULT:SENSOR_DEMO",
        "fault_engine_offline": "FAULT:ENGINE_OFFLINE",
        "fault_battery_offline": "FAULT:BATTERY_OFFLINE",
        "fault_crc": "FAULT:CRC", "fault_counter": "FAULT:COUNTER", "recover_all": "RECOVER:ALL",
        "toggle_lights": "LIGHTS:TOGGLE", "toggle_door": "DOOR:TOGGLE",
        "read_dtcs": "UDS:DTC", "clear_dtcs": "UDS:CLEAR",
        "uds_session": "UDS:SESSION", "uds_reset": "UDS:RESET",
        "tester_present": "UDS:TESTER",
    }
    dtc_code_map = {
        "P0217": "000217", "P0562": "000562", "U0100": "000100",
        "U0111": "000111", "AC0001": "A00001", "AC0002": "A00002",
    }
    did_map = {"F40C", "F405", "F411"}

    class Handler(BaseHTTPRequestHandler):
        server_version = "AutoCoreDashboard/1.6.1"

        def log_message(self, fmt: str, *args: object) -> None:
            print("[dashboard] " + (fmt % args), flush=True)

        def _send(self, status: int, payload: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(payload)

        def _json(self, status: int, obj: Any) -> None:
            self._send(status, json.dumps(obj, separators=(",", ":")).encode("utf-8"), "application/json; charset=utf-8")

        def _read_body(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 8192:
                raise ValueError("Request body must be between 1 and 8192 bytes")
            content_type = self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower()
            if content_type != "application/json":
                raise ValueError("Content-Type must be application/json")
            raw = self.rfile.read(length)
            value = json.loads(raw.decode("utf-8"))
            if not isinstance(value, dict):
                raise ValueError("JSON request must be an object")
            return value

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            try:
                if path in ("/", "/index.html"):
                    self._send(200, HTML.read_bytes(), "text/html; charset=utf-8")
                elif path == "/api/state":
                    self._json(200, bridge.state())
                elif path == "/api/health":
                    with bridge.lock:
                        process_running = bool(bridge.process and bridge.process.poll() is None)
                        error = bridge.last_error
                        stderr_tail = list(bridge.stderr_tail)[-8:]
                    self._json(200, {"ok": process_running and not bool(error),
                        "engine": "running" if process_running else "stopped",
                        "error": error, "diagnostics": stderr_tail})
                elif path == "/api/export.csv":
                    state = bridge.state()
                    from io import StringIO
                    buff = StringIO()
                    writer = csv.writer(buff)
                    writer.writerow(["time_ms", "rpm", "temperature_c", "speed_kph", "voltage_v", "current_a"])
                    for item in state.get("history", []):
                        writer.writerow([item.get(k, "") for k in ("time_ms", "rpm", "temperature_c", "speed_kph", "voltage_v", "current_a")])
                    data = buff.getvalue().encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/csv; charset=utf-8")
                    self.send_header("Content-Disposition", 'attachment; filename="autocore_telemetry.csv"')
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                else:
                    self._json(404, {"error": "Not found"})
            except Exception as exc:
                self._json(500, {"error": str(exc)})

        def do_POST(self) -> None:  # noqa: N802
            if urlparse(self.path).path != "/api/action":
                self._json(404, {"error": "Not found"})
                return
            try:
                body = self._read_body()
                action = str(body.get("action", ""))
                if action in allowed_simple:
                    state = bridge.action(allowed_simple[action])
                elif action == "set_throttle":
                    value = body.get("value")
                    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 100:
                        raise ValueError("Throttle must be an integer from 0 to 100")
                    state = bridge.action(f"THROTTLE:{value}")
                elif action == "read_did":
                    did = str(body.get("did", "")).upper()
                    if did not in did_map:
                        raise ValueError("Unsupported DID")
                    state = bridge.action("UDS:DID:" + did)
                elif action == "read_freeze_frame":
                    code = str(body.get("code", "")).upper()
                    if code not in dtc_code_map:
                        raise ValueError("Unsupported DTC for freeze-frame retrieval")
                    state = bridge.action("UDS:FREEZE:" + dtc_code_map[code])
                elif action == "set_can_filter":
                    import re
                    can_id = str(body.get("can_id", "")).upper().removeprefix("0X")
                    mask = str(body.get("mask", "7FF")).upper().removeprefix("0X")
                    if not re.fullmatch(r"[0-9A-F]{1,3}", can_id):
                        raise ValueError("CAN ID must be 1 to 3 hexadecimal digits")
                    if not re.fullmatch(r"[0-9A-F]{1,3}", mask):
                        raise ValueError("CAN mask must be 1 to 3 hexadecimal digits")
                    if int(can_id, 16) > 0x7FF or int(mask, 16) > 0x7FF:
                        raise ValueError("CAN ID and mask must fit the 11-bit standard identifier range")
                    state = bridge.action(f"CAN_FILTER:{can_id}:{mask}")
                elif action == "clear_can_filter":
                    state = bridge.action("CAN_FILTER:CLEAR")
                elif action in {"can_tx_error_burst", "can_rx_error_burst"}:
                    node = body.get("node")
                    count = body.get("count")
                    valid_nodes = {"ENGINE", "BATTERY", "BODY", "DIAGNOSTICS"}
                    if not isinstance(node, str) or node.upper() not in valid_nodes:
                        raise ValueError("CAN controller node must be ENGINE, BATTERY, BODY or DIAGNOSTICS")
                    maximum = 64 if action == "can_tx_error_burst" else 255
                    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= maximum:
                        raise ValueError(f"Error burst count must be an integer from 1 to {maximum}")
                    operation = "TX_BURST" if action == "can_tx_error_burst" else "RX_BURST"
                    state = bridge.action(f"CAN_CONTROLLER:{operation}:{node.upper()}:{count}")
                elif action in {"can_tx_error_injection", "can_rx_error_injection"}:
                    node = body.get("node")
                    enabled = body.get("enabled")
                    valid_nodes = {"ENGINE", "BATTERY", "BODY", "DIAGNOSTICS"}
                    if not isinstance(node, str) or node.upper() not in valid_nodes:
                        raise ValueError("CAN controller node must be ENGINE, BATTERY, BODY or DIAGNOSTICS")
                    if not isinstance(enabled, bool):
                        raise ValueError("Controller error injection enabled must be a boolean")
                    operation = "TX_INJECT" if action == "can_tx_error_injection" else "RX_INJECT"
                    state = bridge.action(f"CAN_CONTROLLER:{operation}:{node.upper()}:{'ON' if enabled else 'OFF'}")
                elif action == "can_bus_off_recovery":
                    node = body.get("node")
                    valid_nodes = {"ENGINE", "BATTERY", "BODY", "DIAGNOSTICS"}
                    if not isinstance(node, str) or node.upper() not in valid_nodes:
                        raise ValueError("CAN controller node must be ENGINE, BATTERY, BODY or DIAGNOSTICS")
                    current = bridge.state()
                    controllers = current.get("can", {}).get("controller_health", {}).get("controllers", [])
                    selected = next((item for item in controllers if item.get("key") == node.upper()), None)
                    if selected is None or selected.get("state") != "BUS_OFF":
                        raise ValueError("Selected controller must be BUS_OFF before recovery can start")
                    if any(item.get("tx_error_injection") is True for item in controllers):
                        raise ValueError("Disable all continuous TX error injectors across the network before recovery")
                    if selected.get("recovery_in_progress") is True:
                        raise ValueError("Bus-Off recovery is already in progress for the selected controller")
                    state = bridge.action(f"CAN_CONTROLLER:RECOVER:{node.upper()}")
                else:
                    raise ValueError("Unsupported action")
                self._json(200, state)
            except (ValueError, json.JSONDecodeError) as exc:
                self._json(400, {"error": str(exc)})
            except Exception as exc:
                self._json(500, {"error": str(exc)})

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="AutoCore live local dashboard")
    parser.add_argument("--engine", required=True, help="Path to autocore_engine executable")
    parser.add_argument("--host", default="127.0.0.1", help="Loopback address (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--open", action="store_true", help="Open dashboard in browser")
    args = parser.parse_args()
    if args.host != "127.0.0.1":
        parser.error("For safety and predictable browser routing, AutoCore dashboard must bind to 127.0.0.1")
    if not 1 <= args.port <= 65535:
        parser.error("Port must be in the range 1..65535")
    executable = Path(args.engine).resolve()
    bridge = EngineBridge(executable)
    server: ThreadingHTTPServer | None = None
    url = f"http://127.0.0.1:{args.port}/"
    try:
        server = ThreadingHTTPServer((args.host, args.port), make_handler(bridge))
        print("=" * 66)
        print("AutoCore Live Automotive Network Dashboard")
        print(f"Dashboard: {url}")
        print("Simulation engine: C++17 | HTTP API: localhost only")
        print("Press Ctrl+C in this window to stop the application.")
        print("=" * 66, flush=True)
        if args.open:
            threading.Timer(0.8, lambda: webbrowser.open(url)).start()
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nStopping AutoCore dashboard...", flush=True)
    finally:
        if server is not None:
            server.server_close()
        bridge.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
