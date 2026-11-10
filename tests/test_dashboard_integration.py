#!/usr/bin/env python3
"""End-to-end smoke test for the local dashboard bridge and C++ engine."""
from __future__ import annotations

import json
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def reserve_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def get_json(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=3) as response:
        return json.loads(response.read().decode("utf-8"))


def post_json(url: str, payload: dict) -> dict:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=3) as response:
        return json.loads(response.read().decode("utf-8"))


def matching_state(url: str, predicate):
    state = get_json(url)
    return state if predicate(state) else None


def wait_for(predicate, timeout: float = 3.0, interval: float = 0.05):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            value = predicate()
        except (OSError, urllib.error.URLError, ConnectionError):
            value = None
        if value:
            return value
        time.sleep(interval)
    raise AssertionError("Timed out waiting for expected dashboard state")


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_dashboard_integration.py PATH_TO_AUTOCORE_ENGINE")
    engine = Path(sys.argv[1]).resolve()
    server_script = ROOT / "dashboard" / "server.py"
    missing_matrix = ROOT / "config" / "does_not_exist_autocore_matrix.csv"
    startup_failure = subprocess.run(
        [str(engine), "--can-matrix", str(missing_matrix)],
        input="", text=True, capture_output=True, timeout=3,
    )
    assert startup_failure.returncode == 1
    assert "Cannot open CAN message matrix" in startup_failure.stderr
    port = reserve_port()
    base = f"http://127.0.0.1:{port}"
    # Keep startup logs so a Windows-only process/engine failure is reported
    # immediately instead of becoming an opaque connection timeout. A first
    # Windows launch can be slower while Defender scans the freshly built EXE.
    server_log = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")
    proc = subprocess.Popen(
        [sys.executable, str(server_script), "--engine", str(engine), "--port", str(port)],
        cwd=str(ROOT), stdout=server_log, stderr=subprocess.STDOUT,
    )
    checks = 1  # startup rejects a missing CAN message matrix with a useful error

    def dashboard_startup_state():
        if proc.poll() is not None:
            server_log.flush()
            server_log.seek(0)
            detail = server_log.read().strip()
            raise AssertionError(
                f"Dashboard server exited during startup (exit {proc.returncode}).\n{detail}"
            )
        return get_json(base + "/api/state")

    try:
        # Give Windows first-run process creation, endpoint protection and
        # C++ engine startup enough time. The default 3 s was too aggressive.
        initial = wait_for(dashboard_startup_state, timeout=20.0, interval=0.1)
        assert initial["engine"]["online"] and initial["battery"]["online"]
        assert initial["can"]["frames_total"] >= 3 and len(initial["history"]) >= 1
        matrix = initial["can"]["message_database"]
        assert matrix["message_count"] == 5 and matrix["signal_count"] == 16
        assert {item["name"] for item in matrix["messages"]} >= {
            "ENGINE_STATUS", "BATTERY_STATUS", "BODY_STATUS", "UDS_REQUEST", "UDS_RESPONSE"
        }
        assert all("name" in frame for frame in initial["can"]["last_frames"])
        health = initial["can"]["controller_health"]
        assert health["error_active_count"] == 4 and health["error_passive_count"] == 0 and health["bus_off_count"] == 0
        assert {item["key"] for item in health["controllers"]} == {"ENGINE", "BATTERY", "BODY", "DIAGNOSTICS"}
        assert all(item["state"] == "ERROR_ACTIVE" and item["tec"] == 0 and item["rec"] == 0 for item in health["controllers"])
        assert initial["can"]["monitor"]["filter_active"] is False
        assert initial["injected_faults"] == {
            "overtemperature": False, "low_voltage": False,
            "engine_offline": False, "battery_offline": False,
            "crc": False, "counter": False,
        }
        checks += 1

        time.sleep(0.45)
        live = get_json(base + "/api/state")
        assert live["time_ms"] > initial["time_ms"]
        assert live["can"]["frames_total"] > initial["can"]["frames_total"]
        checks += 1

        filtered = post_json(base + "/api/action", {
            "action": "set_can_filter", "can_id": "100", "mask": "7FF"
        })
        assert filtered["can"]["monitor"]["filter_active"] is True
        assert filtered["can"]["monitor"]["filter_id"] == "0x100"
        filtered = wait_for(lambda: matching_state(base + "/api/state", lambda state:
            state["can"]["monitor"]["accepted"] >= 1 and
            state["can"]["monitor"]["rejected"] >= 2 and
            state["can"]["last_frames"] and
            all(frame["id"] == "0x100" for frame in state["can"]["last_frames"])))
        assert filtered["can"]["frames_total"] > 0
        cleared_filter = post_json(base + "/api/action", {"action": "clear_can_filter"})
        assert cleared_filter["can"]["monitor"]["filter_active"] is False
        all_frames = wait_for(lambda: matching_state(base + "/api/state", lambda state:
            {frame["id"] for frame in state["can"]["last_frames"]} >= {"0x100", "0x110", "0x120"}))
        assert all_frames["can"]["monitor"]["accepted"] >= 3
        invalid_filter = urllib.request.Request(base + "/api/action",
            data=json.dumps({"action": "set_can_filter", "can_id": "800", "mask": "7FF"}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(invalid_filter, timeout=3)
            raise AssertionError("out-of-range CAN acceptance filter unexpectedly succeeded")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        checks += 1

        paused = post_json(base + "/api/action", {"action": "pause"})
        assert paused["running"] is False
        frozen_time = paused["time_ms"]
        time.sleep(0.3)
        assert get_json(base + "/api/state")["time_ms"] == frozen_time
        # Fault injection must automatically resume a paused simulation so a user
        # cannot silently inject a fault that will never reach the ECU monitor.
        resumed_by_fault = post_json(base + "/api/action", {"action": "fault_overtemp"})
        assert resumed_by_fault["running"] is True
        assert resumed_by_fault["injected_faults"]["overtemperature"] is True
        overtemp = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            s["engine"]["temperature_c"] >= 120 and
            any(d["code"] == "P0217" and d["active"] for d in s["diagnostics"]["dtcs"])))
        assert overtemp["diagnostics"]["active_faults"] >= 1
        p0217_pending = next(d for d in overtemp["diagnostics"]["dtcs"] if d["code"] == "P0217")
        assert p0217_pending["pending"] or p0217_pending["confirmed"]
        confirmed_state = wait_for(lambda: matching_state(base + "/api/state", lambda state:
            any(d["code"] == "P0217" and d["active"] and d["confirmed"]
                for d in state["diagnostics"]["dtcs"])))
        p0217 = next(d for d in confirmed_state["diagnostics"]["dtcs"] if d["code"] == "P0217")
        assert p0217["freeze_frame"] is not None and p0217["freeze_frame"]["temperature_c"] >= 120
        p0217_events = [event["message"].lower() for event in confirmed_state["events"]]
        assert any("p0217 pending" in message for message in p0217_events)
        assert any("p0217 confirmed" in message for message in p0217_events)
        assert overtemp["injected_faults"]["overtemperature"] is True
        freeze = post_json(base + "/api/action", {"action": "read_freeze_frame", "code": "P0217"})
        assert freeze["uds"]["response"].startswith("59 04 00 02 17")
        checks += 1

        # Dashboard body controls and throttle boundaries must affect the same C++ state.
        assert initial["body"]["headlights"] is True and initial["body"]["door_open"] is False
        body_off = post_json(base + "/api/action", {"action": "toggle_lights"})
        assert body_off["body"]["headlights"] is False
        door_open = post_json(base + "/api/action", {"action": "toggle_door"})
        assert door_open["body"]["door_open"] is True
        body_restore = post_json(base + "/api/action", {"action": "toggle_lights"})
        door_restore = post_json(base + "/api/action", {"action": "toggle_door"})
        assert body_restore["body"]["headlights"] is True and door_restore["body"]["door_open"] is False
        assert any(frame["id"] == "0x120" for frame in get_json(base + "/api/state")["can"]["last_frames"])
        reset_state = post_json(base + "/api/action", {"action": "reset"})
        assert reset_state["time_ms"] == 0 and reset_state["throttle"] == 35
        assert reset_state["body"]["headlights"] is True and reset_state["body"]["door_open"] is False
        assert reset_state["diagnostics"]["active_faults"] == 0
        assert not any(reset_state["injected_faults"].values())
        assert not any(dtc["code"] == "P0217" and dtc["active"] for dtc in reset_state["diagnostics"]["dtcs"])
        checks += 1
        throttle = post_json(base + "/api/action", {"action": "set_throttle", "value": 0})
        assert throttle["throttle"] == 0
        throttle = post_json(base + "/api/action", {"action": "set_throttle", "value": 100})
        assert throttle["throttle"] == 100
        throttle = post_json(base + "/api/action", {"action": "set_throttle", "value": 70})
        assert throttle["throttle"] == 70
        did = post_json(base + "/api/action", {"action": "read_did", "did": "F40C"})
        assert did["uds"]["response"].startswith("62 F4 0C")
        session = post_json(base + "/api/action", {"action": "uds_session"})
        assert session["uds"]["response"].startswith("50 03")
        reset_ack = post_json(base + "/api/action", {"action": "uds_reset"})
        assert reset_ack["uds"]["response"].startswith("51 01")
        tester = post_json(base + "/api/action", {"action": "tester_present"})
        assert tester["uds"]["response"].startswith("7E 00")
        checks += 1

        post_json(base + "/api/action", {"action": "fault_low_voltage"})
        low_v = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            s["battery"]["voltage_v"] < 11.0 and
            any(d["code"] == "P0562" and d["active"] and d["confirmed"] for d in s["diagnostics"]["dtcs"])))
        assert low_v["diagnostics"]["active_faults"] >= 1
        assert low_v["injected_faults"]["low_voltage"] is True
        p0562 = next(d for d in low_v["diagnostics"]["dtcs"] if d["code"] == "P0562")
        assert p0562["freeze_frame"] is not None and p0562["freeze_frame"]["battery_v"] < 11.0
        recovered_sensor = post_json(base + "/api/action", {"action": "recover_all"})
        assert recovered_sensor["running"] is True
        assert all(not value for value in recovered_sensor["injected_faults"].values())
        recovered_sensor_state = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            s["battery"]["voltage_v"] >= 11.0 and
            not any(d["code"] == "P0562" and d["active"] and d["confirmed"] for d in s["diagnostics"]["dtcs"])))
        assert any("p0562 recovered" in event["message"].lower()
                   for event in recovered_sensor_state["events"])

        post_json(base + "/api/action", {"action": "fault_crc"})
        crc_state = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            any(d["code"] == "AC0001" and d["active"] and d["confirmed"] for d in s["diagnostics"]["dtcs"])))
        assert crc_state["injected_faults"]["crc"] is True
        assert any(not f["valid_crc"] for f in crc_state["can"]["last_frames"])
        time.sleep(0.8)
        crc_held = get_json(base + "/api/state")
        assert any(d["code"] == "AC0001" and d["active"] for d in crc_held["diagnostics"]["dtcs"]), \
            "CRC DTC must remain visible across multiple dashboard polling intervals"
        post_json(base + "/api/action", {"action": "recover_all"})
        crc_recovered = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            not any(d["code"] == "AC0001" and d["active"] and d["confirmed"] for d in s["diagnostics"]["dtcs"])))
        assert not any(d["code"] == "AC0002" and d["active"] for d in crc_recovered["diagnostics"]["dtcs"]), \
            "CRC recovery must not create a false rolling-counter DTC"

        post_json(base + "/api/action", {"action": "fault_counter"})
        counter_state = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            any(d["code"] == "AC0002" and d["active"] and d["confirmed"] for d in s["diagnostics"]["dtcs"])))
        assert counter_state["injected_faults"]["counter"] is True
        time.sleep(0.8)
        counter_held = get_json(base + "/api/state")
        assert any(d["code"] == "AC0002" and d["active"] for d in counter_held["diagnostics"]["dtcs"]), \
            "Rolling-counter DTC must remain visible across multiple dashboard polling intervals"
        post_json(base + "/api/action", {"action": "recover_all"})
        wait_for(lambda: matching_state(base + "/api/state", lambda s:
            not any(d["code"] == "AC0002" and d["active"] and d["confirmed"] for d in s["diagnostics"]["dtcs"])))
        checks += 1

        # The prominent paired demo control creates two simultaneously visible sensor DTCs.
        post_json(base + "/api/action", {"action": "fault_sensor_demo"})
        demo = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            {d["code"] for d in s["diagnostics"]["dtcs"] if d["active"]} >= {"P0217", "P0562"}))
        assert demo["engine"]["temperature_c"] >= 120 and demo["battery"]["voltage_v"] < 11.0
        assert demo["injected_faults"]["overtemperature"] and demo["injected_faults"]["low_voltage"]
        dtc_response = post_json(base + "/api/action", {"action": "read_dtcs"})
        assert dtc_response["uds"]["response"].startswith("59 02 2F")
        post_json(base + "/api/action", {"action": "recover_all"})
        wait_for(lambda: matching_state(base + "/api/state", lambda s:
            s["diagnostics"]["active_faults"] == 0))
        checks += 1

        post_json(base + "/api/action", {"action": "fault_engine_offline"})
        offline = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            any(d["code"] == "U0100" and d["active"] and d["confirmed"] for d in s["diagnostics"]["dtcs"])),
            timeout=2.5)
        assert offline["engine"]["online"] is False
        assert offline["injected_faults"]["engine_offline"] is True
        stale_rpm = offline["engine"]["rpm"]
        time.sleep(0.25)
        assert get_json(base + "/api/state")["engine"]["rpm"] == stale_rpm
        checks += 1

        post_json(base + "/api/action", {"action": "fault_battery_offline"})
        battery_offline = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            any(d["code"] == "U0111" and d["active"] and d["confirmed"] for d in s["diagnostics"]["dtcs"])),
            timeout=2.5)
        assert battery_offline["battery"]["online"] is False
        assert battery_offline["injected_faults"]["battery_offline"] is True
        checks += 1

        recovered = post_json(base + "/api/action", {"action": "recover_all"})
        assert recovered["engine"]["online"] and recovered["battery"]["online"]
        stable = wait_for(lambda: matching_state(base + "/api/state", lambda s:
            s["diagnostics"]["active_faults"] == 0), timeout=2.0)
        assert stable["engine"]["temperature_c"] < 115
        checks += 1

        dtc = post_json(base + "/api/action", {"action": "read_dtcs"})
        assert dtc["uds"]["response"].startswith("59 02")
        cleared = post_json(base + "/api/action", {"action": "clear_dtcs"})
        assert cleared["uds"]["response"] == "54"
        checks += 1

        csv_data = urllib.request.urlopen(base + "/api/export.csv", timeout=3).read().decode("utf-8")
        assert csv_data.startswith("time_ms,rpm,temperature_c,speed_kph,voltage_v,current_a")
        html = urllib.request.urlopen(base + "/", timeout=3).read().decode("utf-8")
        assert "AutoCore Control Center" in html or "AUTOCORE" in html
        assert "setInterval(poll,500)" in html and "fault_crc" in html
        assert 'id="faultMonitor"' in html and 'id="runFullDemo"' in html
        assert 'id="globalDemoBtn"' in html and 'id="overviewRunDemo"' in html
        assert 'id="dtcCatalogueRows"' in html and 'id="diagInjectDemo"' in html
        assert "runFullDiagnosticDemo" in html and "renderMonitorCatalogue" in html
        health = get_json(base + "/api/health")
        assert health["ok"] is True and health["engine"] == "running"
        checks += 1

        invalid = urllib.request.Request(base + "/api/action", data=b'{"action":"not_allowed"}',
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(invalid, timeout=3)
            raise AssertionError("Unsupported dashboard action was not rejected")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        invalid_throttle = urllib.request.Request(base + "/api/action", data=b'{"action":"set_throttle","value":101}',
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(invalid_throttle, timeout=3)
            raise AssertionError("Out-of-range throttle was not rejected")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        invalid_bool = urllib.request.Request(base + "/api/action", data=b'{"action":"set_throttle","value":true}',
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(invalid_bool, timeout=3)
            raise AssertionError("Boolean throttle was not rejected as an integer")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        invalid_did = urllib.request.Request(base + "/api/action", data=b'{"action":"read_did","did":"FFFF"}',
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(invalid_did, timeout=3)
            raise AssertionError("Unsupported DID was not rejected")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        invalid_freeze = urllib.request.Request(base + "/api/action", data=b'{"action":"read_freeze_frame","code":"P9999"}',
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(invalid_freeze, timeout=3)
            raise AssertionError("Unsupported freeze-frame DTC was not rejected")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        bad_content_type = urllib.request.Request(base + "/api/action", data=b'{"action":"pause"}',
            headers={"Content-Type": "text/plain"}, method="POST")
        try:
            urllib.request.urlopen(bad_content_type, timeout=3)
            raise AssertionError("Non-JSON action content type was not rejected")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        checks += 1

        # Reproduce the exact guided Scenario Lab workflow against the real
        # C++ engine. This is deliberately separate from the browser mock test.
        post_json(base + "/api/action", {"action": "reset"})
        post_json(base + "/api/action", {"action": "start"})
        guided_cases = [
            ("fault_overtemp", "P0217", lambda state: state["engine"]["temperature_c"] >= 120, True),
            ("fault_low_voltage", "P0562", lambda state: state["battery"]["voltage_v"] < 11.0, True),
            ("fault_engine_offline", "U0100", lambda state: not state["engine"]["online"], False),
            ("fault_battery_offline", "U0111", lambda state: not state["battery"]["online"], False),
            ("fault_crc", "AC0001", lambda state: state["injected_faults"]["crc"], False),
            ("fault_counter", "AC0002", lambda state: True, False),
        ]
        for action_name, code, signal_check, needs_freeze in guided_cases:
            injected = post_json(base + "/api/action", {"action": action_name})
            assert injected["injected_faults"]
            detected = wait_for(lambda: matching_state(base + "/api/state", lambda state:
                signal_check(state) and any(dtc["code"] == code and dtc["active"]
                                             for dtc in state["diagnostics"]["dtcs"])),
                timeout=4.0)
            read = post_json(base + "/api/action", {"action": "read_dtcs"})
            assert read["uds"]["response"].startswith("59 02 2F"), (code, read["uds"])
            if needs_freeze:
                snapshot = post_json(base + "/api/action", {"action": "read_freeze_frame", "code": code})
                snapshot_bytes = [int(byte, 16) for byte in snapshot["uds"]["response"].split()]
                expected_dtc = int(code[1:], 16)
                expected_dtc_bytes = [(expected_dtc >> 16) & 0xFF,
                                      (expected_dtc >> 8) & 0xFF, expected_dtc & 0xFF]
                expected_dids = [[0xF4, 0x0C], [0xF4, 0x05], [0xF4, 0x0D],
                                 [0xF4, 0x11], [0xF4, 0xA0]]
                actual_dids = [snapshot_bytes[index:index + 2] for index in (8, 12, 16, 20, 24)]
                assert snapshot_bytes[:5] == [0x59, 0x04, *expected_dtc_bytes], (code, snapshot["uds"])
                assert len(snapshot_bytes) == 30 and snapshot_bytes[7] == 5 and \
                    actual_dids == expected_dids, (code, snapshot["uds"])
                record = next(dtc for dtc in detected["diagnostics"]["dtcs"] if dtc["code"] == code)
                assert record["freeze_frame"] is not None
            post_json(base + "/api/action", {"action": "recover_all"})
            wait_for(lambda: matching_state(base + "/api/state", lambda state:
                not any(dtc["code"] == code and dtc["active"] for dtc in state["diagnostics"]["dtcs"])
                and state["engine"]["online"] and state["battery"]["online"]
                and state["engine"]["temperature_c"] < 115
                and state["battery"]["voltage_v"] >= 11.0), timeout=5.0)

        for did in ("F40C", "F405", "F411"):
            response = post_json(base + "/api/action", {"action": "read_did", "did": did})
            prefix = "62 " + did[:2] + " " + did[2:]
            assert response["uds"]["response"].startswith(prefix), (did, response["uds"])
        assert post_json(base + "/api/action", {"action": "uds_session"})["uds"]["response"].startswith("50 03")
        assert post_json(base + "/api/action", {"action": "tester_present"})["uds"]["response"].startswith("7E 00")
        assert post_json(base + "/api/action", {"action": "uds_reset"})["uds"]["response"].startswith("51 01")
        history = get_json(base + "/api/state")
        assert history["diagnostics"]["active_faults"] == 0
        assert history["diagnostics"]["dtc_count"] >= 6
        cleared = post_json(base + "/api/action", {"action": "clear_dtcs"})
        assert cleared["uds"]["response"] == "54"
        checks += 1

        # CAN controller error-confinement workflow: TEC/REC thresholds,
        # bus-off isolation, and the 128-step educational recovery model.
        controller_state = post_json(base + "/api/action", {"action": "reset"})
        selected = next(item for item in controller_state["can"]["controller_health"]["controllers"]
                        if item["key"] == "ENGINE")
        assert selected["state"] == "ERROR_ACTIVE"
        # Recovery requests are rejected at the HTTP boundary until the node is
        # Bus-Off, not merely logged as an unsuccessful action by the engine.
        not_bus_off = urllib.request.Request(base + "/api/action",
            data=json.dumps({"action": "can_bus_off_recovery", "node": "ENGINE"}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(not_bus_off, timeout=3)
            raise AssertionError("recovery unexpectedly succeeded for an Error Active controller")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        passive = post_json(base + "/api/action", {
            "action": "can_tx_error_burst", "node": "ENGINE", "count": 24
        })
        engine_ctrl = next(item for item in passive["can"]["controller_health"]["controllers"]
                           if item["key"] == "ENGINE")
        assert engine_ctrl["tec"] == 192 and engine_ctrl["state"] == "ERROR_PASSIVE", engine_ctrl
        receive_passive = post_json(base + "/api/action", {
            "action": "can_rx_error_burst", "node": "BATTERY", "count": 128
        })
        battery_ctrl = next(item for item in receive_passive["can"]["controller_health"]["controllers"]
                            if item["key"] == "BATTERY")
        assert battery_ctrl["rec"] == 128 and battery_ctrl["state"] == "ERROR_PASSIVE", battery_ctrl
        bus_off = post_json(base + "/api/action", {
            "action": "can_tx_error_burst", "node": "ENGINE", "count": 32
        })
        engine_ctrl = next(item for item in bus_off["can"]["controller_health"]["controllers"]
                           if item["key"] == "ENGINE")
        assert engine_ctrl["tec"] == 256 and engine_ctrl["state"] == "BUS_OFF"
        assert bus_off["engine"]["online"] is False
        frames_before = bus_off["can"]["frames_total"]
        other_nodes = wait_for(lambda: matching_state(base + "/api/state", lambda state:
            state["can"]["frames_total"] > frames_before and
            state["engine"]["online"] is False and state["battery"]["online"] and state["body"]["online"]))
        post_busoff_frames = [frame for frame in other_nodes["can"]["last_frames"]
                              if frame["time_ms"] > bus_off["time_ms"]]
        assert post_busoff_frames and all(frame["id"] != "0x100" for frame in post_busoff_frames)
        assert {frame["id"] for frame in post_busoff_frames} >= {"0x110", "0x120"}
        busoff_dtc = wait_for(lambda: matching_state(base + "/api/state", lambda state:
            any(dtc["code"] == "U0100" and dtc["active"] and dtc["confirmed"]
                for dtc in state["diagnostics"]["dtcs"])), timeout=2.5)
        assert busoff_dtc["diagnostics"]["active_faults"] >= 1
        # A different ECU's persistent TX injector also blocks network recovery.
        post_json(base + "/api/action", {
            "action": "can_tx_error_injection", "node": "BATTERY", "enabled": True
        })
        blocked_recovery = urllib.request.Request(base + "/api/action",
            data=json.dumps({"action": "can_bus_off_recovery", "node": "ENGINE"}).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            urllib.request.urlopen(blocked_recovery, timeout=3)
            raise AssertionError("recovery unexpectedly succeeded while a network TX injector was enabled")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
        post_json(base + "/api/action", {
            "action": "can_tx_error_injection", "node": "BATTERY", "enabled": False
        })
        recovery_start = post_json(base + "/api/action", {
            "action": "can_bus_off_recovery", "node": "ENGINE"
        })
        engine_ctrl = next(item for item in recovery_start["can"]["controller_health"]["controllers"]
                           if item["key"] == "ENGINE")
        assert engine_ctrl["recovery_in_progress"] is True
        recovered = wait_for(lambda: matching_state(base + "/api/state", lambda state:
            next(item for item in state["can"]["controller_health"]["controllers"]
                 if item["key"] == "ENGINE")["state"] == "ERROR_ACTIVE"), timeout=16.5, interval=0.1)
        engine_ctrl = next(item for item in recovered["can"]["controller_health"]["controllers"]
                           if item["key"] == "ENGINE")
        assert engine_ctrl["recovery_sequences_seen"] == 128
        assert engine_ctrl["bus_off_recoveries"] == 1
        assert recovered["engine"]["online"] is True
        recovered_dtc = wait_for(lambda: matching_state(base + "/api/state", lambda state:
            not any(dtc["code"] == "U0100" and dtc["active"]
                    for dtc in state["diagnostics"]["dtcs"])), timeout=2.5)
        assert recovered_dtc["engine"]["online"] is True
        # A persistent TX injector accumulates eight TEC counts per attempted
        # engine transmission; disabling it allows normal transmissions to recover TEC.
        enabled = post_json(base + "/api/action", {
            "action": "can_tx_error_injection", "node": "ENGINE", "enabled": True
        })
        assert next(item for item in enabled["can"]["controller_health"]["controllers"]
                    if item["key"] == "ENGINE")["tx_error_injection"] is True
        disabled = post_json(base + "/api/action", {
            "action": "can_tx_error_injection", "node": "ENGINE", "enabled": False
        })
        assert next(item for item in disabled["can"]["controller_health"]["controllers"]
                    if item["key"] == "ENGINE")["tx_error_injection"] is False
        # Reject invalid node names, bool-as-int counts, and an over-range burst.
        for payload in (
            {"action": "can_tx_error_burst", "node": "FAKE", "count": 1},
            {"action": "can_tx_error_burst", "node": "ENGINE", "count": True},
            {"action": "can_rx_error_burst", "node": "ENGINE", "count": 256},
            {"action": "can_tx_error_injection", "node": "ENGINE", "enabled": "yes"},
        ):
            bad = urllib.request.Request(base + "/api/action", data=json.dumps(payload).encode(),
                headers={"Content-Type": "application/json"}, method="POST")
            try:
                urllib.request.urlopen(bad, timeout=3)
                raise AssertionError("Invalid CAN controller action unexpectedly succeeded")
            except urllib.error.HTTPError as exc:
                assert exc.code == 400
        checks += 1

        print(f"PASS: dashboard-to-C++ integration checks ({checks} groups, including full diagnostics and CAN controller fault-confinement workflow)")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)
        server_log.close()


if __name__ == "__main__":
    raise SystemExit(main())
