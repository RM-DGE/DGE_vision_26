#!/usr/bin/env python3
"""Live Foxglove protocol smoke: start graphical simulator and auto_aim_sim first.

Uses only Python's standard library; does not send robot control commands.
"""
import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import time


def exact(stream, size):
    value = bytearray()
    while len(value) < size:
        part = stream.recv(size - len(value))
        if not part:
            raise RuntimeError("WebSocket disconnected")
        value.extend(part)
    return bytes(value)


def receive(stream):
    header = exact(stream, 2)
    opcode, size = header[0] & 15, header[1] & 127
    if size == 126:
        size = struct.unpack("!H", exact(stream, 2))[0]
    elif size == 127:
        size = struct.unpack("!Q", exact(stream, 8))[0]
    if header[1] & 128:
        raise RuntimeError("Unexpected masked server frame")
    return opcode, exact(stream, size)


def send(stream, payload, opcode=1):
    mask = os.urandom(4)
    size = len(payload)
    header = bytes([128 | opcode])
    if size < 126:
        header += bytes([128 | size])
    elif size <= 65535:
        header += bytes([128 | 126]) + struct.pack("!H", size)
    else:
        header += bytes([128 | 127]) + struct.pack("!Q", size)
    stream.sendall(header + mask + bytes(v ^ mask[i % 4] for i, v in enumerate(payload)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--duration", type=float, default=12)
    parser.add_argument("--require-model", action="store_true", help="Require a tracked four-plate model and a valid Aimer selection")
    parser.add_argument("--status-only", action="store_true", help="Check disconnected/paused status without requiring image traffic")
    parser.add_argument("--expect-inactive", action="store_true", help="Require stale state and invalid observation (use after stopping simulator)")
    options = parser.parse_args()
    expected = {"/camera/image", "/camera/annotations", "/dge/scene", "/dge/prediction",
                "/tf", "/tracker/target", "/tracker/info", "/dge/debug", "/sim/status",
                "/dge/ekf_model", "/dge/selection"}
    if options.status_only:
        expected = {"/sim/status"}
    with socket.create_connection((options.host, options.port), timeout=5) as stream:
        key = base64.b64encode(os.urandom(16)).decode()
        stream.sendall((f"GET / HTTP/1.1\r\nHost: {options.host}:{options.port}\r\n"
                        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                        "Origin: http://localhost\r\n"
                        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
                        "Sec-WebSocket-Protocol: foxglove.sdk.v1, foxglove.websocket.v1\r\n\r\n").encode())
        response = bytearray()
        while not response.endswith(b"\r\n\r\n"):
            response.extend(exact(stream, 1))
        accept = base64.b64encode(hashlib.sha1(
            (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
        if response.split(b"\r\n", 1)[0].split()[1:2] != [b"101"] or accept.lower() not in response.lower():
            raise RuntimeError("WebSocket upgrade failed: " + response.decode(errors="replace"))
        subscriptions, counts, samples = {}, {}, {}
        selected_frames = 0
        deadline = time.monotonic() + options.duration
        while time.monotonic() < deadline:
            stream.settimeout(max(0.01, deadline - time.monotonic()))
            try:
                opcode, payload = receive(stream)
            except socket.timeout:
                break
            if opcode == 9:
                send(stream, payload, 10)
            elif opcode == 1:
                message = json.loads(payload)
                if message.get("op") == "advertise":
                    additions = []
                    for channel in message["channels"]:
                        if channel["topic"] in expected:
                            identity = channel["id"]
                            subscriptions[identity] = channel["topic"]
                            additions.append({"id": identity, "channelId": identity})
                    if additions:
                        send(stream, json.dumps({"op": "subscribe", "subscriptions": additions}).encode())
            elif opcode == 2 and len(payload) >= 13 and payload[0] == 1:
                identity = struct.unpack_from("<I", payload, 1)[0]
                topic = subscriptions.get(identity)
                if topic:
                    counts[topic] = counts.get(topic, 0) + 1
                    if topic in {"/sim/status", "/dge/debug", "/tracker/target", "/dge/selection"}:
                        samples[topic] = json.loads(payload[13:])
                    if topic == "/dge/selection":
                        model = samples[topic]
                        if model["tracking"] and model["aim_valid"]:
                            if len(model["plates"]) != 4 or sum(p["selected"] for p in model["plates"]) != 1:
                                raise RuntimeError("Expected four plates and exactly one selected plate")
                            if model["plates"][model["selected_id"]]["selected"] is not True:
                                raise RuntimeError("Selected plate ID disagrees with per-plate flags")
                            if abs(model["height_m"] - 0.056) > 1e-9:
                                raise RuntimeError("Rendered model height differs from EKF lightbar geometry")
                            selected_frames += 1
            elif opcode == 8:
                raise RuntimeError("Server closed before smoke completed")
        missing = expected - counts.keys()
        if missing:
            raise RuntimeError("No data received on: " + ", ".join(sorted(missing)))
        if options.require_model and selected_frames == 0:
            raise RuntimeError("No valid tracked model/selection received")
        status = samples["/sim/status"]
        if "fresh" not in status or "observation_valid" not in status:
            raise RuntimeError("Simulation status fields missing")
        if options.expect_inactive and (status["fresh"] is not False or status["observation_valid"] is not False):
            raise RuntimeError("Expected fresh=false and observation_valid=false after simulator stops")
        if not options.status_only and ("stats" not in status or "session_id" not in status):
            raise RuntimeError("Simulation state missing")
        if not options.status_only and ("fps" not in samples["/dge/debug"] or "tracking" not in samples["/tracker/target"]):
            raise RuntimeError("Algorithm diagnostics missing")
        print(json.dumps({"result": "PASS", "received": counts, "selected_frames": selected_frames,
                          "selection": samples.get("/dge/selection"), "status": status}, ensure_ascii=False))


if __name__ == "__main__":
    main()
