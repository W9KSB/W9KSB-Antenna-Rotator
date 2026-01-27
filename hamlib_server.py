# hamlib_server.py
import socket
import threading
import time

import config
from controller import RotatorController

HOST = "0.0.0.0"
PORT = 4533  # rotctld default

PRINT_RAW_BYTES = False  # set True if you need to debug traffic


def format_pos_two_lines(az, el) -> bytes:
    return f"{az:.6f}\n{el:.6f}\n".encode("ascii")


def reply_rprt(conn, code: int):
    conn.send(f"RPRT {code}\n".encode("ascii"))


def dump_state_text() -> bytes:
    lines = [
        "Model: PythonRotator AZ/EL",
        "MinAz: 0.0",
        "MaxAz: 360.0",
        f"MinEl: {config.EL_MIN_DEG:.1f}",
        f"MaxEl: {config.EL_MAX_DEG:.1f}",
    ]
    return ("\n".join(lines) + "\n").encode("ascii")


def normalize_cmd(s: str) -> str:
    s = s.strip()
    while s and s[0] in "+;|,":
        s = s[1:].lstrip()
    return s


def looks_like_complete_command_bytes(buf: bytes) -> bool:
    b = buf.strip()
    if not b:
        return False

    if b in (b"p", b"S", b"s", b"_", b"q"):
        return True

    try:
        s = b.decode("ascii", errors="ignore").strip()
    except Exception:
        return False

    s = normalize_cmd(s)
    parts = s.split()
    if not parts:
        return False

    if parts[0] in ("P", r"\set_pos", "set_pos"):
        return len(parts) >= 3

    if parts[0] in (
        "S", "s", r"\stop", "stop",
        "_", r"\get_info", "get_info",
        r"\dump_state", "dump_state",
        "p", r"\get_pos", "get_pos",
        "q",
    ):
        return True

    return False


def handle_command(cmd: str, conn, rc: RotatorController):
    # Return False to close connection
    if cmd == "q":
        return False

    if cmd in ("_", r"\get_info", "get_info"):
        conn.send(b"Info: PythonRotator AZ/EL\n")
        reply_rprt(conn, 0)
        return True

    if cmd in (r"\dump_state", "dump_state"):
        conn.send(dump_state_text())
        reply_rprt(conn, 0)
        return True

    if cmd in ("S", "s", r"\stop", "stop"):
        rc.stop()
        reply_rprt(conn, 0)
        return True

    if cmd in ("p", r"\get_pos", "get_pos"):
        az, el = rc.get_position()
        conn.send(format_pos_two_lines(az, el))
        return True

    if cmd.startswith("P ") or cmd.startswith(r"\set_pos ") or cmd.startswith("set_pos "):
        parts = cmd.split()
        if len(parts) >= 3:
            try:
                az_t = float(parts[1])
                el_t = float(parts[2])
                rc.set_target(az_t, el_t)
                reply_rprt(conn, 0)
            except Exception:
                reply_rprt(conn, 1)
        else:
            reply_rprt(conn, 1)
        return True

    reply_rprt(conn, 1)
    return True


def process_one_line(raw_line: bytes, addr, conn, rc):
    raw_line = raw_line.strip()
    if not raw_line:
        return True

    try:
        line = raw_line.decode("ascii", errors="ignore")
    except Exception:
        reply_rprt(conn, 1)
        return True

    cmd = normalize_cmd(line)
    # Uncomment if you want to see parsed commands:
    # print(f"[CMD] {addr}: {cmd}", flush=True)
    return handle_command(cmd, conn, rc)


def send_home_both(rc: RotatorController, reason: str, addr=None):
    # Home definition:
    #   AZ=0.00 -> your session "home" (with auto-zero AZ on controller startup)
    #   EL=0.00 -> your elevation home per el_offset_deg
    tag = f"[HOME] ({reason})"
    if addr:
        tag += f" client={addr}"
    print(f"{tag} -> AZ=0.00 EL=0.00", flush=True)
    rc.set_target(0.0, 0.0)


def handle_client(conn, addr, rc: RotatorController):
    print(f"Client connected: {addr}", flush=True)

    # If we see an explicit quit command, we’ll set this and home on the way out.
    quit_requested = False

    try:
        conn.settimeout(0.2)
        buf = b""
        last_rx = time.time()

        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    # client closed socket
                    break

                last_rx = time.time()
                if PRINT_RAW_BYTES:
                    print(f"[RAW] {addr}: {data!r}", flush=True)

                buf += data

            except socket.timeout:
                pass

            # Newline terminated
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                keep = process_one_line(line, addr, conn, rc)
                if not keep:
                    quit_requested = True
                    return

            # CR terminated
            while b"\r" in buf:
                line, buf = buf.split(b"\r", 1)
                keep = process_one_line(line, addr, conn, rc)
                if not keep:
                    quit_requested = True
                    return

            # Single-byte immediate commands
            if buf.strip() in (b"p", b"_", b"q", b"S", b"s"):
                keep = process_one_line(buf, addr, conn, rc)
                buf = b""
                if not keep:
                    quit_requested = True
                    return

            # Idle flush for no-newline clients (like your gpredict behavior)
            if buf.strip():
                idle = time.time() - last_rx
                if idle >= 0.10 and looks_like_complete_command_bytes(buf):
                    keep = process_one_line(buf, addr, conn, rc)
                    buf = b""
                    if not keep:
                        quit_requested = True
                        return

    finally:
        try:
            conn.close()
        except Exception:
            pass

        # ALWAYS home on disconnect, per your request
        send_home_both(rc, reason=("quit" if quit_requested else "disconnect"), addr=addr)

        print(f"Client disconnected: {addr}", flush=True)


def main():
    rc = RotatorController(debug=False)
    rc.start()

    # Optional: keep your boot behavior (home EL only, hold AZ where it is)
    az_now, _ = rc.get_position()
    print(f"[BOOT] Homing EL to 0.00° (keeping AZ at {az_now:.2f}°)", flush=True)
    rc.set_target(az_now, 0.0)

    print(f"Hamlib rotctld-compatible server listening on {HOST}:{PORT}", flush=True)
    print("Supports: p/\\get_pos, P/\\set_pos, S/\\stop, _/\\get_info, \\dump_state, q", flush=True)
    print(f"Using calibration file: {config.CAL_FILE}", flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)

    try:
        while True:
            conn, addr = srv.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr, rc), daemon=True)
            t.start()
    finally:
        try:
            srv.close()
        except Exception:
            pass
        rc.shutdown()


if __name__ == "__main__":
    main()
