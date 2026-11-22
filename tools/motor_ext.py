#!/usr/bin/env python3
"""Host tool for the MOTOR_EXT protocol of the extended firmware (Linux).

Talks to the board through the USB-C port labelled UART2 (USART3, 1 Mbaud).
Self-contained: standard library only, nothing to install.

Read-only commands:
  motor_ext.py --caps                which motor the firmware controls
  motor_ext.py --status              position, target, error, speed, state
  motor_ext.py --imu 5               listen to the IMU (and battery) frames for 5 s

Commands that change state but do not start a movement:
  motor_ext.py --zero                make the current position the origin
  motor_ext.py --cancel              cancel the move or the HOLD in progress
  motor_ext.py --stop                hard stop of the four motors

Commands that MOVE THE MOTOR (12 V connected, wheel free, motor switch at hand);
they are refused without --i-know-it-moves:
  motor_ext.py --deg 30 --i-know-it-moves
  motor_ext.py --ticks 110 --rps 0.25 --i-know-it-moves
  motor_ext.py --hold --i-know-it-moves

Guards while moving (a hard stop is sent if one trips): the shaft does not move
within 0.3/rps seconds (encoder not counting, or no motor supply), or it moves
away from the target (encoder sign opposite to the PWM sign). In the second case
the software stop does NOT hold the motor, because the loop pushes again: switch
the motors off on the board or cut the 12 V supply.

The port is HIWONDER_DEVICE if set, else --dev, else the only
usb-1a86_USB_Single_Serial_* entry in /dev/serial/by-id.
"""
import argparse
import math
import os
import select
import struct
import sys
import termios
import time

BY_ID = "/dev/serial/by-id"
BY_ID_PREFIX = "usb-1a86_USB_Single_Serial_"

FUNC_SYS, FUNC_MOTOR, FUNC_IMU, FUNC_MOTOR_EXT = 0x00, 0x03, 0x07, 0x0B
CAPS, MOVE, CANCEL, ZERO, STATUS, HOLD = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
STATE = {0: "idle", 1: "moving", 2: "reached", 3: "cancelled", 4: "timeout",
         5: "invalid", 6: "holding", 7: "stalled"}
NO_MOVE_RPS_S = 0.3     # no movement after 0.3/rps seconds -> encoder not counting


def discover_device(explicit):
    forced = os.environ.get("HIWONDER_DEVICE") or explicit
    if forced:
        return forced
    try:
        found = sorted(n for n in os.listdir(BY_ID) if n.startswith(BY_ID_PREFIX))
    except FileNotFoundError:
        found = []
    if len(found) == 1:
        return os.path.join(BY_ID, found[0])
    if not found:
        raise SystemExit(f"board not found in {BY_ID} (is the UART2 USB-C port "
                         "connected?); use --dev or HIWONDER_DEVICE=/dev/...")
    raise SystemExit("several boards are connected; choose one with --dev or "
                     "HIWONDER_DEVICE:\n  " +
                     "\n  ".join(os.path.join(BY_ID, n) for n in found))


def crc8(data):
    """CRC-8/MAXIM over function, length and payload."""
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
    return c


def frame(func, payload):
    body = bytes([func, len(payload)]) + payload
    return b"\xAA\x55" + body + bytes([crc8(body)])


def open_port(dev):
    # termios is set in this same process: settings made with stty are lost
    # when its descriptor closes, and the port would be read at 9600 baud.
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = termios.B1000000
    termios.tcsetattr(fd, termios.TCSANOW,
                      [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL | speed,
                       0, speed, speed, attrs[6]])
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def read_frames(fd, seconds):
    """Every frame with a valid CRC received in the next `seconds`."""
    buf, t0 = b"", time.time()
    while time.time() - t0 < seconds:
        ready, _, _ = select.select([fd], [], [], 0.05)
        if ready:
            try:
                data = os.read(fd, 8192)
                if data:
                    buf += data
            except BlockingIOError:
                pass
    out, i = [], 0
    while True:
        j = buf.find(b"\xAA\x55", i)
        if j < 0 or len(buf) - j < 5:
            break
        func, length = buf[j + 2], buf[j + 3]
        if len(buf) - j < length + 5:
            break
        if crc8(buf[j + 2:j + 4 + length]) == buf[j + length + 4]:
            out.append((func, buf[j + 4:j + 4 + length]))
            i = j + length + 5
        else:
            i = j + 2       # not a real header: look for the next one
    return out


def hard_stop(fd):
    """MOTOR sub-command 3: stop by mask, all four motors. Sent several times."""
    for _ in range(4):
        os.write(fd, frame(FUNC_MOTOR, bytes([0x03, 0x0F])))
        time.sleep(0.04)


def capabilities(fd):
    os.write(fd, frame(FUNC_MOTOR_EXT, bytes([CAPS])))
    for func, pl in read_frames(fd, 0.3):
        if func == FUNC_MOTOR_EXT and len(pl) == 8 and pl[0] == CAPS:
            ticks, = struct.unpack("<I", pl[4:8])
            return pl[1], pl[2], ticks
    return None


def show_status(fd, motor_id, ticks_per_turn, wait=0.3):
    os.write(fd, frame(FUNC_MOTOR_EXT, bytes([STATUS, motor_id])))
    for func, pl in read_frames(fd, wait):
        if func == FUNC_MOTOR_EXT and len(pl) >= 20 and pl[0] == STATUS:
            state, active = pl[2], pl[3]
            pos, target, err = struct.unpack("<iii", pl[4:16])
            rps, = struct.unpack("<f", pl[16:20])
            print(f"  M{pl[1] + 1} state={state}({STATE.get(state, '?')}) active={active}  "
                  f"pos={pos} target={target} error={err}  speed={rps:+.3f} rev/s  "
                  f"({pos * 360.0 / ticks_per_turn:+.1f} deg)")
            return state, active, pos, err
    print("  (no STATUS reply)")
    return None


def listen_imu(fd, seconds):
    frames = read_frames(fd, seconds)
    imu = [struct.unpack("<6f", pl) for func, pl in frames if func == FUNC_IMU and len(pl) == 24]
    battery = [struct.unpack("<H", pl[1:3])[0] for func, pl in frames
               if func == FUNC_SYS and len(pl) == 3 and pl[0] == 0x04]
    print(f"valid frames: {len(frames)} in {seconds:g} s")
    if battery:
        print(f"battery     : {battery[-1] / 1000.0:.2f} V  (about 4.1 V means USB power only)")
    if not imu:
        print("IMU         : no frames (the vendor firmware does not send them)")
        return 1
    n = len(imu)
    mean = [sum(s[k] for s in imu) / n for k in range(6)]
    norm = sum(math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2) for s in imu) / n
    print(f"IMU         : {n} frames, {n / seconds:.1f} Hz")
    print(f"  accel mean: {mean[0]:+.3f} {mean[1]:+.3f} {mean[2]:+.3f} m/s^2   |a| = {norm:.3f}")
    print(f"  gyro mean : {mean[3]:+.4f} {mean[4]:+.4f} {mean[5]:+.4f} rad/s  "
          f"({math.degrees(mean[3]):+.2f} {math.degrees(mean[4]):+.2f} {math.degrees(mean[5]):+.2f} deg/s)")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--caps", action="store_true", help="ask for CAPABILITIES")
    g.add_argument("--status", action="store_true", help="read the position status")
    g.add_argument("--imu", type=float, metavar="SECONDS", help="listen to IMU and battery frames")
    g.add_argument("--zero", action="store_true", help="current position becomes the origin")
    g.add_argument("--cancel", action="store_true", help="cancel the move or the HOLD")
    g.add_argument("--stop", action="store_true", help="hard stop of the four motors")
    g.add_argument("--deg", type=float, help="move by degrees of the output shaft (MOVES)")
    g.add_argument("--ticks", type=int, help="move by encoder ticks (MOVES)")
    g.add_argument("--hold", action="store_true", help="position HOLD (MOVES if pushed)")
    p.add_argument("--i-know-it-moves", action="store_true",
                   help="required by --deg, --ticks and --hold")
    p.add_argument("--motor", type=int, default=4, choices=(1, 2, 3, 4),
                   help="motor port 1..4 (default 4, the one the released image controls)")
    p.add_argument("--rps", type=float, default=0.25,
                   help="speed limit in rev/s (default 0.25; the firmware accepts up to 5.7)")
    p.add_argument("--tol", type=int, default=15, help="tolerance in ticks (default 15)")
    p.add_argument("--timeout", type=int, default=5000, help="move timeout in ms (default 5000)")
    p.add_argument("--max-pulse", type=int, default=400,
                   help="PWM cap of the HOLD, 300..1000 (default 400 = 40 %%)")
    p.add_argument("--dev", help="serial port of the board")
    a = p.parse_args()
    motor_id = a.motor - 1
    moves = a.deg is not None or a.ticks is not None or a.hold
    if moves and not a.i_know_it_moves:
        print("this command can move the motor: add --i-know-it-moves once the wheel is "
              "free and the motor switch is within reach")
        return 2

    fd = open_port(discover_device(a.dev))
    try:
        if a.stop:
            hard_stop(fd)
            print("hard stop sent (M1..M4)")
            return 0
        if a.imu is not None:
            return listen_imu(fd, a.imu)

        caps = capabilities(fd)
        if caps is None:
            print("no reply to CAPABILITIES: this is not the extended firmware")
            return 2
        version, mask, ticks_per_turn = caps
        controlled = ", ".join(f"M{i + 1}" for i in range(4) if mask & (1 << i))
        print(f"MOTOR_EXT v{version}: position control on {controlled}, "
              f"{ticks_per_turn} ticks per turn")
        if a.caps:
            return 0
        if not mask & (1 << motor_id):
            print(f"the firmware does not control the position of M{a.motor}")
            return 2

        if a.status:
            show_status(fd, motor_id, ticks_per_turn)
            return 0
        if a.cancel:
            os.write(fd, frame(FUNC_MOTOR_EXT, bytes([CANCEL, motor_id])))
            print("cancel sent")
            show_status(fd, motor_id, ticks_per_turn)
            return 0
        if a.zero:
            os.write(fd, frame(FUNC_MOTOR_EXT, bytes([ZERO, motor_id])))
            print("origin set at the current position")
            show_status(fd, motor_id, ticks_per_turn)
            return 0
        if a.hold:
            if version < 3:
                print(f"this firmware has no HOLD (MOTOR_EXT v{version} < 3)")
                return 2
            pl = bytes([HOLD, motor_id]) + struct.pack("<fHH", a.rps, a.tol, a.max_pulse)
            os.write(fd, frame(FUNC_MOTOR_EXT, pl))
            print(f"HOLD M{a.motor}: back at {a.rps} rev/s, tolerance {a.tol}, PWM <= {a.max_pulse}")
            show_status(fd, motor_id, ticks_per_turn)
            return 0

        ticks = a.ticks if a.ticks is not None else round(a.deg * ticks_per_turn / 360.0)
        if ticks == 0:
            print("a move of 0 ticks is rejected by the firmware")
            return 2
        if not 0 < a.rps <= 10.0:
            print("--rps out of range (0, 10]")
            return 2
        # The real limit is the firmware's (5.7 rev/s in the released image):
        # above it the MOVE is rejected, which is reported below.

        before = show_status(fd, motor_id, ticks_per_turn)
        if before is None:
            return 2
        pos0 = before[2]
        print(f"MOVE_RELATIVE M{a.motor} {ticks} ticks "
              f"({ticks * 360.0 / ticks_per_turn:+.1f} deg) at {a.rps} rev/s, "
              f"tolerance {a.tol}, timeout {a.timeout} ms")
        pl = bytes([MOVE, motor_id]) + struct.pack("<ifHH", ticks, a.rps, a.tol, a.timeout)
        os.write(fd, frame(FUNC_MOTOR_EXT, pl))
        t0 = time.time()
        no_move_s = max(0.6, NO_MOVE_RPS_S / a.rps)
        reply = [pl for func, pl in read_frames(fd, 0.15)
                 if func == FUNC_MOTOR_EXT and len(pl) >= 20 and pl[0] == MOVE]
        if reply and reply[0][2] == 5:
            print("MOVE rejected by the firmware (speed above its limit, or tolerance "
                  "or timeout out of range)")
            return 2

        wrong_way = max(2 * a.tol, 30)
        while time.time() - t0 < a.timeout / 1000.0 + 3:
            r = show_status(fd, motor_id, ticks_per_turn, 0.1)
            if r is None:
                continue
            state, active, pos, _ = r
            moved = pos - pos0
            if active and moved * ticks < 0 and abs(moved) > wrong_way:
                hard_stop(fd)
                print("!! moving the wrong way: the encoder sign is opposite to the PWM sign.\n"
                      "!! A stop was sent, but the loop may push again:\n"
                      "!! SWITCH THE MOTORS OFF on the board or cut the 12 V supply.")
                return 3
            if active and abs(moved) < 3 and time.time() - t0 > no_move_s:
                hard_stop(fd)
                print("!! no movement measured: encoder not counting or no 12 V supply.\n"
                      "!! Hard stop sent.")
                return 3
            if state in (2, 3, 4, 5) and active == 0:
                print(f"-> {STATE.get(state)} in {time.time() - t0:.1f} s, moved {moved} ticks "
                      f"({moved * 360.0 / ticks_per_turn:+.1f} deg)")
                return 0 if state == 2 else 1
        hard_stop(fd)
        print("-> not finished in the expected time; hard stop sent")
        return 1
    except KeyboardInterrupt:
        hard_stop(fd)
        print("\ninterrupted: hard stop sent")
        return 130
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
