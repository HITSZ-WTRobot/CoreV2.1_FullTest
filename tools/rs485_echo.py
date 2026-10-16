#!/usr/bin/env python3
"""RS485 host peer for the CoreV2.1_FullTest UART test (fulltest::uart).

USART2 (RS485, hardware driver enable on PD4) requires a real host on A/B:
this script replies FF/A to its R requests. The six TTL UARTs instead form
three bidirectional crossing pairs, without this script or local TX/RX jumpers:
UART9 PD15 TX -> USART3 PD9 RX; USART3 PD8 TX -> UART9 PD14 RX.
USART6 PC6 TX -> USART10 PE2 RX; USART10 PE3 TX -> USART6 PC7 RX.
USART1 PA9 TX -> UART4 PA11 RX; UART4 PA12 TX -> USART1 PA10 RX.
Use common ground and 3.3 V TTL; never short a port's own TX to its RX.
Both directions in each pair must pass three consecutive rounds for pair PASS.

Wiring (RS485, the case this tool exists for):

    board A   <-> USB-RS485 adapter A
    board B   <-> USB-RS485 adapter B
    board GND <-> adapter GND   (common reference; not a signal return path)
    120 ohm termination at BOTH ends of the pair. Many USB-RS485 adapters carry
    a fixed or switchable 120 ohm resistor; do not stack a second one on the
    same node. No termination, or termination at one end only, tends to show up
    as occasional bad frames at 115200 and is the first thing to check when the
    board reports timeouts while this tool reports requests.

The board is the master: it never answers anything, so this tool must only react
to request frames (kind 'R') coming from the board's port index. The adapter's
own receiver hears what the adapter transmits (2-wire loopback). That echo
arrives as a kind 'A' frame and is deliberately ignored here: it must never be
mistaken for the board, and it must never be answered, otherwise the pair
carries an endless answer ping-pong.

Usage:
    python3 tools/rs485_echo.py --port /dev/ttyUSB0
    python3 tools/rs485_echo.py --port COM7 --device-port 01 --count 50 -v

Frame (identical in the firmware, 15 bytes, 115200 8N1, no flow control):
    '$' src(2 hex) kind(1: R|A) seq(4 hex) pat(4 hex) crc(2 hex) '\\n'
    crc = CRC-8, poly 0x07, init 0x00, no reflection, no final xor, over the
    11 bytes between '$' and the crc field.

Firmware reception uses package-managed circular RX DMA after '$' header sync;
transmission uses UART interrupts. PASS requires three consecutive verified
round trips with actual TX completion. Firmware emits only fixed-size test
frames; observe board counters through SWD or the Ethernet board report.
"""

import argparse
import sys
import time
from typing import List, Optional, Tuple

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - message only
    sys.stderr.write(
        "pyserial is required:  python3 -m pip install pyserial\n"
        "The script only reads and writes a serial port, no board access is needed.\n"
    )
    raise SystemExit(2)

START = 0x24  # '$'
END = 0x0A  # '\n'
FRAME_LEN = 15
KIND_REQUEST = "R"
KIND_ANSWER = "A"
HOST_SRC = 0xFF

# Must match fulltest::uart::pattern_for() in UserCode/uart_test.cpp.
PATTERN_SEQ_MUL = 2654435761
PATTERN_SRC_MUL = 0x2B1D


def crc8(data: bytes) -> int:
    """CRC-8, polynomial 0x07, init 0x00, no reflection, no final xor."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def pattern_for(seq: int, src: int) -> int:
    mixed = (seq * PATTERN_SEQ_MUL) & 0xFFFFFFFF
    return ((mixed >> 16) ^ mixed ^ ((src * PATTERN_SRC_MUL) & 0xFFFFFFFF)) & 0xFFFF


def build_frame(src: int, kind: str, seq: int, pat: int) -> bytes:
    payload = ("%02X%s%04X%04X" % (src & 0xFF, kind, seq & 0xFFFF, pat & 0xFFFF)).encode("ascii")
    assert len(payload) == 11, len(payload)
    return bytes([START]) + payload + ("%02X" % crc8(payload)).encode("ascii") + bytes([END])


def parse_frame(frame: bytes) -> Optional[Tuple[int, str, int, int]]:
    """Return (src, kind, seq, pat) or None when the frame is not well formed."""
    if len(frame) != FRAME_LEN or frame[0] != START or frame[-1] != END:
        return None
    try:
        body = frame[1:12].decode("ascii")
        src = int(body[0:2], 16)
        kind = body[2]
        seq = int(body[3:7], 16)
        pat = int(body[7:11], 16)
        crc = int(frame[12:14].decode("ascii"), 16)
    except (UnicodeDecodeError, ValueError):
        return None
    if kind not in (KIND_REQUEST, KIND_ANSWER):
        return None
    if crc8(frame[1:12]) != crc:
        return None
    return src, kind, seq, pat


class FrameParser:
    """Host-side byte parser that re-anchors on '$'.

    Accepted frames are queued in `frames` so bytes read while waiting for bus
    turnaround are not dropped. Non-frame bytes are discarded. Firmware itself
    delegates header synchronization and fixed-frame DMA reception to UartRxSync.
    """

    def __init__(self) -> None:
        self.buf = bytearray()
        self.frames: List[Tuple[int, str, int, int]] = []
        self.dropped = 0

    def feed(self, byte: int) -> None:
        if not self.buf:
            if byte != START:
                self.dropped += 1
                return
            self.buf.append(byte)
            return
        self.buf.append(byte)
        if len(self.buf) < FRAME_LEN:
            return
        raw = bytes(self.buf)
        parsed = parse_frame(raw)
        if parsed is not None:
            self.buf.clear()
            self.frames.append(parsed)
            return
        self.dropped += 1
        # Re-anchor the host parser; firmware synchronization is package-owned.
        anchor = self.buf.find(START, 1)
        if anchor == -1:
            self.buf.clear()
        else:
            del self.buf[:anchor]


def drain_until_quiet(ser, parser: FrameParser, gap_s: float, deadline_s: float) -> None:
    """Parse whatever arrives until the pair has been idle for gap_s seconds.

    The board deasserts its RS485 driver one bit time after its last stop bit,
    so the peer must not answer before the bus is really released.
    """
    end = time.monotonic() + deadline_s
    last = time.monotonic()
    while True:
        waiting = ser.in_waiting
        if waiting:
            for byte in ser.read(waiting):
                parser.feed(byte)
            last = time.monotonic()
        elif (time.monotonic() - last) >= gap_s:
            return
        if time.monotonic() > end:
            return
        time.sleep(0.0005)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="rs485_echo.py",
        description="Answer fulltest::uart USART2 request frames through a real RS485 host.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Protocol: '$' src(2) kind(1) seq(4) pat(4) crc(2) '\\n'\n"
            "  A request (kind 'R') from --device-port is answered with kind 'A',\n"
            "  src FF, the same seq and pattern and a recomputed CRC, which is\n"
            "  exactly what the board verifies before it reports PASS on USART2.\n"
            "  TTL pairs use direct cross-wiring, not this host: UART9 <-> USART3,\n"
            "  USART6 <-> USART10 and USART1 <-> UART4, with TX -> peer RX in\n"
            "  both directions, common ground and 3.3 V levels; no local TX/RX shorts.\n"
            "  Frames from any other source, and the adapter's own answer echo, are\n"
            "  counted and ignored: they are never answered, so no echo storm starts.\n"
            "  Firmware emits fixed 15-byte frames only; use SWD records or the\n"
            "  Ethernet board report for board-local counters. PASS needs three\n"
            "  consecutive matching round trips with completed board TX, not merely\n"
            "  host receipt.\n"
            "Board timing: one request per port every ~120 ms, answer deadline 60 ms,\n"
            "bus turnaround guard 6 ms, 115200 8N1, no flow control, no parity.\n"
        ),
    )
    parser.add_argument("--port", required=True,
                        help="serial device of the USB-RS485 adapter, e.g. /dev/ttyUSB0 or COM7")
    parser.add_argument("--baud", type=int, default=115200,
                        help="baud rate (default 115200, must match the board)")
    parser.add_argument("--device-port", default="01",
                        help="hex port index the board transmits from, 00..06 (default 01 = USART2/RS485)")
    parser.add_argument("--count", type=int, default=0,
                        help="stop after this many replies (0 = run until Ctrl-C)")
    parser.add_argument("--answer-delay", type=float, default=0.0,
                        help="extra delay in seconds before replying (default 0)")
    parser.add_argument("--quiet-gap", type=float, default=0.002,
                        help="wait for this idle time on the pair before replying (default 0.002 s)")
    parser.add_argument("--listen-only", action="store_true",
                        help="parse and report but never transmit")
    parser.add_argument("--raw", action="store_true", help="also hex-dump every received byte")
    parser.add_argument("--idle-warn", type=float, default=10.0,
                        help="warn after this many seconds without a valid request (0 disables)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="print every accepted request and reply")
    args = parser.parse_args(argv)

    try:
        device_port = int(str(args.device_port), 16)
    except ValueError:
        parser.error("--device-port must be a hex byte such as 01")
    if not 0 <= device_port <= 6:
        parser.error("--device-port must be 00..06 (the board has seven ports)")

    try:
        ser = serial.Serial(args.port, args.baud, bytesize=8, parity="N", stopbits=1, timeout=0.05)
    except Exception as exc:  # serial.SerialException and friends
        sys.stderr.write("cannot open %s: %s\n" % (args.port, exc))
        return 2

    frames = FrameParser()
    ticks = {"requests": 0, "replies": 0, "other_src": 0, "own_answer": 0, "malformed": 0,
             "duplicates": 0}
    answered = []
    started = time.monotonic()
    last_request = started
    warned = False

    sys.stderr.write("listening on %s @ %d 8N1, board port %02X, FF/A host replies%s\n"
                     % (args.port, args.baud, device_port,
                        " (listen only)" if args.listen_only else ""))

    try:
        running = True
        while running:
            chunk = ser.read(1)
            if chunk:
                if args.raw:
                    sys.stdout.write("%02X " % chunk[0])
                    sys.stdout.flush()
                frames.feed(chunk[0])
            elif args.idle_warn and not warned and (time.monotonic() - last_request) > args.idle_warn:
                warned = True
                sys.stderr.write("no request from board port %02X for %.0fs: check A/B, GND, "
                                 "termination and that the board port is %02X\n"
                                 % (device_port, args.idle_warn, device_port))

            while frames.frames:
                src, kind, seq, pat = frames.frames.pop(0)

                if kind == KIND_ANSWER:
                    # Our own transmission coming back through the 2-wire pair
                    # (or a second tool on the bus). Never answer an answer.
                    ticks["own_answer"] += 1
                    if args.verbose:
                        sys.stderr.write("ignored answer echo src=%02X seq=%04X\n" % (src, seq))
                    continue

                if src != device_port:
                    ticks["other_src"] += 1
                    if args.verbose:
                        sys.stderr.write("ignored request from src=%02X (board port is %02X)\n"
                                         % (src, device_port))
                    continue

                expected = pattern_for(seq, src)
                if pat != expected:
                    ticks["malformed"] += 1
                    sys.stderr.write("request seq=%04X carries pattern %04X, expected %04X\n"
                                     % (seq, pat, expected))
                    continue

                ticks["requests"] += 1
                last_request = time.monotonic()
                warned = False
                if args.verbose:
                    sys.stderr.write("request src=%02X seq=%04X pat=%04X\n" % (src, seq, pat))

                if args.listen_only:
                    continue

                key = (src, seq)
                if key in answered:
                    ticks["duplicates"] += 1
                    if args.verbose:
                        sys.stderr.write("duplicate request seq=%04X ignored\n" % seq)
                    continue

                drain_until_quiet(ser, frames, args.quiet_gap, args.quiet_gap * 8.0)
                if args.answer_delay > 0:
                    time.sleep(args.answer_delay)

                reply = build_frame(HOST_SRC, KIND_ANSWER, seq, pat)
                ser.write(reply)
                ser.flush()

                ticks["replies"] += 1
                answered.append(key)
                del answered[:-32]
                if args.verbose:
                    sys.stderr.write("replied  %s\n" % reply)

                if ticks["requests"] % 50 == 0:
                    sys.stderr.write("requests=%d replies=%d malformed=%d duplicates=%d "
                                     "own_echo=%d other_src=%d\n"
                                     % (ticks["requests"], ticks["replies"], ticks["malformed"],
                                        ticks["duplicates"], ticks["own_answer"], ticks["other_src"]))

                if args.count and ticks["replies"] >= args.count:
                    running = False
                    break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    elapsed = max(time.monotonic() - started, 1e-6)
    sys.stdout.write(
        "\nsummary: requests=%d replies=%d malformed=%d duplicates=%d own_answer_echo=%d "
        "other_src=%d unparsed_bytes=%d elapsed=%.1fs rate=%.2f/s\n"
        % (ticks["requests"], ticks["replies"], ticks["malformed"], ticks["duplicates"],
           ticks["own_answer"], ticks["other_src"], frames.dropped,
           elapsed, ticks["replies"] / elapsed))
    sys.stdout.write(
        "expect the board record for port %02X to show good advancing once per request\n"
        "(tx and good equal, timeouts and bad stable). replies>0 here with growing board\n"
        "timeouts or bad means the board could not verify the answer, i.e. the return path\n"
        "is disturbed (A/B swapped, no termination, or the first byte truncated).\n"
        % device_port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
