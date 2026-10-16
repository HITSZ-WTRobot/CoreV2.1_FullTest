#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["rich>=13.9,<15"]
# ///
"""STM32H723 + DP83822 UDP integrity acceptance and frozen whole-board reports.

Run directly: uv run tools/eth_test.py
Single snapshot: uv run tools/eth_test.py --once --results-only
TTY defaults to a live dashboard; redirected output defaults to one snapshot.
Integrity runs once; subsequent refreshes only read real firmware reports.
No retransmission or forged ACK: lost RACK may leave the board BUSY until reset.
All peripheral values come from firmware; ETH completely offline still needs SWD.
See --help for protocol details, wiring and interaction/observation requirements.
"""

import argparse
from contextlib import nullcontext
import ipaddress
import math
import signal
import socket
import sys
import time
import threading

from rich.console import Console
from rich.live import Live

from eth_dashboard import Dashboard

LENGTHS = (8, 64, 256, 512, 1400)
ACK = b"CONFIRM ACK\n"
MAX_ROUNDS = (2**32 - 1) // len(LENGTHS)
RESULT_CAPACITY = 3072
RESULT_CHUNK = 1100
RESULT_HEADER = 80
UINT32_MAX = 2**32 - 1
HELP_EPILOG = r"""
Wiring/network:
  Connect the board RJ45 to the PC or a switch (10/100 Mbps Auto-MDIX).
  Board: 192.168.1.48/24, UDP 5001. PC example: 192.168.1.10/24.
  Select the PC interface with --bind when multiple routes exist.
  DP83822 MDIO address 1; PB14 RESET_N, PB10 active-low PHY INT (pull-up,
  falling-edge EXTI15_10, priority 5). RMII pin map: UserCode/ethernet_test.hpp.

Protocol (datagrams contain no trailing NUL):
  Every round sends A patterns of exactly 8, 64, 256, 512, 1400 bytes, in order.
  Bytes 0..3: A="FTA1", B="FTB1"; byte 4: length index 0..4.
  For offset i>=5: A[i]=(i*37 + index*19 + 0x53) & 255; B[i]=A[i] xor 0xA5.
  The board strictly validates A and submits the corresponding immutable B.
  The host checks every B byte before sending the next A (stop-and-wait).
  After all rounds: "CONFIRM <rounds> 0\n" -> fixed "CONFIRM ACK\n".
  This host feedback is what makes the board PASS, not successful sendTo calls.
  The board verifies the source endpoint, complete rounds and exact packet count.
  On failure the host best-effort sends "CONFIRM <completed-rounds> 1\n";
  board rejects it with fixed "ERR FAIL\n" and latches FAIL. Disconnection may
  prevent failure feedback reaching the board; no CONFIRM must not mean PASS.

Whole-board results (default after integrity, or --results-only):
  "RESULT\n" synchronously captures current firmware uptime and peripheral fields
  into one frozen snapshot (<=3072 bytes); chunks never mix different timestamps.
  "RESULT <session> <part> <parts> <total> <crc8>\n<body>" uses decimal numbers,
  zero-based parts and <=1100-byte bodies. CRC-8: polynomial 0x07, init/xorout 0,
  no reflection. The host checks endpoint, session, ordering, lengths and CRC,
  then sends "RACK <session> <part> <crc8>\n" exactly once per valid chunk.
  After the final RACK it must receive exactly "RESULT DONE\n".
  "RESULT NOT_READY\n": no report provider; "RESULT BUSY\n": awaiting an ACK.
  A query is independent of the integrity FAIL latch: --results-only can diagnose
  failed integrity runs without sending patterns or CONFIRM. It cannot make PASS.
  Every query gets a new snapshot/session. No invented host-side peripheral data.

Acceptance/limitations:
  --once (also the default when stdout is not a TTY) prints one dashboard.
  A TTY refreshes every --interval seconds; integrity is checked only once.
  --results-only never sends integrity patterns or CONFIRM.
  Live errors retain the last valid snapshot, marked stale; later intervals make
  a new RESULT query, not a retransmission of a chunk or ACK. BUSY may need reset.
  Ctrl-C requests stop; an active bounded report read finishes its genuine ACKs.
  Exit 0: integrity/CONFIRM succeeded (unless --results-only), and a complete
  CRC-checked, parsed report plus final RESULT DONE were received.
  Exit 1: integrity failure or no valid report; --once also fails on report errors.
  Exit 2: bad arguments or unable to configure the chosen local socket.
  There is NO retransmission: loss is a failure, never hidden by retrying.
  Firmware failures are cumulative; reset before a new acceptance run after a
  failed/incomplete integrity session. A completed run permits a new one.
  Fixed B contains no sequence number: identical-case duplicate/replay traffic is
  not distinguished. Use an isolated bench network and one host at a time.
  One mutable DMA TX buffer is sent once per chunk and frozen until the matching
  source/session/part/CRC ACK; lost ACK or failed submission NEVER permits timeout
  reuse. It may stay BUSY until that ACK arrives or the board is reset. No retries.
  ACKs assume a trusted bench host, not protection against malicious ACK forgery.
  Report uptime/counters describe capture time, not the later printing time.
  If ETH is completely unavailable, use SWD g_fulltest_report for diagnostics.
  Whole-MAC TX callbacks are diagnostic activity, not per-application completions.
  --timeout applies to each datagram; tune it for the bench, not to mask loss.
  Dashboard PASS/FAIL are firmware states, not host verdicts on other peripherals.
  Keys need two press/release cycles; I2C without external devices may stay WAIT.
  LED walk and TIM15 PWM RUN need manual observation; RUN is not a PASS or fault.
  uv run tools/eth_test.py installs its declared Rich dependency automatically.
"""


class BoardError(Exception):
    """A consumer-visible protocol or transport failure."""


def positive_rounds(text):
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError("rounds must be an integer") from None
    if not 1 <= value <= MAX_ROUNDS:
        raise argparse.ArgumentTypeError("rounds must be 1..%d" % MAX_ROUNDS)
    return value


def positive_timeout(text):
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError("timeout must be a number") from None
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("timeout must be finite and positive")
    return value


def positive_interval(text):
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError("interval must be a number") from None
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("interval must be finite and positive")
    return value


def ipv4(text):
    try:
        return str(ipaddress.IPv4Address(text))
    except ipaddress.AddressValueError:
        raise argparse.ArgumentTypeError("expected an IPv4 address, got %r" % text) from None


def make_pattern(index, response=False):
    length = LENGTHS[index]
    prefix = b"FTB1" if response else b"FTA1"
    mask = 0xA5 if response else 0
    return prefix + bytes((index,)) + bytes(
        ((offset * 37 + index * 19 + 0x53) & 0xFF) ^ mask
        for offset in range(5, length)
    )


def exchange(sock, request, expected, label):
    try:
        sent = sock.send(request)
        if sent != len(request):
            raise BoardError("%s: incomplete UDP submission %d/%d" % (label, sent, len(request)))
        reply = sock.recv(2048)
    except socket.timeout:
        raise BoardError("%s: timeout/disconnection (no retransmission)" % label) from None
    except OSError as error:
        raise BoardError("%s: network error: %s" % (label, error)) from error
    if reply == expected:
        return
    if reply.startswith(b"ERR "):
        raise BoardError("%s: board rejected request: %r" % (label, reply))
    if len(reply) != len(expected):
        raise BoardError("%s: wrong length %d, expected %d" % (label, len(reply), len(expected)))
    offset = next(i for i, (got, want) in enumerate(zip(reply, expected)) if got != want)
    raise BoardError("%s: byte %d is 0x%02x, expected 0x%02x" %
                     (label, offset, reply[offset], expected[offset]))


def crc8(data):
    """Match libs::CRC: width 8, poly 0x07, init/xorout 0, no reflection."""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x07 if value & 0x80 else 0)) & 0xFF
    return value


def send_result_message(sock, message):
    try:
        sent = sock.send(message)
    except OSError as error:
        raise BoardError("RESULT: send error: %s" % error) from error
    if sent != len(message):
        raise BoardError("RESULT: incomplete UDP submission %d/%d" % (sent, len(message)))


def receive_result_message(sock):
    try:
        return sock.recv(2048)
    except socket.timeout:
        raise BoardError("RESULT: timeout/disconnection (no retransmission; lost RACK may leave BUSY)") from None
    except OSError as error:
        raise BoardError("RESULT: receive error: %s" % error) from error


def read_report(sock):
    """Only ACK the exact next complete, CRC-valid chunk of one frozen snapshot."""
    send_result_message(sock, b"RESULT\n")
    metadata = None
    body_parts = []
    # ceil(3072/1100) bounds both memory and the number of receive/ACK steps.
    for expected_part in range((RESULT_CAPACITY + RESULT_CHUNK - 1) // RESULT_CHUNK):
        packet = receive_result_message(sock)
        header, separator, body = packet.partition(b"\n")
        if not separator or len(header) + 1 >= RESULT_HEADER:
            raise BoardError("RESULT: missing/oversized header: %r" % header[:RESULT_HEADER])
        fields = header.split(b" ")
        if len(fields) != 6 or fields[0] != b"RESULT":
            raise BoardError("RESULT: board rejected request or malformed header: %r" % header)
        numbers = fields[1:]
        if any(not field or len(field) > 10 or any(c < 48 or c > 57 for c in field)
               for field in numbers):
            raise BoardError("RESULT: header fields must be bounded decimal integers")
        session, part, parts, total, checksum = (int(field) for field in numbers)
        if (not 1 <= session <= UINT32_MAX or part != expected_part or
                not 1 <= total <= RESULT_CAPACITY or not 0 <= checksum <= 255 or
                parts != (total + RESULT_CHUNK - 1) // RESULT_CHUNK or part >= parts):
            raise BoardError("RESULT: invalid session, part ordering, count, total or CRC field")
        current_metadata = (session, parts, total)
        if metadata is None:
            metadata = current_metadata
        elif metadata != current_metadata:
            raise BoardError("RESULT: session/count/total changed within a snapshot")
        expected_length = min(RESULT_CHUNK, total - part * RESULT_CHUNK)
        if len(body) != expected_length:
            raise BoardError("RESULT: part %d length %d, expected %d" %
                             (part, len(body), expected_length))
        actual_crc = crc8(body)
        if actual_crc != checksum:
            raise BoardError("RESULT: part %d CRC %d, expected %d" % (part, actual_crc, checksum))
        body_parts.append(body)
        # This is the firmware's only permission to reuse its borrowed DMA buffer.
        # Never ACK a duplicate/out-of-order/damaged packet or retransmit an ACK.
        send_result_message(sock, b"RACK %d %d %d\n" % (session, part, checksum))
        if part + 1 == parts:
            done = receive_result_message(sock)
            if done != b"RESULT DONE\n":
                raise BoardError("RESULT: expected final RESULT DONE, received %r" % done)
            try:
                return b"".join(body_parts).decode("utf-8")
            except UnicodeDecodeError as error:
                raise BoardError("RESULT: firmware report is not UTF-8: %s" % error) from error
    raise BoardError("RESULT: incomplete snapshot")


def report_failure(sock, completed_rounds):
    """Notify once and consume its real rejection before diagnostic RESULT traffic."""
    try:
        exchange(sock, b"CONFIRM %d 1\n" % completed_rounds, b"ERR FAIL\n", "failure CONFIRM")
    except BoardError as error:
        return "Failure feedback unconfirmed: %s" % error
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Check UDP integrity once, then show a live whole-board dashboard (TTY).",
        epilog=HELP_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--board-ip", type=ipv4, default="192.168.1.48", help="board IPv4 address")
    parser.add_argument("--bind", type=ipv4, default="0.0.0.0", help="local IPv4 source address")
    parser.add_argument("--rounds", type=positive_rounds, default=20, help="five-length rounds (default: 20)")
    parser.add_argument("--results-only", action="store_true",
                        help="skip integrity/CONFIRM; read firmware peripheral report even after FAIL")
    parser.add_argument("--timeout", type=positive_timeout, default=1.0,
                        help="seconds per reply; no retransmission (default: 1)")
    parser.add_argument("--interval", type=positive_interval, default=1.0,
                        help="seconds between live report reads, finite and positive (default: 1)")
    parser.add_argument("--once", action="store_true",
                        help="print one dashboard and exit (automatic when stdout is not a TTY)")
    args = parser.parse_args(argv)
    realtime = sys.stdout.isatty() and not args.once
    console = Console()
    dashboard = Dashboard("%s:5001" % args.board_ip, args.interval, args.timeout)
    stop = threading.Event()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except OSError as error:
        print("ERROR creating local socket: %s" % error, file=sys.stderr)
        return 2
    with sock:
        try:
            sock.bind((args.bind, 0))
            sock.settimeout(args.timeout)
        except (OSError, OverflowError) as error:
            print("ERROR configuring local socket: %s" % error, file=sys.stderr)
            return 2

        try:
            sock.connect((args.board_ip, 5001))
        except OSError as error:
            print("FAIL connecting to board: %s" % error, file=sys.stderr)
            return 1

        def request_stop(signum, frame):
            # Raising KeyboardInterrupt here could strand a genuine RESULT chunk
            # before its RACK. Let the bounded in-flight exchange/read finish.
            stop.set()

        previous_sigint = signal.signal(signal.SIGINT, request_stop)
        integrity_failed = False
        have_report = False
        report_failed = False
        display = (Live(dashboard, console=console, refresh_per_second=8,
                        vertical_overflow="crop") if realtime else nullcontext())
        try:
            with display:
                if args.results_only:
                    dashboard.set_integrity(None, "Skipped (--results-only); firmware ETH state is independent")
                else:
                    completed_rounds = 0
                    verified = 0
                    start = time.monotonic()
                    dashboard.set_integrity(None, "Checking five A/B patterns per round; CONFIRM pending")
                    try:
                        # Every B is compared byte-for-byte; local send success is
                        # never evidence of board reception or peripheral PASS.
                        requests = tuple(make_pattern(i) for i in range(len(LENGTHS)))
                        expected = tuple(make_pattern(i, True) for i in range(len(LENGTHS)))
                        for round_index in range(args.rounds):
                            for index, length in enumerate(LENGTHS):
                                if stop.is_set():
                                    raise BoardError("interrupted before integrity/CONFIRM completed")
                                label = "round %d/%d length %d" % (round_index + 1, args.rounds, length)
                                dashboard.set_activity("Integrity: %s; %d packets verified" % (label, verified))
                                exchange(sock, requests[index], expected[index], label)
                                verified += 1
                            completed_rounds += 1
                        if stop.is_set():
                            raise BoardError("interrupted before final CONFIRM")
                        dashboard.set_activity("All patterns verified; awaiting final CONFIRM ACK")
                        exchange(sock, b"CONFIRM %d 0\n" % completed_rounds, ACK, "final CONFIRM")
                    except (BoardError, OSError) as error:
                        integrity_failed = True
                        reason = "FAIL after %d verified packets: %s" % (verified, error)
                        feedback_error = report_failure(sock, completed_rounds)
                        if feedback_error:
                            reason += "; " + feedback_error
                        dashboard.set_integrity(False, reason)
                        dashboard.set_error(reason)
                    else:
                        dashboard.set_integrity(
                            True, "%d packets verified, %d rounds, CONFIRM ACK, %.3fs" %
                            (verified, completed_rounds, time.monotonic() - start),
                        )

                while not stop.is_set():
                    dashboard.set_activity("Reading a new frozen firmware snapshot")
                    try:
                        # No stop check inside read_report: only validated chunks
                        # receive their single real RACK, even after Ctrl-C.
                        dashboard.update_report(read_report(sock))
                    except (BoardError, ValueError) as error:
                        report_failed = True
                        dashboard.set_error(
                            "%s. No chunk/ACK replay; if board stays BUSY, a board reset may be required." % error
                        )
                    else:
                        have_report = True
                        report_failed = False
                        dashboard.set_activity("Snapshot received; waiting for next regular sample" if realtime
                                               else "Single firmware snapshot received")
                    if not realtime:
                        break
                    # Event.wait keeps Ctrl-C responsive without interrupting the
                    # wire protocol. Cap individual waits for huge finite intervals.
                    remaining = args.interval
                    while remaining > 0 and not stop.is_set():
                        wait_started = time.monotonic()
                        stop.wait(min(remaining, threading.TIMEOUT_MAX))
                        remaining -= time.monotonic() - wait_started
        finally:
            # Live exits first, restoring the cursor even on render/network errors.
            signal.signal(signal.SIGINT, previous_sigint)

        if realtime:
            console.print("Stopped. " + dashboard.summary(), markup=False)
        else:
            console.print(dashboard)
        return int(integrity_failed or not have_report or report_failed)


if __name__ == "__main__":
    sys.exit(main())
