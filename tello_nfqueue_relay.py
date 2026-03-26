#!/usr/bin/env python3
"""
Tello UDP relay for Linux using iptables NFQUEUE + netfilterqueue + scapy.
Linux equivalent of tello_windivert_relay_v4-1.py (Windows/pydivert).

Dependencies:
    pip install netfilterqueue scapy

System requirements:
    sudo apt install libnetfilter-queue-dev
    Must be run as root (or with CAP_NET_ADMIN).

How it works:
    iptables rules send matching drone UDP packets to NFQUEUE.
    This script reads each packet, rewrites ports, recalculates checksums,
    and reinjects them — identical logic to the WinDivert version.

Outbound (PC → Drone):
    dst_ip=tello_ip, dst_port=mapped_cmd_port  →  rewrite dst_port to 8889

Inbound (Drone → PC):
    src_ip=tello_ip, dst_port=8890             →  rewrite dst_port to mapped_state_port
    src_ip=tello_ip, dst_port=11111            →  rewrite dst_port to mapped_video_port
    src_ip=tello_ip, src_port=8889             →  rewrite src_port to mapped_cmd_port

Usage:
    sudo python3 tello_nfqueue_relay.py --config drones.json [--ids 5,6] [--verbose] [--diag] [--queue-num 0]

The script automatically installs and removes the iptables rules on start/stop.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional

try:
    import netfilterqueue
except ImportError as exc:
    raise SystemExit(
        "netfilterqueue is not installed.\n"
        "Run: sudo apt install libnetfilter-queue-dev && pip install netfilterqueue"
    ) from exc

try:
    from scapy.layers.inet import IP, UDP
    from scapy.packet import Raw
except ImportError as exc:
    raise SystemExit("scapy is not installed. Run: pip install scapy") from exc


TELLO_NATIVE_CMD_PORT   = 8889
TELLO_NATIVE_STATE_PORT = 8890
TELLO_NATIVE_VIDEO_PORT = 11111


# ── Data structures ────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class DroneMap:
    drone_id:          int
    tello_ip:          str
    mapped_cmd_port:   int
    mapped_state_port: int
    mapped_video_port: int


class Stats:
    def __init__(self) -> None:
        self.cmd_out_rewritten   = 0
        self.cmd_in_rewritten    = 0
        self.state_in_rewritten  = 0
        self.video_in_rewritten  = 0
        self.other_seen          = 0
        self.start_time          = time.time()

    def snapshot(self) -> str:
        uptime = time.time() - self.start_time
        return (
            f"uptime={uptime:8.1f}s | "
            f"cmd_out={self.cmd_out_rewritten} | "
            f"cmd_in={self.cmd_in_rewritten} | "
            f"state_in={self.state_in_rewritten} | "
            f"video_in={self.video_in_rewritten} | "
            f"other={self.other_seen}"
        )


# ── Config loading ─────────────────────────────────────────────────────────────

def parse_ids(text: str) -> set[int]:
    return {int(p.strip()) for p in text.split(",") if p.strip()}


def load_drones(path: Path, only_ids: Optional[set[int]] = None) -> Dict[str, DroneMap]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    out: Dict[str, DroneMap] = {}
    for item in raw:
        did = int(item["id"])
        if only_ids and did not in only_ids:
            continue
        ip = str(item["TELLO_IP"])
        out[ip] = DroneMap(
            drone_id=did,
            tello_ip=ip,
            mapped_cmd_port=int(item["TELLO_PORT"]),
            mapped_state_port=int(item["LOCAL_PORT"]),
            mapped_video_port=int(item["VIDEO_PORT"]),
        )
    return out


# ── iptables rule management ───────────────────────────────────────────────────

def _run(cmd: List[str], check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, check=check, capture_output=True, text=True)


class IptablesManager:
    """Installs and removes iptables NFQUEUE rules for drone traffic."""

    def __init__(self, drones: Dict[str, DroneMap], queue_num: int) -> None:
        self.drones    = drones
        self.queue_num = queue_num
        self._rules: List[List[str]] = []   # store inserted rules for cleanup

    def install(self) -> None:
        for drone in self.drones.values():
            ip = drone.tello_ip
            q  = str(self.queue_num)

            rules = [
                # Outbound: PC → drone cmd port (mapped port → 8889)
                ["iptables", "-I", "OUTPUT", "-p", "udp",
                 "-d", ip, "--dport", str(drone.mapped_cmd_port),
                 "-j", "NFQUEUE", "--queue-num", q],

                # Inbound state: drone:8890 → PC
                ["iptables", "-I", "INPUT", "-p", "udp",
                 "-s", ip, "--dport", str(TELLO_NATIVE_STATE_PORT),
                 "-j", "NFQUEUE", "--queue-num", q],

                # Inbound video: drone:11111 → PC
                ["iptables", "-I", "INPUT", "-p", "udp",
                 "-s", ip, "--dport", str(TELLO_NATIVE_VIDEO_PORT),
                 "-j", "NFQUEUE", "--queue-num", q],

                # Inbound cmd response: drone:8889 → PC
                ["iptables", "-I", "INPUT", "-p", "udp",
                 "-s", ip, "--sport", str(TELLO_NATIVE_CMD_PORT),
                 "-j", "NFQUEUE", "--queue-num", q],
            ]

            for rule in rules:
                _run(rule)
                # Store the DELETE equivalent for cleanup
                self._rules.append(
                    [rule[0], "-D"] + rule[2:]
                )
                print(f"[IPTABLES] Installed: {' '.join(rule[3:])}", flush=True)

    def remove(self) -> None:
        for rule in reversed(self._rules):
            _run(rule, check=False)   # best-effort; don't crash on cleanup
            print(f"[IPTABLES] Removed: {' '.join(rule[3:])}", flush=True)
        self._rules.clear()


# ── Relay core ─────────────────────────────────────────────────────────────────

class Relay:
    def __init__(
        self,
        drones:         Dict[str, DroneMap],
        queue_num:      int   = 0,
        verbose:        bool  = False,
        diag:           bool  = False,
        stats_interval: float = 5.0,
    ) -> None:
        self.drones         = drones
        self.queue_num      = queue_num
        self.verbose        = verbose
        self.diag           = diag
        self.stats_interval = stats_interval
        self.stats          = Stats()
        self._running       = True
        self._last_stats    = 0.0

        # Reverse lookup: (tello_ip, mapped_cmd_port) → drone  — for outbound rewrites
        self._outbound_cmd_map: Dict[tuple, DroneMap] = {
            (d.tello_ip, d.mapped_cmd_port): d for d in drones.values()
        }

        self._iptables = IptablesManager(drones, queue_num)

    def stop(self, *_args) -> None:
        self._running = False

    def log(self, message: str) -> None:
        print(message, flush=True)

    def maybe_log_stats(self) -> None:
        now = time.time()
        if now - self._last_stats >= self.stats_interval:
            self.log("[STATS] " + self.stats.snapshot())
            self._last_stats = now

    # ── Packet handler (called by netfilterqueue for every intercepted packet) ──

    def _handle_packet(self, nfpkt) -> None:
        """
        Called by netfilterqueue for every intercepted packet.
        Parse with scapy, rewrite ports if needed, recalculate checksums, release.
        """
        raw = nfpkt.get_payload()
        pkt = IP(raw)

        if not pkt.haslayer(UDP):
            nfpkt.accept()
            return

        modified = self._rewrite(pkt)

        if modified:
            # Delete scapy's cached checksums so they are recalculated on serialisation
            del pkt[IP].chksum
            del pkt[UDP].chksum
            nfpkt.set_payload(bytes(pkt))

        nfpkt.accept()
        self.maybe_log_stats()

    def _rewrite(self, pkt) -> bool:
        """
        Inspect and rewrite a scapy IP/UDP packet.
        Returns True if the packet was modified.
        """
        ip  = pkt[IP]
        udp = pkt[UDP]

        src_ip   = ip.src
        dst_ip   = ip.dst
        src_port = udp.sport
        dst_port = udp.dport

        if self.diag:
            if src_ip in self.drones or dst_ip in self.drones:
                self.log(f"[RAW] {src_ip}:{src_port} -> {dst_ip}:{dst_port}")

        # ── Outbound: PC → Drone ──────────────────────────────────────────────
        # PC sends to tello_ip:mapped_cmd_port → rewrite dst_port to 8889
        outbound_key = (dst_ip, dst_port)
        if outbound_key in self._outbound_cmd_map:
            drone = self._outbound_cmd_map[outbound_key]
            old   = dst_port
            udp.dport = TELLO_NATIVE_CMD_PORT
            self.stats.cmd_out_rewritten += 1
            if self.verbose:
                self.log(
                    f"[CMD OUT] {src_ip}:{src_port} -> {dst_ip}:{old} "
                    f"rewritten dst to {dst_ip}:{TELLO_NATIVE_CMD_PORT} (ID {drone.drone_id})"
                )
            return True

        # ── Inbound: Drone → PC ───────────────────────────────────────────────
        if src_ip not in self.drones:
            self.stats.other_seen += 1
            return False

        drone = self.drones[src_ip]

        # State: drone → PC:8890 → rewrite dst to mapped_state_port
        if dst_port == TELLO_NATIVE_STATE_PORT:
            old = dst_port
            udp.dport = drone.mapped_state_port
            self.stats.state_in_rewritten += 1
            if self.verbose:
                self.log(
                    f"[STATE  ] {src_ip}:{src_port} -> {dst_ip}:{old} "
                    f"rewritten dst to {dst_ip}:{drone.mapped_state_port} (ID {drone.drone_id})"
                )
            return True

        # Video: drone → PC:11111 → rewrite dst to mapped_video_port
        if dst_port == TELLO_NATIVE_VIDEO_PORT:
            old = dst_port
            udp.dport = drone.mapped_video_port
            self.stats.video_in_rewritten += 1
            if self.verbose:
                self.log(
                    f"[VIDEO  ] {src_ip}:{src_port} -> {dst_ip}:{old} "
                    f"rewritten dst to {dst_ip}:{drone.mapped_video_port} (ID {drone.drone_id})"
                )
            return True

        # Cmd response: drone:8889 → PC → rewrite src_port to mapped_cmd_port
        if src_port == TELLO_NATIVE_CMD_PORT:
            old = src_port
            udp.sport = drone.mapped_cmd_port
            self.stats.cmd_in_rewritten += 1
            if self.verbose:
                self.log(
                    f"[CMD IN ] {src_ip}:{old} -> {dst_ip}:{dst_port} "
                    f"rewritten src to {src_ip}:{drone.mapped_cmd_port} (ID {drone.drone_id})"
                )
            return True

        self.stats.other_seen += 1
        return False

    # ── Main loop ──────────────────────────────────────────────────────────────

    def run(self) -> int:
        if os.geteuid() != 0:
            raise SystemExit("This script must be run as root (sudo).")

        signal.signal(signal.SIGINT,  self.stop)
        signal.signal(signal.SIGTERM, self.stop)

        self.log("[INFO] Starting Linux NFQUEUE relay")
        self.log("[INFO] Loaded drones: " + ", ".join(
            f"ID {d.drone_id}={d.tello_ip}" for d in self.drones.values()
        ))

        # Install iptables rules
        self._iptables.install()

        try:
            nfqueue = netfilterqueue.NetfilterQueue()
            nfqueue.bind(self.queue_num, self._handle_packet)
            self.log(f"[INFO] Bound to NFQUEUE {self.queue_num} — relay running.")

            # Use the file-descriptor based loop so SIGINT can interrupt it
            import select
            import socket
            s = socket.fromfd(nfqueue.get_fd(), socket.AF_UNIX, socket.SOCK_STREAM)
            while self._running:
                try:
                    ready, _, _ = select.select([s], [], [], 1.0)
                    if ready:
                        nfqueue.run(block=False)
                except Exception:
                    pass
                self.maybe_log_stats()

        except OSError as exc:
            self.log(f"[ERROR] NFQUEUE bind failed: {exc}")
            return 1
        finally:
            try:
                nfqueue.unbind()
            except Exception:
                pass
            self._iptables.remove()
            self.log("[INFO] Relay stopped. iptables rules removed.")

        return 0


# ── Entry point ────────────────────────────────────────────────────────────────

def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Linux UDP relay for port-mapped Tello control/state/video (iptables NFQUEUE)"
    )
    parser.add_argument("--config",         type=Path,  default=Path("drones.json"))
    parser.add_argument("--ids",            type=str,   default="")
    parser.add_argument("--queue-num",      type=int,   default=0,
                        help="NFQUEUE queue number (default 0)")
    parser.add_argument("--verbose",        action="store_true")
    parser.add_argument("--diag",           action="store_true")
    parser.add_argument("--stats-interval", type=float, default=5.0)
    args = parser.parse_args(list(argv) if argv is not None else None)

    only_ids = parse_ids(args.ids) if args.ids else None
    drones   = load_drones(args.config, only_ids)
    if not drones:
        raise SystemExit("No drones loaded from config / ids selection.")

    relay = Relay(
        drones=drones,
        queue_num=args.queue_num,
        verbose=args.verbose,
        diag=args.diag,
        stats_interval=args.stats_interval,
    )
    return relay.run()


if __name__ == "__main__":
    raise SystemExit(main())