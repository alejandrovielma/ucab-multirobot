#!/usr/bin/env python3
"""Find the Raspberry Pi robots on the network (Windows, Linux, macOS; Python standard library only).

    python tools/lab/encontrar_pi.py              # names, ARP table, sweep of your own /24 networks
    python tools/lab/encontrar_pi.py --linklocal  # also the 169.254.0.0/16 cable link (~1 min)
    python tools/lab/encontrar_pi.py --ip 10.68.17  # sweep another /24 prefix

It never logs in: it only asks "is port 22 open?" and reads the SSH banner and the ARP table. A host is
marked as a Raspberry Pi when its MAC address belongs to the Raspberry Pi Foundation / Trading.
"""

import argparse
import re
import socket
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

NAMES = ("phineas", "ferb", "perry", "raspberrypi")  # also tried with the .local suffix (mDNS)
# MAC prefixes (OUI) of Raspberry Pi boards: Pi 1-4 and Pi 5.
PI_OUIS = ("b8:27:eb", "dc:a6:32", "e4:5f:01", "d8:3a:dd", "2c:cf:67", "28:cd:c1")
ARP_LINE = re.compile(r"(\d{1,3}(?:\.\d{1,3}){3})\b[^\n]*?([0-9a-fA-F]{2}(?:[-:][0-9a-fA-F]{2}){5})")


def parse_arp(text: str) -> dict[str, str]:
    """{ip: mac with colons, lowercase} from the output of `arp -a` / `arp -an` (any OS)."""
    found = {}
    for ip, mac in ARP_LINE.findall(text):
        mac = mac.lower().replace("-", ":")
        if mac not in ("ff:ff:ff:ff:ff:ff", "00:00:00:00:00:00") and not ip.endswith(".255"):
            found[ip] = mac
    return found


def is_pi_mac(mac: str | None) -> bool:
    return bool(mac) and mac.lower().replace("-", ":").startswith(PI_OUIS)


def local_ips() -> list[str]:
    """IPv4 addresses of this machine (the ones a sweep makes sense on), without 127.x."""
    ips = set()
    try:
        ips.update(socket.gethostbyname_ex(socket.gethostname())[2])
    except OSError:
        pass
    for probe in ("10.255.255.255", "192.168.255.255", "169.254.255.255"):  # no packet is sent
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((probe, 1))
                ips.add(s.getsockname()[0])
        except OSError:
            pass
    return sorted(ip for ip in ips if not ip.startswith("127.") and ip != "0.0.0.0")


def ssh_banner(ip: str, timeout: float = 0.4) -> str | None:
    """The SSH banner if port 22 answers, else None."""
    try:
        with socket.create_connection((ip, 22), timeout=timeout) as s:
            s.settimeout(timeout)
            try:
                return s.recv(80).decode(errors="replace").strip() or "ssh"
            except OSError:
                return "ssh"
    except OSError:
        return None


def arp_table() -> dict[str, str]:
    for cmd in (["arp", "-a"], ["arp", "-an"], ["ip", "neigh"]):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=10).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        table = parse_arp(out.replace("lladdr", " "))
        if table:
            return table
    return {}


def sweep(prefixes: list[str], third_octets: range | None = None) -> list[str]:
    """Addresses with port 22 open in each a.b.c.0/24 (or, for a.b, every c in third_octets)."""
    targets = []
    for prefix in prefixes:
        parts = prefix.split(".")
        if len(parts) == 3:
            targets += [f"{prefix}.{i}" for i in range(1, 255)]
        elif len(parts) == 2 and third_octets is not None:
            targets += [f"{prefix}.{c}.{i}" for c in third_octets for i in range(1, 255)]
    with ThreadPoolExecutor(max_workers=256) as pool:
        return [ip for ip, banner in zip(targets, pool.map(ssh_banner, targets)) if banner]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", action="append", default=[], help="extra /24 prefix to sweep, for example 10.68.17")
    ap.add_argument("--linklocal", action="store_true", help="sweep all of 169.254.0.0/16 (the Ethernet cable)")
    args = ap.parse_args()

    candidates: dict[str, str] = {}  # ip -> how it was found
    for name in NAMES:
        for host in (name, f"{name}.local"):
            try:
                candidates.setdefault(socket.getaddrinfo(host, 22, socket.AF_INET)[0][4][0], f"name {host}")
            except OSError:
                pass
    arp = arp_table()
    for ip, mac in arp.items():
        if is_pi_mac(mac):
            candidates.setdefault(ip, "ARP: Raspberry Pi MAC")

    prefixes = [".".join(ip.split(".")[:3]) for ip in local_ips() if not ip.startswith("169.254.")] + args.ip
    link = args.linklocal or any(ip.startswith("169.254.") for ip in local_ips())
    print(f"sweeping {', '.join(p + '.0/24' for p in sorted(set(prefixes))) or 'no network'}"
          + (" and 169.254.0.0/16 (about a minute)" if args.linklocal else ""), file=sys.stderr)
    swept = sweep(sorted(set(prefixes)))
    if args.linklocal:
        swept += sweep(["169.254"], range(0, 256))
    elif link:
        print("tip: you have a cable link (169.254.x.x): add --linklocal to search it all", file=sys.stderr)
    arp = {**arp_table(), **arp}  # the sweep fills the table with the hosts that answered
    for ip in swept:
        candidates.setdefault(ip, "SSH open")

    rows = []
    for ip, how in sorted(candidates.items(), key=lambda kv: tuple(int(p) for p in kv[0].split("."))):
        banner = ssh_banner(ip)
        if banner is None:
            continue
        mac = arp.get(ip)
        rows.append((is_pi_mac(mac), ip, mac or "-", banner, how))
    if not rows:
        print("No host with SSH found. Is the Pi on (green LED), on the same Wi-Fi or on the cable?\n"
              "On Windows with a cable, wait ~30 s after plugging it in and try again with --linklocal.")
        return 1
    print(f"{'':3}{'IP':<17}{'MAC':<19}{'SSH banner':<34}found by")
    for pi, ip, mac, banner, how in sorted(rows, reverse=True):
        print(f"{'PI' if pi else '':3}{ip:<17}{mac:<19}{banner[:32]:<34}{how}")
    pis = [r for r in sorted(rows, reverse=True) if r[0]]
    if pis:
        print(f"\nTry:  ssh <user>@{pis[0][1]}   (the user and password are the ones of that robot)")
    else:
        print("\nNone has a Raspberry Pi MAC: those are other devices with SSH (a router, a VPN, a PC). The MAC is only\n"
              "known for hosts on your own network segment; if the Pi is behind a router, try the name of the robot.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
