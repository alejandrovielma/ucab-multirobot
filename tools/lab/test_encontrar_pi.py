import os
import socket
import sys
import threading

sys.path.insert(0, os.path.dirname(__file__))
from encontrar_pi import is_pi_mac, parse_arp, ssh_banner  # noqa: E402

WINDOWS = """
Interface: 169.254.59.65 --- 0x12
  Internet Address      Physical Address      Type
  169.254.59.2          d8-3a-dd-12-34-56     dynamic
  169.254.255.255       ff-ff-ff-ff-ff-ff     static
  224.0.0.22            01-00-5e-00-00-16     static
"""
LINUX = """? (192.168.1.50) at b8:27:eb:aa:bb:cc [ether] on eth0
? (192.168.1.1) at 00:11:22:33:44:55 [ether] on eth0
? (192.168.1.99) at <incomplete> on eth0
"""
IP_NEIGH = "10.68.17.47 dev wlan0 lladdr 2C:CF:67:01:02:03 REACHABLE\n"


def test_parse_arp_reads_windows_linux_and_ip_neigh_and_skips_broadcast():
    assert parse_arp(WINDOWS)["169.254.59.2"] == "d8:3a:dd:12:34:56"
    assert "169.254.255.255" not in parse_arp(WINDOWS)
    assert parse_arp(LINUX) == {"192.168.1.50": "b8:27:eb:aa:bb:cc", "192.168.1.1": "00:11:22:33:44:55"}
    assert parse_arp(IP_NEIGH.replace("lladdr", " "))["10.68.17.47"] == "2c:cf:67:01:02:03"


def test_raspberry_pi_macs():
    assert is_pi_mac("D8-3A-DD-12-34-56") and is_pi_mac("b8:27:eb:00:00:00")
    assert not is_pi_mac("00:11:22:33:44:55") and not is_pi_mac(None)


def test_ssh_banner_reads_an_open_port_and_none_when_closed():
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def serve():
        conn, _ = server.accept()
        conn.sendall(b"SSH-2.0-OpenSSH_9.6 Ubuntu\r\n")
        conn.close()

    threading.Thread(target=serve, daemon=True).start()
    import encontrar_pi

    real = socket.create_connection
    encontrar_pi.socket.create_connection = lambda addr, timeout=None: real(("127.0.0.1", port), timeout=timeout)
    try:
        assert ssh_banner("10.0.0.1").startswith("SSH-2.0-OpenSSH")
    finally:
        encontrar_pi.socket.create_connection = real
        server.close()
    assert ssh_banner("127.0.0.1", timeout=0.2) is None  # nothing listens on port 22 here
