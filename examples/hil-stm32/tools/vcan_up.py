#!/usr/bin/env python3
"""在目前的 netns 建一個 vcan 介面並帶起來——只用標準庫的 AF_NETLINK,不需要 iproute2。

    python3 tools/vcan_up.py vcan0

需要 CAP_NET_ADMIN(docker --cap-add NET_ADMIN)。核心收到未知 link 型別的 RTM_NEWLINK
時會自己 request_module("rtnl-link-vcan"),所以主機不必先 modprobe——但主機一樣多載一個模組。
這是 docs/hil/37 篇 §4 的路 ②。
"""
import os
import socket
import struct
import sys

NETLINK_ROUTE = 0
RTM_NEWLINK = 16
NLM_F_REQUEST = 0x01
NLM_F_ACK = 0x04
NLM_F_CREATE = 0x400
NLM_F_EXCL = 0x200
IFLA_IFNAME = 3
IFLA_LINKINFO = 18
IFLA_INFO_KIND = 1
IFF_UP = 0x1
NLMSG_ERROR = 2


def attr(kind, payload):
    n = 4 + len(payload)
    return struct.pack("=HH", n, kind) + payload + b"\0" * ((4 - n % 4) % 4)


def newlink(name, flags_ifi, nl_flags, seq, with_kind):
    ifinfo = struct.pack("=BxHiII", socket.AF_UNSPEC, 0, 0, flags_ifi, 0xFFFFFFFF if flags_ifi else 0)
    attrs = attr(IFLA_IFNAME, name.encode() + b"\0")
    if with_kind:
        attrs += attr(IFLA_LINKINFO, attr(IFLA_INFO_KIND, b"vcan\0"))
    body = ifinfo + attrs
    hdr = struct.pack("=IHHII", 16 + len(body), RTM_NEWLINK, nl_flags, seq, os.getpid())
    return hdr + body


def send(sock, msg):
    sock.send(msg)
    rep = sock.recv(4096)
    _, typ, _, _, _ = struct.unpack("=IHHII", rep[:16])
    if typ == NLMSG_ERROR:
        err = struct.unpack("=i", rep[16:20])[0]
        if err:
            raise OSError(-err, os.strerror(-err))


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "vcan0"
    s = socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, NETLINK_ROUTE)
    s.bind((0, 0))
    try:
        send(s, newlink(name, 0, NLM_F_REQUEST | NLM_F_ACK | NLM_F_CREATE | NLM_F_EXCL, 1, True))
        created = True
    except OSError as e:
        if e.errno != 17:  # EEXIST
            raise
        created = False
    # 帶起來:RTM_NEWLINK 只改 flags(ifi_change 全 1、ifi_flags IFF_UP)
    send(s, newlink(name, IFF_UP, NLM_F_REQUEST | NLM_F_ACK, 2, False))
    flags = int(open(f"/sys/class/net/{name}/flags").read(), 16)
    print(f"vcan_up: {name} {'created' if created else 'already exists'}, flags=0x{flags:x} ({'UP' if flags & IFF_UP else 'DOWN'})")


if __name__ == "__main__":
    main()
