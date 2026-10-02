#!/usr/bin/env python3
"""Run PX4 NSH commands through MAVROS (MAVLink SERIAL_CONTROL on /mavlink/to and /mavlink/from).

  python3 scripts/px4_shell.py "logger status" "gps status"

Needs a running MAVROS that is connected to the FC; pymavlink is not required. Run inside the
container. Other scripts import Px4Shell after their own rospy.init_node().
"""

import struct
import sys
import time

import rospy
from mavros_msgs.msg import Mavlink

SERIAL_CONTROL_ID, SERIAL_CONTROL_CRC_EXTRA = 126, 220
DEV_SHELL = 10
FLAGS = 2 | 4  # RESPOND | EXCLUSIVE
PROMPT = "nsh> "
# Own component id: frames injected with the id of MAVROS (240) would break the sequence
# numbers PX4 uses for its rx loss statistics of the MAVROS link.
SYSTEM_ID, COMPONENT_ID = 1, 250


def x25(data):
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class Px4Shell:
    def __init__(self):
        self._seq = 0
        self._out = bytearray()
        self._pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)
        self._sub = rospy.Subscriber("/mavlink/from", Mavlink, self._on_msg, queue_size=500)
        time.sleep(1.0)  # let the publisher connect to MAVROS
        self.run("", wait=1.0)  # the first frame only starts the shell on the FC

    def _on_msg(self, msg):
        if msg.msgid != SERIAL_CONTROL_ID:
            return
        raw = struct.pack("<%dQ" % len(msg.payload64), *msg.payload64)[:msg.len].ljust(79, b"\0")
        self._out += raw[9:9 + raw[8]]

    def _send(self, data):
        payload = struct.pack("<IHBBB70s", 0, 0, DEV_SHELL, FLAGS, len(data), data).rstrip(b"\0") or b"\0"
        msg = Mavlink()
        msg.header.stamp = rospy.Time.now()
        msg.framing_status = Mavlink.FRAMING_OK
        msg.magic = Mavlink.MAVLINK_V20
        msg.len = len(payload)
        msg.seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        msg.sysid, msg.compid, msg.msgid = SYSTEM_ID, COMPONENT_ID, SERIAL_CONTROL_ID
        head = struct.pack("<BBBBBB", msg.len, 0, 0, msg.seq, msg.sysid, msg.compid)
        head += struct.pack("<I", SERIAL_CONTROL_ID)[:3]
        msg.checksum = x25(head + payload + bytes([SERIAL_CONTROL_CRC_EXTRA]))
        padded = payload.ljust((len(payload) + 7) // 8 * 8, b"\0")
        msg.payload64 = list(struct.unpack("<%dQ" % (len(padded) // 8), padded))
        self._pub.publish(msg)

    def run(self, command, wait=4.0):
        """Send one command line; return what the shell printed until its next prompt (or wait s)."""
        self._out = bytearray()
        line = (command + "\n").encode()
        for i in range(0, len(line), 70):
            self._send(line[i:i + 70])
        end = time.time() + wait
        while time.time() < end and not rospy.is_shutdown():
            time.sleep(0.2)
            text = self._out.decode(errors="replace")
            if command and text.rstrip(" ").endswith(PROMPT.rstrip()) and command in text:
                break
            self._send(b"")  # empty frame: ask for pending output
        text = self._out.decode(errors="replace").replace("\r", "")
        text = text.replace("\x1b[0m", "").replace("\x1b[K", "")  # terminal escapes of the NSH prompt
        return "\n".join(l for l in text.split("\n") if l.strip() not in ("", command, PROMPT.strip()))


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)
    rospy.init_node("px4_shell", anonymous=True)
    shell = Px4Shell()
    for cmd in sys.argv[1:]:
        print("### " + cmd)
        print(shell.run(cmd))
