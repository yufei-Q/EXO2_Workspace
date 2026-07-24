#!/usr/bin/env python3

import time

import rospy
import serial
from std_msgs.msg import Bool


def read_exact(port, size, timeout):
    data = bytearray()
    deadline = time.monotonic() + timeout

    while len(data) < size and time.monotonic() < deadline:
        data.extend(port.read(size - len(data)))

    return bytes(data)


def main():
    rospy.init_node("usb_loopback_test")

    device = rospy.get_param("~port", "/dev/ttyACM0")
    baudrate = rospy.get_param("~baudrate", 115200)
    timeout = rospy.get_param("~timeout", 0.5)
    test_rate = rospy.get_param("~rate", 1.0)

    result_pub = rospy.Publisher("~ok", Bool, queue_size=10)

    try:
        usb = serial.Serial(
            port=device,
            baudrate=baudrate,
            timeout=0.05,
            write_timeout=timeout,
        )
    except serial.SerialException as error:
        rospy.logfatal("Cannot open %s: %s", device, error)
        return

    rospy.on_shutdown(usb.close)
    rospy.loginfo("USB loopback test started on %s", device)

    sequence = 0
    rate = rospy.Rate(test_rate)

    while not rospy.is_shutdown():
        message = "ROS_USB_TEST_{:06d}\n".format(sequence).encode("ascii")
        sequence += 1

        try:
            usb.reset_input_buffer()
            usb.write(message)
            usb.flush()
            reply = read_exact(usb, len(message), timeout)
            passed = reply == message
        except serial.SerialException as error:
            passed = False
            reply = b""
            rospy.logerr_throttle(1.0, "USB communication error: %s", error)

        result_pub.publish(Bool(data=passed))

        if passed:
            rospy.loginfo("PASS: %r", reply)
