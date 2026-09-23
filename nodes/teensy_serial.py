#!/usr/bin/env python3

import rospy
import serial
import threading
from std_msgs.msg import String

SERIAL_PORT = '/dev/ttyACM0'  # change if needed
BAUD_RATE = 115200

ser = None
event_pub = None

def read_serial():
    """Continuously read from Teensy and publish events."""
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    ros_time = rospy.Time.now().to_sec()
                    msg = String()
                    msg.data = f"ROS_TIME={ros_time:.6f},{line}"
                    event_pub.publish(msg)
                    rospy.loginfo(f"Teensy event: {msg.data}")
        except Exception as e:
            rospy.logerr(f"Serial read error: {e}")


def _is_signed_float(s):
    """True for '0', '123', '-5', '45.5', '-12.3', etc. (used to validate
    S<speed> and P<position>, which the geometry-corrected .ino parses
    with atof -- real degrees, decimals allowed)."""
    if s.startswith('-'):
        s = s[1:]
    if s.count('.') > 1:
        return False
    s = s.replace('.', '', 1)
    return len(s) > 0 and s.isdigit()


def command_callback(msg):
    """Receive ROS command and send to Teensy.

    Accepted formats (matches patterns_091326_geometrycorrected.ino)
    ------------------------------------------------------------------
    '0' .. '9'         : immediate pattern select
                          (0 off, 1 stripe grid, 2 single stripe,
                           3/4/5 loom L/C/R paper-speed, 6/7/8 loom
                           L/C/R previous-speed, 9 random rotating
                           texture)
    'N<number>'        : general pattern select, e.g. 'N10'..'N13'
                          (diagnostics; also works for 0-9, same as
                          sending the bare digit)
    'S<signed number>' : set speed in REAL DEG/S for whichever movable
                          pattern (1, 2, or 9) is currently selected.
                          Positive/negative = direction, 0 = frozen.
                          Decimals OK. e.g. 'S45.5', 'S-45.5', 'S0'
    'P<number>'        : jump the current movable pattern (1, 2, or 9)
                          to an absolute azimuthal position in REAL
                          DEGREES, then motion continues from there.
                          Decimals OK. e.g. 'P-12.3'
    All commands are sent to the Teensy with a trailing newline.
    """
    cmd = msg.data.strip()
    ros_time = rospy.Time.now().to_sec()

    if cmd in [str(d) for d in range(10)]:
        rospy.loginfo(f"Sending command {cmd} at ROS_TIME={ros_time:.6f}")
        ser.write((cmd + '\n').encode())

    elif len(cmd) > 1 and cmd[0] == 'N' and cmd[1:].isdigit():
        rospy.loginfo(f"Sending pattern select {cmd} at ROS_TIME={ros_time:.6f}")
        ser.write((cmd + '\n').encode())

    elif len(cmd) > 1 and cmd[0] == 'S' and _is_signed_float(cmd[1:]):
        # Speed command — send at info level rather than debug to keep
        # visibility, but callers driving this at high rate (e.g. closed
        # loop) should prefer batching/log sparingly upstream if it gets noisy.
        rospy.loginfo(f"Sending speed {cmd} at ROS_TIME={ros_time:.6f}")
        ser.write((cmd + '\n').encode())

    elif len(cmd) > 1 and cmd[0] == 'P' and _is_signed_float(cmd[1:]):
        rospy.loginfo(f"Sending position {cmd} at ROS_TIME={ros_time:.6f}")
        ser.write((cmd + '\n').encode())

    else:
        rospy.logwarn(f"Unknown command: {cmd}")

def shutdown_hook():
    """Force the panel off on any shutdown (Ctrl+C, node kill, roslaunch
    stop, etc.) -- writes directly to the serial port rather than going
    through ROS pub/sub, since other nodes/topics may already be tearing
    down by the time this runs."""
    global ser
    if ser is not None and ser.is_open:
        try:
            rospy.loginfo("Shutdown: sending panel OFF")
            ser.write(b'0\n')
            ser.flush()
        except Exception as e:
            rospy.logerr(f"Shutdown: failed to send OFF: {e}")

def main():
    global ser, event_pub

    rospy.init_node('teensy_serial')

    serial_port = rospy.get_param('~serial_port', SERIAL_PORT)
    baud_rate = rospy.get_param('~baud_rate', BAUD_RATE)

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        rospy.loginfo(f"Connected to Teensy on {serial_port}")
    except Exception as e:
        rospy.logerr(f"Could not open serial port: {e}")
        return

    rospy.on_shutdown(shutdown_hook)

    event_pub = rospy.Publisher('/teensy/event', String, queue_size=10)
    rospy.Subscriber('/teensy/command', String, command_callback)

    # Start serial read thread
    t = threading.Thread(target=read_serial)
    t.daemon = True
    t.start()

    rospy.loginfo("Teensy serial node ready")
    rospy.spin()

    ser.close()

if __name__ == '__main__':
    main()
