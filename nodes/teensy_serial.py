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

def command_callback(msg):
    """Receive ROS command and send to Teensy.

    Accepted formats
    ----------------
    '0' .. '5', '8', '9'   : immediate pattern commands
                              (0 off, 1/2 stripes L/R, 3/4/5 loom,
                               8/9 camo L/R)
    'L' / 'M' / 'H'        : immediate stripe/camo speed select
                              (slow / medium / fast)
    'F'                    : immediate static (frozen) camo frame
    'T' / 'C' / 'O'        : immediate hardware diagnostic test patterns
                              (row test / static circle test / off-center test)
    'P<NNN>'               : closed-loop stripe grid position, e.g. 'P096'
    'Q<NNN>'               : closed-loop single-stripe position, e.g. 'Q096'
                              (also used for a fixed/static single stripe)
    'G<...>'               : live camo blotch tuning, e.g. 'G50,5,1,2,80'
    'D<...>'               : live camo dot-overlay tuning, e.g. 'D50,1,1'
    All commands are sent to the Teensy with a trailing newline.
    """
    cmd = msg.data.strip()
    ros_time = rospy.Time.now().to_sec()

    if cmd in ['0', '1', '2', '3', '4', '5', '8', '9', 'L', 'M', 'H', 'F', 'T', 'C', 'O']:
        rospy.loginfo(f"Sending command {cmd} at ROS_TIME={ros_time:.6f}")
        ser.write((cmd + '\n').encode())

    elif len(cmd) == 4 and cmd[0] in ('P', 'Q') and cmd[1:].isdigit():
        # Position command — send at debug level to avoid log spam at 100 Hz
        rospy.loginfo(f"Sending position {cmd} at ROS_TIME={ros_time:.6f}")
        ser.write((cmd + '\n').encode())

    elif len(cmd) > 1 and cmd[0] in ('G', 'D'):
        # Live camo tuning command — forward as-is, Teensy parses the rest.
        rospy.loginfo(f"Sending camo tuning {cmd} at ROS_TIME={ros_time:.6f}")
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
