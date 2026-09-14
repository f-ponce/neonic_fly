#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import time

def send_command(pub, cmd, label):
    msg = String()
    msg.data = cmd
    rospy.loginfo(f"Sending: {label} (cmd={cmd})")
    pub.publish(msg)

def wait(seconds, label):
    rospy.loginfo(f"Waiting {seconds}s: {label}")
    rospy.sleep(seconds)

def main():
    rospy.init_node('experiment_sequencer')

    cmd_pub = rospy.Publisher('/teensy/command', String, queue_size=10)

    # Give publisher time to connect
    rospy.sleep(1.0)

    rospy.loginfo("Starting experiment sequence...")

    # OFF 2s
    send_command(cmd_pub, '0', 'OFF')
    wait(2.0, 'off period')

#    # Stripes left 5s
#    send_command(cmd_pub, '1', 'Stripes LEFT')
#    wait(10.0, 'stripes left')

#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')

#    # Stripes right 5s
#    send_command(cmd_pub, '2', 'Stripes RIGHT')
#    wait(10.0, 'stripes right')

#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')

#    # Loom center
#    send_command(cmd_pub, '3', 'Loom CENTER')
#    wait(6.0, 'loom center')  # wait longer than loom duration to ensure it finishes

#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')

#    # Loom left
#    send_command(cmd_pub, '5', 'Loom LEFT')
#    wait(6.0, 'loom left')

#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')

#    # Loom right
#    send_command(cmd_pub, '4', 'Loom RIGHT')
#    wait(6.0, 'loom right')
#    
#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')

#    # Stripes left 5s
#    send_command(cmd_pub, 'L', 'Speed SLOW')
#    send_command(cmd_pub, '1', 'Stripes LEFT')
#    wait(10.0, 'stripes left')

#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')

#    # Stripes right 5s
#    send_command(cmd_pub, 'R', 'Speed SLOW')
#    send_command(cmd_pub, '1', 'Stripes RIGHT')
#    wait(10.0, 'stripes right')

#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')
#    
#    # Stripes left 5s
#    send_command(cmd_pub, 'H', 'Speed FAST')
#    send_command(cmd_pub, '1', 'Stripes LEFT')
#    wait(10.0, 'stripes left')

#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')

#    # Stripes right 5s
#    send_command(cmd_pub, 'H', 'Stripes FAST')
#    send_command(cmd_pub, '1', 'Stripes RIGHT')
#    wait(10.0, 'stripes right')
#    
#    # OFF 2s
#    send_command(cmd_pub, '0', 'OFF')
#    wait(2.0, 'off period')

    # CAMO 5s
    send_command(cmd_pub, 'L', 'Speed FAST')
    send_command(cmd_pub, '8', 'Camo LEFT')
    wait(10.0, 'camo left fast')
    
    # OFF 2s
    send_command(cmd_pub, '0', 'OFF')
    wait(2.0, 'off period')

    # CAMO 5s
    send_command(cmd_pub, 'L', 'Speed FAST')
    send_command(cmd_pub, '9', 'Camo RIGHT')
    wait(10.0, 'camo left fast')
    
    # Final OFF
    send_command(cmd_pub, '0', 'OFF')

    rospy.loginfo("Experiment sequence complete.")

if __name__ == '__main__':
    main()
