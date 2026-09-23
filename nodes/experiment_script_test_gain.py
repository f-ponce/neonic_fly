#!/usr/bin/env python3
"""
Gain comparison: closed loop on single stripe ONLY, testing three GAIN
values, TRIAL_REPEATS trials each, shuffled order. No motion trials, no
loom, no "in-between" vs "proper" CL distinction -- just 9 trials total
(3 gains x 3 repeats), each a straight 20s closed-loop window.

Requires:
    - flystate2teensy.py (or flystate2teensy_velocitymode.py) running,
      fixed command topic (not anonymous)
    - patterns_091326_geometrycorrected.ino on the Teensy
    - teensy_serial.py with the N/S/P whitelist (S/P accept signed decimals)
"""
import random
import rospy
from std_msgs.msg import String

# ── Parameters ────────────────────────────────────────────────────────────
OFF_DURATION           = 2.0
STATIC_DURATION        = 1.0    # brief settle before each trial's closed loop starts
TRIAL_DURATION_S       = 60.0
SINGLE_STRIPE_PATTERN  = 2
STATIC_POSITION_DEG    = 0.0

GAINS_DEG_RAD = [-250, -300, -350.0]  # deg/rad; see flystate2teensy.py for the Giraldo et al. 2018
                                           # derivation of the -270 end of this range
TRIAL_REPEATS = 2                          # each gain value presented this many times

FLYSTATE_NODE_COMMAND_TOPIC = '/Flystate2Teensy/command'


def send_command(pub, cmd, label):
    msg = String()
    msg.data = cmd
    rospy.loginfo(f"Sending: {label} (cmd={cmd})")
    pub.publish(msg)


def wait(seconds, label):
    rospy.loginfo(f"Waiting {seconds}s: {label}")
    rospy.sleep(seconds)


def build_trial_list():
    trials = []
    for gain in GAINS_DEG_RAD:
        for _ in range(TRIAL_REPEATS):
            trials.append({'gain': gain, 'label': f'GAIN {gain:.0f}'})
    random.shuffle(trials)
    return trials


def run_gain_trial(teensy_pub, closedloop_pub, trial, idx, total):
    rospy.loginfo(f"--- Trial {idx}/{total}: {trial['label']} ---")

    send_command(teensy_pub, f"N{SINGLE_STRIPE_PATTERN}", "Select single stripe")
    send_command(teensy_pub, 'S0', 'Zero speed (freeze)')
    send_command(teensy_pub, f'P{STATIC_POSITION_DEG}', 'Static position (pre)')
    wait(STATIC_DURATION, 'static (pre)')

    send_command(closedloop_pub, f"gain {trial['gain']}", f"Set gain {trial['gain']}")
    send_command(closedloop_pub, f"pattern {SINGLE_STRIPE_PATTERN}", f"Set pattern {SINGLE_STRIPE_PATTERN}")
    send_command(closedloop_pub, 'start', 'Closed-loop START')
    wait(TRIAL_DURATION_S, f"closed-loop ({trial['label']})")
    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')


def main():
    rospy.init_node('experiment_gain_comparison')

    teensy_pub     = rospy.Publisher('/teensy/command', String, queue_size=10)
    closedloop_pub = rospy.Publisher(FLYSTATE_NODE_COMMAND_TOPIC, String, queue_size=10)

    rospy.sleep(1.0)  # allow publishers to connect

    # Blank the display first, before the safety reset -- see the main
    # experiment script for why this ordering avoids a leftover-pattern flash.
    send_command(teensy_pub, 'N0', 'OFF')
    send_command(closedloop_pub, 'stop', 'Closed-loop STOP (safety reset)')
    send_command(teensy_pub, 'S0', 'Zero speed (safety reset)')

    trials = build_trial_list()
    rospy.loginfo(f"Starting gain comparison: {len(trials)} trials (order shuffled).")
    rospy.loginfo("Order: " + ', '.join(t['label'] for t in trials))

    wait(OFF_DURATION, 'initial off period')

    for i, trial in enumerate(trials, start=1):
        run_gain_trial(teensy_pub, closedloop_pub, trial, i, len(trials))

    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')
    send_command(teensy_pub, 'N0', 'OFF')
    rospy.loginfo("Gain comparison complete.")


if __name__ == '__main__':
    main()
