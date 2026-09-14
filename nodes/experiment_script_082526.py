#!/usr/bin/env python3
"""
Randomized stripe/camo trial sequence, each trial followed by closed loop.

Sequence:
    2s   panels OFF (once, at start)
    Then 8 trials in one randomly shuffled order (no repeats):
      - 6 stripe trials: {SLOW, MEDIUM, FAST} x {LEFT, RIGHT}
      - 2 camo trials:   {LEFT, RIGHT} at MEDIUM speed only

    Each STRIPE trial:
        1s  static single stripe
        10s moving stripes (that trial's speed + direction, open loop)
        1s  static single stripe
        10s closed loop (gain = CLOSED_LOOP_GAIN)

    Each CAMO trial:
        1s  static (frozen) camo frame
        10s moving camo (that trial's direction, MEDIUM speed only)
        1s  static (frozen) camo frame
        10s closed loop (gain = CLOSED_LOOP_GAIN)

Requires:
    - flystate2teensy.py started WITHOUT anonymous=True (fixed command topic)
    - neonic_full_patterns.ino with the 'F' static-camo command (pattern 13)
    - teensy_serial.py whitelist including 'F'
"""
import random
import rospy
from std_msgs.msg import String

# ── Experiment parameters ────────────────────────────────────────────────────
OFF_DURATION            = 2.0
STATIC_DURATION         = 1.0
MOVING_DURATION         = 10.0
CLOSED_LOOP_DURATION    = 10.0
CLOSED_LOOP_GAIN        = -180.0
CLOSED_LOOP_PATTERN     = 'Q'    # 'Q' = single stripe (pattern 7), 'P' = stripe grid (pattern 6)
                                  # change this one line to switch what closed loop drives
STATIC_STRIPE_POS       = 48     # px, center (LOGICAL_WIDTH/2 = 96/2, vertical rig panels)

STRIPE_SPEEDS      = ['L', 'M', 'H']       # slow, medium, fast
STRIPE_DIRECTIONS  = ['1', '2']            # left, right
CAMO_SPEED         = 'M'                   # camo trials always use medium speed
CAMO_DIRECTIONS    = ['8', '9']            # left, right

LOOM_POSITIONS       = ['3', '4', '5']     # center, right, left
LOOM_REPEATS         = 2                   # each position presented this many times
LOOM_WAIT_DURATION   = 6.0                 # seconds to let the 5s loom finish (with margin)

FLYSTATE_NODE_COMMAND_TOPIC = '/Flystate2Teensy/command'

SPEED_LABELS = {'L': 'SLOW', 'M': 'MEDIUM', 'H': 'FAST'}
DIR_LABELS   = {'1': 'LEFT', '2': 'RIGHT', '8': 'LEFT', '9': 'RIGHT'}
LOOM_LABELS  = {'3': 'CENTER', '4': 'RIGHT', '5': 'LEFT'}


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
    for speed in STRIPE_SPEEDS:
        for direction in STRIPE_DIRECTIONS:
            trials.append({
                'type': 'stripe',
                'speed_cmd': speed,
                'dir_cmd': direction,
                'label': f'STRIPES {DIR_LABELS[direction]} {SPEED_LABELS[speed]}',
            })
    for direction in CAMO_DIRECTIONS:
        trials.append({
            'type': 'camo',
            'speed_cmd': CAMO_SPEED,
            'dir_cmd': direction,
            'label': f'CAMO {DIR_LABELS[direction]} {SPEED_LABELS[CAMO_SPEED]}',
        })
    random.shuffle(trials)  # single shuffled list, stripe + camo interleaved
    return trials


def run_stripe_trial(teensy_pub, closedloop_pub, trial, idx, total):
    rospy.loginfo(f"--- Trial {idx}/{total}: {trial['label']} ---")

    send_command(teensy_pub, f'P{STATIC_STRIPE_POS:03d}', 'Static stripe grid')
    wait(STATIC_DURATION, 'static stripe (pre)')

    send_command(teensy_pub, trial['speed_cmd'], f"Speed {SPEED_LABELS[trial['speed_cmd']]}")
    send_command(teensy_pub, trial['dir_cmd'], f"Stripes {DIR_LABELS[trial['dir_cmd']]}")
    wait(MOVING_DURATION, f"moving stripes ({trial['label']})")

    send_command(teensy_pub, f'P{STATIC_STRIPE_POS:03d}', 'Static stripe grid')
    wait(STATIC_DURATION, 'static stripe (post)')

    send_command(closedloop_pub, f'gain {CLOSED_LOOP_GAIN}', f'Set gain {CLOSED_LOOP_GAIN}')
    send_command(closedloop_pub, f'pattern {CLOSED_LOOP_PATTERN}', f'Set pattern {CLOSED_LOOP_PATTERN}')
    send_command(closedloop_pub, 'start', 'Closed-loop START')
    wait(CLOSED_LOOP_DURATION, f'closed-loop (gain={CLOSED_LOOP_GAIN})')
    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')


def run_camo_trial(teensy_pub, closedloop_pub, trial, idx, total):
    rospy.loginfo(f"--- Trial {idx}/{total}: {trial['label']} ---")

    send_command(teensy_pub, 'F', 'Static camo')
    wait(STATIC_DURATION, 'static camo (pre)')

    send_command(teensy_pub, trial['speed_cmd'], f"Speed {SPEED_LABELS[trial['speed_cmd']]}")
    send_command(teensy_pub, trial['dir_cmd'], f"Camo {DIR_LABELS[trial['dir_cmd']]}")
    wait(MOVING_DURATION, f"moving camo ({trial['label']})")

    send_command(teensy_pub, 'F', 'Static camo')
    wait(STATIC_DURATION, 'static camo (post)')

    send_command(closedloop_pub, f'gain {CLOSED_LOOP_GAIN}', f'Set gain {CLOSED_LOOP_GAIN}')
    send_command(closedloop_pub, f'pattern {CLOSED_LOOP_PATTERN}', f'Set pattern {CLOSED_LOOP_PATTERN}')
    send_command(closedloop_pub, 'start', 'Closed-loop START')
    wait(CLOSED_LOOP_DURATION, f'closed-loop (gain={CLOSED_LOOP_GAIN})')
    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')


def build_loom_trial_list():
    trials = []
    for pos in LOOM_POSITIONS:
        for _ in range(LOOM_REPEATS):
            trials.append({
                'type': 'loom',
                'pos_cmd': pos,
                'label': f'LOOM {LOOM_LABELS[pos]}',
            })
    random.shuffle(trials)
    return trials


def run_loom_trial(teensy_pub, closedloop_pub, trial, idx, total):
    rospy.loginfo(f"--- Loom trial {idx}/{total}: {trial['label']} ---")

    send_command(teensy_pub, trial['pos_cmd'], trial['label'])
    wait(LOOM_WAIT_DURATION, f"loom playing ({trial['label']})")

    send_command(closedloop_pub, f'gain {CLOSED_LOOP_GAIN}', f'Set gain {CLOSED_LOOP_GAIN}')
    send_command(closedloop_pub, f'pattern {CLOSED_LOOP_PATTERN}', f'Set pattern {CLOSED_LOOP_PATTERN}')
    send_command(closedloop_pub, 'start', 'Closed-loop START')
    wait(CLOSED_LOOP_DURATION, f'closed-loop (gain={CLOSED_LOOP_GAIN})')
    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')


def main():
    rospy.init_node('experiment_stripes_camo_random')

    teensy_pub     = rospy.Publisher('/teensy/command', String, queue_size=10)
    closedloop_pub = rospy.Publisher(FLYSTATE_NODE_COMMAND_TOPIC, String, queue_size=10)

    rospy.sleep(1.0)  # allow publishers to connect

    # Safety reset: flystate2teensy.py is a long-running node that persists
    # across experiment runs. If a previous run was interrupted mid-closed-loop
    # (Ctrl+C, crash, etc.), it can be left silently running (self.bRunning=True)
    # and will keep publishing Q###/P### forever, fighting every command this
    # run sends. Force it to a known-stopped state before doing anything else.
    send_command(closedloop_pub, 'stop', 'Closed-loop STOP (safety reset)')
    wait(0.5, 'safety reset settle')

    trials = build_trial_list()
    rospy.loginfo(f"Starting randomized experiment: {len(trials)} trials (order shuffled).")
    rospy.loginfo("Order: " + ', '.join(t['label'] for t in trials))

    send_command(teensy_pub, '0', 'OFF')
    wait(OFF_DURATION, 'initial off period')

    for i, trial in enumerate(trials, start=1):
        if trial['type'] == 'stripe':
            run_stripe_trial(teensy_pub, closedloop_pub, trial, i, len(trials))
        else:
            run_camo_trial(teensy_pub, closedloop_pub, trial, i, len(trials))

    # ── Loom section ──────────────────────────────────────────────────────────
    loom_trials = build_loom_trial_list()
    rospy.loginfo(f"Starting loom section: {len(loom_trials)} trials (order shuffled).")
    rospy.loginfo("Order: " + ', '.join(t['label'] for t in loom_trials))

    for i, trial in enumerate(loom_trials, start=1):
        run_loom_trial(teensy_pub, closedloop_pub, trial, i, len(loom_trials))

    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')
    send_command(teensy_pub, '0', 'OFF')
    rospy.loginfo("Experiment complete.")


if __name__ == '__main__':
    main()

##CRICULAR ARENA CHANGED THIS SCRIPT + FLYSTATE2TEENSY

##!/usr/bin/env python3
#"""
#Randomized stripe/camo trial sequence, each trial followed by closed loop.

#Sequence:
#    2s   panels OFF (once, at start)
#    Then 8 trials in one randomly shuffled order (no repeats):
#      - 6 stripe trials: {SLOW, MEDIUM, FAST} x {LEFT, RIGHT}
#      - 2 camo trials:   {LEFT, RIGHT} at MEDIUM speed only

#    Each STRIPE trial:
#        1s  static single stripe
#        10s moving stripes (that trial's speed + direction, open loop)
#        1s  static single stripe
#        10s closed loop (gain = CLOSED_LOOP_GAIN)

#    Each CAMO trial:
#        1s  static (frozen) camo frame
#        10s moving camo (that trial's direction, MEDIUM speed only)
#        1s  static (frozen) camo frame
#        10s closed loop (gain = CLOSED_LOOP_GAIN)

#Requires:
#    - flystate2teensy.py started WITHOUT anonymous=True (fixed command topic)
#    - neonic_full_patterns.ino with the 'F' static-camo command (pattern 13)
#    - teensy_serial.py whitelist including 'F'
#"""
#import random
#import rospy
#from std_msgs.msg import String

## ── Experiment parameters ────────────────────────────────────────────────────
#OFF_DURATION            = 2.0
#STATIC_DURATION         = 1.0
#MOVING_DURATION         = 10.0
#CLOSED_LOOP_DURATION    = 10.0
#CLOSED_LOOP_GAIN        = -180.0
#CLOSED_LOOP_PATTERN     = 'Q'    # 'Q' = single stripe (pattern 7), 'P' = stripe grid (pattern 6)
#                                  # change this one line to switch what closed loop drives
#STATIC_STRIPE_POS       = 96     # px, center

#STRIPE_SPEEDS      = ['L', 'M', 'H']       # slow, medium, fast
#STRIPE_DIRECTIONS  = ['1', '2']            # left, right
#CAMO_SPEED         = 'M'                   # camo trials always use medium speed
#CAMO_DIRECTIONS    = ['8', '9']            # left, right

#LOOM_POSITIONS       = ['3', '4', '5']     # center, right, left
#LOOM_REPEATS         = 2                   # each position presented this many times
#LOOM_WAIT_DURATION   = 6.0                 # seconds to let the 5s loom finish (with margin)

#FLYSTATE_NODE_COMMAND_TOPIC = '/Flystate2Teensy/command'

#SPEED_LABELS = {'L': 'SLOW', 'M': 'MEDIUM', 'H': 'FAST'}
#DIR_LABELS   = {'1': 'LEFT', '2': 'RIGHT', '8': 'LEFT', '9': 'RIGHT'}
#LOOM_LABELS  = {'3': 'CENTER', '4': 'RIGHT', '5': 'LEFT'}


#def send_command(pub, cmd, label):
#    msg = String()
#    msg.data = cmd
#    rospy.loginfo(f"Sending: {label} (cmd={cmd})")
#    pub.publish(msg)


#def wait(seconds, label):
#    rospy.loginfo(f"Waiting {seconds}s: {label}")
#    rospy.sleep(seconds)


#def build_trial_list():
#    trials = []
#    for speed in STRIPE_SPEEDS:
#        for direction in STRIPE_DIRECTIONS:
#            trials.append({
#                'type': 'stripe',
#                'speed_cmd': speed,
#                'dir_cmd': direction,
#                'label': f'STRIPES {DIR_LABELS[direction]} {SPEED_LABELS[speed]}',
#            })
#    for direction in CAMO_DIRECTIONS:
#        trials.append({
#            'type': 'camo',
#            'speed_cmd': CAMO_SPEED,
#            'dir_cmd': direction,
#            'label': f'CAMO {DIR_LABELS[direction]} {SPEED_LABELS[CAMO_SPEED]}',
#        })
#    random.shuffle(trials)  # single shuffled list, stripe + camo interleaved
#    return trials


#def run_stripe_trial(teensy_pub, closedloop_pub, trial, idx, total):
#    rospy.loginfo(f"--- Trial {idx}/{total}: {trial['label']} ---")

#    send_command(teensy_pub, f'P{STATIC_STRIPE_POS:03d}', 'Static stripe grid')
#    wait(STATIC_DURATION, 'static stripe (pre)')

#    send_command(teensy_pub, trial['speed_cmd'], f"Speed {SPEED_LABELS[trial['speed_cmd']]}")
#    send_command(teensy_pub, trial['dir_cmd'], f"Stripes {DIR_LABELS[trial['dir_cmd']]}")
#    wait(MOVING_DURATION, f"moving stripes ({trial['label']})")

#    send_command(teensy_pub, f'P{STATIC_STRIPE_POS:03d}', 'Static stripe grid')
#    wait(STATIC_DURATION, 'static stripe (post)')

#    send_command(closedloop_pub, f'gain {CLOSED_LOOP_GAIN}', f'Set gain {CLOSED_LOOP_GAIN}')
#    send_command(closedloop_pub, f'pattern {CLOSED_LOOP_PATTERN}', f'Set pattern {CLOSED_LOOP_PATTERN}')
#    send_command(closedloop_pub, 'start', 'Closed-loop START')
#    wait(CLOSED_LOOP_DURATION, f'closed-loop (gain={CLOSED_LOOP_GAIN})')
#    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')


#def run_camo_trial(teensy_pub, closedloop_pub, trial, idx, total):
#    rospy.loginfo(f"--- Trial {idx}/{total}: {trial['label']} ---")

#    send_command(teensy_pub, 'F', 'Static camo')
#    wait(STATIC_DURATION, 'static camo (pre)')

#    send_command(teensy_pub, trial['speed_cmd'], f"Speed {SPEED_LABELS[trial['speed_cmd']]}")
#    send_command(teensy_pub, trial['dir_cmd'], f"Camo {DIR_LABELS[trial['dir_cmd']]}")
#    wait(MOVING_DURATION, f"moving camo ({trial['label']})")

#    send_command(teensy_pub, 'F', 'Static camo')
#    wait(STATIC_DURATION, 'static camo (post)')

#    send_command(closedloop_pub, f'gain {CLOSED_LOOP_GAIN}', f'Set gain {CLOSED_LOOP_GAIN}')
#    send_command(closedloop_pub, f'pattern {CLOSED_LOOP_PATTERN}', f'Set pattern {CLOSED_LOOP_PATTERN}')
#    send_command(closedloop_pub, 'start', 'Closed-loop START')
#    wait(CLOSED_LOOP_DURATION, f'closed-loop (gain={CLOSED_LOOP_GAIN})')
#    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')


#def build_loom_trial_list():
#    trials = []
#    for pos in LOOM_POSITIONS:
#        for _ in range(LOOM_REPEATS):
#            trials.append({
#                'type': 'loom',
#                'pos_cmd': pos,
#                'label': f'LOOM {LOOM_LABELS[pos]}',
#            })
#    random.shuffle(trials)
#    return trials


#def run_loom_trial(teensy_pub, closedloop_pub, trial, idx, total):
#    rospy.loginfo(f"--- Loom trial {idx}/{total}: {trial['label']} ---")

#    send_command(teensy_pub, trial['pos_cmd'], trial['label'])
#    wait(LOOM_WAIT_DURATION, f"loom playing ({trial['label']})")

#    send_command(closedloop_pub, f'gain {CLOSED_LOOP_GAIN}', f'Set gain {CLOSED_LOOP_GAIN}')
#    send_command(closedloop_pub, f'pattern {CLOSED_LOOP_PATTERN}', f'Set pattern {CLOSED_LOOP_PATTERN}')
#    send_command(closedloop_pub, 'start', 'Closed-loop START')
#    wait(CLOSED_LOOP_DURATION, f'closed-loop (gain={CLOSED_LOOP_GAIN})')
#    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')


#def main():
#    rospy.init_node('experiment_stripes_camo_random')

#    teensy_pub     = rospy.Publisher('/teensy/command', String, queue_size=10)
#    closedloop_pub = rospy.Publisher(FLYSTATE_NODE_COMMAND_TOPIC, String, queue_size=10)

#    rospy.sleep(1.0)  # allow publishers to connect

#    # Safety reset: flystate2teensy.py is a long-running node that persists
#    # across experiment runs. If a previous run was interrupted mid-closed-loop
#    # (Ctrl+C, crash, etc.), it can be left silently running (self.bRunning=True)
#    # and will keep publishing Q###/P### forever, fighting every command this
#    # run sends. Force it to a known-stopped state before doing anything else.
#    send_command(closedloop_pub, 'stop', 'Closed-loop STOP (safety reset)')
#    wait(0.5, 'safety reset settle')

#    trials = build_trial_list()
#    rospy.loginfo(f"Starting randomized experiment: {len(trials)} trials (order shuffled).")
#    rospy.loginfo("Order: " + ', '.join(t['label'] for t in trials))

#    send_command(teensy_pub, '0', 'OFF')
#    wait(OFF_DURATION, 'initial off period')

#    for i, trial in enumerate(trials, start=1):
#        if trial['type'] == 'stripe':
#            run_stripe_trial(teensy_pub, closedloop_pub, trial, i, len(trials))
#        else:
#            run_camo_trial(teensy_pub, closedloop_pub, trial, i, len(trials))

#    # ── Loom section ──────────────────────────────────────────────────────────
#    loom_trials = build_loom_trial_list()
#    rospy.loginfo(f"Starting loom section: {len(loom_trials)} trials (order shuffled).")
#    rospy.loginfo("Order: " + ', '.join(t['label'] for t in loom_trials))

#    for i, trial in enumerate(loom_trials, start=1):
#        run_loom_trial(teensy_pub, closedloop_pub, trial, i, len(loom_trials))

#    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')
#    send_command(teensy_pub, '0', 'OFF')
#    rospy.loginfo("Experiment complete.")


#if __name__ == '__main__':
#    main()
