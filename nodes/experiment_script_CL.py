#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

# ── Experiment timing ─────────────────────────────────────────────────────────
CLOSED_LOOP_DURATION = 200.0   # seconds of closed-loop per cycle
STATIC_DURATION       = 1.0   # seconds of static single-stripe per cycle
NUM_CYCLES            = 1     # edit to change total experiment length
                               # total = NUM_CYCLES * (CLOSED_LOOP_DURATION + STATIC_DURATION)

STATIC_STRIPE_POS = 96        # px, fixed position for the static single stripe (center)

# Gain (px/rad) for closed-loop, one value per cycle. Must have NUM_CYCLES
# entries -- edit this list to vary gain trial to trial. All-same-value is
# fine too, e.g. [200.0] * NUM_CYCLES.
GAINS_PER_CYCLE = [-180.0]

# Requires flystate2teensy.py to be started WITHOUT anonymous=True, so its
# command topic is fixed at /Flystate2Teensy/command.
FLYSTATE_NODE_COMMAND_TOPIC = '/Flystate2Teensy/command'


def send_command(pub, cmd, label):
    msg = String()
    msg.data = cmd
    rospy.loginfo(f"Sending: {label} (cmd={cmd})")
    pub.publish(msg)


def wait(seconds, label):
    rospy.loginfo(f"Waiting {seconds}s: {label}")
    rospy.sleep(seconds)


def main():
    rospy.init_node('experiment_closedloop_static')

    teensy_pub    = rospy.Publisher('/teensy/command', String, queue_size=10)
    closedloop_pub = rospy.Publisher(FLYSTATE_NODE_COMMAND_TOPIC, String, queue_size=10)

    rospy.sleep(1.0)  # allow publishers to connect

    if len(GAINS_PER_CYCLE) != NUM_CYCLES:
        rospy.logerr(
            f"GAINS_PER_CYCLE has {len(GAINS_PER_CYCLE)} entries but "
            f"NUM_CYCLES={NUM_CYCLES}. Fix the list before running."
        )
        return

    rospy.loginfo(
        f"Starting alternating closed-loop/static experiment: "
        f"{NUM_CYCLES} cycles of {CLOSED_LOOP_DURATION}s closed-loop + "
        f"{STATIC_DURATION}s static stripe."
    )

    for i in range(1, NUM_CYCLES + 1):
        gain = GAINS_PER_CYCLE[i - 1]
        rospy.loginfo(f"--- Cycle {i}/{NUM_CYCLES} (gain={gain}) ---")

        # Set gain for this cycle, then start closed loop
        send_command(closedloop_pub, f'gain {gain}', f'Set gain {gain}')
        send_command(closedloop_pub, 'start', 'Closed-loop START')
        wait(CLOSED_LOOP_DURATION, f'closed-loop ({CLOSED_LOOP_DURATION}s)')

        # Stop closed loop before switching to static stimulus, otherwise
        # flystate2teensy keeps publishing P### and will fight the Q### below.
        send_command(closedloop_pub, 'stop', 'Closed-loop STOP')

        # Static single stripe, fixed position -- Teensy holds this pattern
        # until the next command since nothing keeps updating stripeOffset.
        send_command(teensy_pub, f'Q{STATIC_STRIPE_POS:03d}', 'Static single stripe')
        wait(STATIC_DURATION, f'static stripe ({STATIC_DURATION}s)')

    # Final cleanup
    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')
    send_command(teensy_pub, '0', 'OFF')
    rospy.loginfo("Experiment complete.")


if __name__ == '__main__':
    main()
