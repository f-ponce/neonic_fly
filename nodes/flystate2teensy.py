#!/usr/bin/env python3
from __future__ import division
import rospy
import numpy as np
from std_msgs.msg import String
from Kinefly.msg import MsgFlystate


# ── Experiment parameters — edit these ───────────────────────────────────────
#
# VELOCITY-MODE VARIANT. This bridge does NOT integrate position itself.
# It computes an instantaneous velocity (deg/s) from the current wing
# angles, exactly like Kinefly's own flystate2phidgetsanalog /
# flystate2ledpanels nodes do, and sends that as "S<velocity>" every
# flystate frame. The Teensy's own onboard integrator
# (updatePatternPosition(), clocked by micros()) is what turns that into
# an actual moving position -- the same division of labor a real LED
# panel controller uses in velocity mode. See patterns_091326_
# geometrycorrected.ino for that integrator.
#
# Consequence: there is no "offset_deg" state here at all, and no drift
# risk from a stale integrator on either side -- there's only ever ONE
# active speed value in the system at a time, whichever S command was
# sent most recently. The one thing THIS design must get right instead:
# always sending S0 on stop, since the Teensy will otherwise keep
# obeying the last speed forever once we stop publishing.
#
# GAIN is deg/s per rad. Original placeholder (-150) was an unjustified
# round-number guess with no real derivation (see chat history).
#
# Updated per Giraldo et al. 2018 (Curr Biol) -- an actual published
# Kinefly closed-loop rig, same lab (Dickinson), same panel hardware
# lineage (IORodeo). They report gain in deg/s per DEGREE of WBA:
#   14.67 (sun orientation), 5.88 (functional imaging), 4.75 (genetic
#   silencing) deg/s per deg-WBA. Converted to deg/s per RADIAN (x180/pi)
#   to match Kinefly's native radian units:
#     14.67 -> ~840   5.88 -> ~337   4.75 -> ~272
# They note lower gain was needed for stable closed loop in some lines,
# so this isn't one canonical number -- it's a real, evidence-backed
# range (~270-840) to tune within, instead of an arbitrary guess.
# Starting at the low/conservative end of that range:
GAIN = -300.0    # deg/rad -- retune empirically from here; see chat for derivation

# Safety clamp -- caps a single bad-but-finite flystate reading (tracking
# spike, brief occlusion) from becoming a standing commanded speed until
# the next good sample arrives. NOT the same as the missing-track (NaN)
# case below, which skips publishing entirely rather than clamping.
#
# 200 deg/s was an earlier unjustified round-number guess. At GAIN=-270,
# that would clamp at WBA ~= 0.74 rad (42 deg) -- plausible for a real
# vigorous turn, so it risked flattening genuine behavior, not just
# catching glitches. Updated to 600 deg/s, referencing Cheng et al./the
# wing-damage compensation literature describing ~9-612 deg/s as "within
# the visual bandwidth limits of the yaw optomotor response of
# Drosophila" -- i.e. a ceiling grounded in what the fly's visual system
# can behaviorally respond to, not an arbitrary margin. At GAIN=-270,
# 600 deg/s corresponds to WBA ~= 2.22 rad (~127 deg), well beyond any
# real two-wing WBA difference -- so it should only trip on genuine
# glitches. Re-check this math if GAIN ends up outside the ~270-840
# range after empirical tuning.
MAX_SPEED_DEG_S = 600.0

# Which pattern closed-loop drives. Must be one of the three patterns that
# share the Teensy's motion state (S/P): 1 = stripe grid, 2 = single stripe,
# 9 = random rotating texture. Change here, override with _pattern:=2 at
# launch, or switch live with the 'pattern <1|2|9>' command.
CLOSED_LOOP_PATTERN = 2

# Where the pattern starts each time closed loop begins, before speed
# commands take over. None of this is required for correctness (the
# pattern will move to wherever the first velocity command takes it
# regardless), but a known starting phase makes trials comparable.
START_POSITION_DEG = 0.0

X0   =    0.0   # deg/s  -- baseline drift (0 = no open-loop component)
XL1  =    1.0   # left  wing angle coefficient  (WBA = L - R)
XR1  =   -1.0   # right wing angle coefficient
XL2  =    0.0   # left  wing minor angle
XLR  =    0.0   # left  wing radius
XR2  =    0.0   # right wing minor angle
XRR  =    0.0   # right wing radius
XHA  =    0.0   # head angle
XHR  =    0.0   # head radius
XAA  =    0.0   # abdomen angle
XAR  =    0.0   # abdomen radius
XXI  =    0.0   # aux intensity
# ─────────────────────────────────────────────────────────────────────────────

VALID_PATTERNS = (1, 2, 9)


###############################################################################
###############################################################################
class Flystate2TeensyVelocityMode:
    """
    Closed-loop bridge: Kinefly flystate -> Teensy LED panel velocity.

    Velocity mode (matches Kinefly's own flystate2ledpanels design):
    every flystate frame, compute velocity_deg_s = dot(a, state) * gain
    and publish it directly as "S<velocity>". No integration happens
    here -- patterns_091326_geometrycorrected.ino's own
    updatePatternPosition() (clocked by micros()) is the sole place
    position gets accumulated, exactly like a real panel controller
    board integrating a velocity-mode ADC input onboard.

    IMPORTANT — missed wing tracks:
    Following Kinefly's own convention (a bodypart's angle list is empty
    when nothing was detected that frame, never a fabricated value), a
    dropped wing track is represented as NaN, not 0.0. If a NaN would
    reach a nonzero coefficient, flystate_callback skips publishing
    entirely for that frame -- the Teensy just keeps obeying its last
    commanded speed until tracking returns, rather than us asserting "the
    fly stopped turning" (S0) when we actually have no idea.

    Topics
    ------
    Subscribes:
        <namespace>/flystate        (Kinefly/MsgFlystate)
        /Flystate2Teensy/command    (std_msgs/String)  start|stop|exit|help
    Publishes:
        /teensy/command             (std_msgs/String)  "N<pattern>" / "P<deg>" / "S<deg/s>"

    NOTE: node name is fixed (anonymous=False) so the command topic
    (/Flystate2Teensy/command) is stable across runs and other nodes/scripts
    can reliably publish 'start'/'stop' to it.
    """

    def __init__(self):
        self.bInitialized = False
        self.bRunning     = False

        self.name = 'Flystate2Teensy'
        rospy.init_node(self.name)  # anonymous=True removed -> fixed topic name
        self.nodename  = rospy.get_name()
        self.namespace = rospy.get_namespace()

        self.gain = rospy.get_param('~gain', GAIN)

        pattern = rospy.get_param('~pattern', CLOSED_LOOP_PATTERN)
        try:
            pattern = int(pattern)
        except (TypeError, ValueError):
            pattern = CLOSED_LOOP_PATTERN
        if pattern not in VALID_PATTERNS:
            rospy.logwarn("%s: invalid ~pattern param %r, defaulting to %r" %
                           (self.name, pattern, CLOSED_LOOP_PATTERN))
            pattern = CLOSED_LOOP_PATTERN
        self.pattern_num = pattern

        # Coefficient vector — edit constants at top of file
        self.a = np.array([X0,
                           XL1, XL2, XLR,
                           XR1, XR2, XRR,
                           XHA, XHR,
                           XAA, XAR,
                           XXI], dtype=np.float32)

        # ── Publishers / Subscribers ──────────────────────────────────────────
        # queue_size=10, not 1: 'start' publishes N<pattern> then P<pos> back-
        # to-back on this same publisher. With queue_size=1, if the first
        # message hasn't flushed before the second is published, ROS drops
        # the older one -- silently losing the pattern-select command while
        # the position command still gets through.
        self.pubTeensy = rospy.Publisher('/teensy/command', String, queue_size=10)

        self.subFlystate = rospy.Subscriber(
            '/kinefly/flystate',
            MsgFlystate, self.flystate_callback, queue_size=1000)

        self.subCommand = rospy.Subscriber(
            '%s/command' % self.nodename.rstrip('/'),
            String, self.command_callback, queue_size=1000)

        rospy.sleep(1)  # allow publishers/subscribers to connect

        self.bInitialized = True
        rospy.loginfo('%s: initialized (velocity mode).  Send "start" to begin.' % self.name)
        rospy.loginfo('%s: command topic is %s/command' % (self.name, self.nodename.rstrip('/')))
        rospy.loginfo('%s: initial gain = %.2f deg/rad' % (self.name, self.gain))
        rospy.loginfo('%s: closed-loop pattern = %d' % (self.name, self.pattern_num))


    # ── State vector ──────────────────────────────────────────────────────────

    def state_from_flystate(self, flystate):
        """Same convention as Kinefly itself: an empty angle/radius list
        means "no detection this frame", represented as NaN rather than a
        fabricated 0.0. See weighted_sum_or_nan() for how this propagates."""
        l1 = flystate.left.angles[0]    if len(flystate.left.angles)    > 0 else np.nan
        l2 = flystate.left.angles[1]    if len(flystate.left.angles)    > 1 else np.nan
        lr = flystate.left.radii[0]     if len(flystate.left.radii)     > 0 else np.nan
        r1 = flystate.right.angles[0]   if len(flystate.right.angles)   > 0 else np.nan
        r2 = flystate.right.angles[1]   if len(flystate.right.angles)   > 1 else np.nan
        rr = flystate.right.radii[0]    if len(flystate.right.radii)    > 0 else np.nan
        ha = flystate.head.angles[0]    if len(flystate.head.angles)    > 0 else np.nan
        hr = flystate.head.radii[0]     if len(flystate.head.radii)     > 0 else np.nan
        aa = flystate.abdomen.angles[0] if len(flystate.abdomen.angles) > 0 else np.nan
        ar = flystate.abdomen.radii[0]  if len(flystate.abdomen.radii)  > 0 else np.nan
        xi = flystate.aux.intensity

        return np.array([1.0, l1, l2, lr, r1, r2, rr, ha, hr, aa, ar, xi],
                        dtype=np.float32)

    def weighted_sum_or_nan(self, state):
        """dot(a, state), but a NaN only "counts" for terms whose
        coefficient is actually nonzero -- e.g. a missing head angle
        shouldn't block a velocity computation that never uses it. Returns
        NaN if any USED term is missing."""
        contrib = np.where(self.a == 0.0, 0.0, self.a * state)
        return float(np.sum(contrib))


    # ── Main callback ─────────────────────────────────────────────────────────

    def flystate_callback(self, flystate):
        if not self.bRunning:
            return

        state    = self.state_from_flystate(flystate)
        velocity = self.weighted_sum_or_nan(state) * self.gain  # deg/s

        if not np.isfinite(velocity):
            # A field our coefficients actually use is missing this frame
            # (dropped wing track, etc). Following Kinefly's own
            # convention -- report nothing rather than fabricate a value --
            # we publish nothing here either. The Teensy just keeps
            # obeying whatever speed it was last told, which is a more
            # honest "we don't know" than snapping to S0 (S0 asserts "the
            # fly stopped turning", which isn't something we actually know
            # from a missing frame).
            rospy.logwarn_throttle(5, '%s: missing tracked field this frame, holding last speed' % self.name)
            return

        # Safety clamp -- see MAX_SPEED_DEG_S comment above. This guards
        # against a genuinely bad (but present) reading, e.g. a brief
        # tracking spike, as opposed to the missing-data case above.
        velocity = max(-MAX_SPEED_DEG_S, min(MAX_SPEED_DEG_S, velocity))

        self.pubTeensy.publish(String(data='S%.2f' % velocity))


    # ── Command callback ──────────────────────────────────────────────────────

    def command_callback(self, command):
        cmd = command.data.strip()

        if cmd == 'start':
            self.pubTeensy.publish(String(data='N%d' % self.pattern_num))
            self.pubTeensy.publish(String(data='P%.2f' % START_POSITION_DEG))
            self.bRunning = True
            rospy.loginfo('%s: started (pattern %d, velocity mode).' % (self.name, self.pattern_num))

        elif cmd == 'stop':
            self.bRunning = False
            # Must explicitly zero speed here -- unlike the position-based
            # design, simply stopping our own publishing does NOT stop the
            # Teensy. It will keep obeying the last S value it was sent
            # indefinitely, since its integrator has no way to know we've
            # stopped talking to it.
            self.pubTeensy.publish(String(data='S0'))
            rospy.loginfo('%s: stopped (speed zeroed).' % self.name)

        elif cmd == 'exit':
            self.bRunning = False
            self.pubTeensy.publish(String(data='S0'))
            self.pubTeensy.publish(String(data='0'))
            rospy.signal_shutdown('User requested exit.')

        elif cmd.lower().startswith('gain'):
            value_str = cmd[4:].strip()
            try:
                new_gain = float(value_str)
                self.gain = new_gain
                rospy.loginfo('%s: gain set to %.2f deg/rad' % (self.name, self.gain))
            except ValueError:
                rospy.logwarn('%s: invalid gain command: %r (expected e.g. "gain 270")' % (self.name, cmd))

        elif cmd.lower().startswith('pattern'):
            value_str = cmd[7:].strip()
            try:
                new_pattern = int(value_str)
            except ValueError:
                new_pattern = None
            if new_pattern in VALID_PATTERNS:
                self.pattern_num = new_pattern
                rospy.loginfo('%s: closed-loop pattern set to %d' % (self.name, new_pattern))
                if self.bRunning:
                    # Zero speed across the switch -- the OLD pattern's
                    # last commanded speed would otherwise carry straight
                    # over onto the newly selected pattern, since they
                    # share the same motion state on the Teensy.
                    self.pubTeensy.publish(String(data='S0'))
                    self.pubTeensy.publish(String(data='N%d' % self.pattern_num))
            else:
                rospy.logwarn('%s: invalid pattern command: %r (expected "pattern 1", "pattern 2", or "pattern 9")' % (self.name, cmd))

        elif cmd == 'help':
            rospy.logwarn('Commands on %s/command:' % self.nodename.rstrip('/'))
            rospy.logwarn('  start        Begin closed-loop velocity mode (selects pattern, sets start position)')
            rospy.logwarn('  stop         Stop and zero speed (S0)')
            rospy.logwarn('  gain <value> Set gain (deg/rad) live, e.g. "gain 270"')
            rospy.logwarn('  pattern <1|2|9> Set closed-loop pattern live, e.g. "pattern 1"')
            rospy.logwarn('  exit         Shutdown node')
            rospy.logwarn('  help         This message')
            rospy.logwarn('Other coefficients (XL1, XR1, etc.) are edited at top of script.')


    def run(self):
        rospy.spin()
        # Always leave the panel stopped on shutdown -- same reasoning as
        # the 'stop' command above: nothing else will zero the speed for us.
        self.pubTeensy.publish(String(data='S0'))
        self.pubTeensy.publish(String(data='0'))


###############################################################################
if __name__ == '__main__':
    node = Flystate2TeensyVelocityMode()
    node.run()
