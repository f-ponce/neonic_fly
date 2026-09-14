#!/usr/bin/env python3
from __future__ import division
import rospy
import numpy as np
from std_msgs.msg import String
from Kinefly.msg import MsgFlystate


# ── Experiment parameters — edit these ───────────────────────────────────────
#
# VERTICAL RIG: 3 portrait-mounted HUB75 panels = 96px wide (logical), 64px tall
# (see neonic_full_patterns_vertical.ino for the native<->logical rotation).
# Visual angle coverage: ~90° around the fly (measured geometry, see notes)
# px/degree: 96 / 90 = 1.067
#
# Target: pattern moves at ~30°/s for a moderate WBA of 0.2 rad
#   -> 30 deg/s x 1.067 px/deg = 32 px/s at WBA=0.2 rad
#   -> gain = 32 / 0.2 = 160
# GAIN below is a starting point carried over from the old rig -- retune
# on real hardware, same as before.
#
DISPLAY_WIDTH  = 96     # px  -- must match LOGICAL_WIDTH in the vertical .ino
DISPLAY_HEIGHT = 64     # px  -- informational only

GAIN =  -200.0   # px/rad -- overall velocity scale; tune this first
CLOSED_LOOP_PATTERN = 'Q'   # 'Q' = single stripe (pattern 7), 'P' = stripe grid (pattern 6)
                            # -- change here, or override with _pattern:=P at launch,
                            # or live with the 'pattern <P|Q>' command.
X0   =    0.0   # px/s   -- baseline drift (0 = no open-loop component)
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


###############################################################################
###############################################################################
class Flystate2Teensy:
    """
    Closed-loop bridge: Kinefly flystate -> Teensy LED panel stripe velocity.

    Velocity mode: stripe_offset += dot(a, state) * gain * dt
    Offset wraps into [0, DISPLAY_WIDTH) and is sent as "Q<NNN>" (single
    stripe, pattern 7) so closed-loop and the static breaks both use the
    same single-stripe stimulus.

    Topics
    ------
    Subscribes:
        <namespace>/flystate        (Kinefly/MsgFlystate)
        /Flystate2Teensy/command    (std_msgs/String)  start|stop|exit|help
    Publishes:
        /teensy/command             (std_msgs/String)  "Q<NNN>"

    NOTE: node name is fixed (anonymous=False) so the command topic
    (/Flystate2Teensy/command) is stable across runs and other nodes/scripts
    can reliably publish 'start'/'stop' to it.
    """

    def __init__(self):
        self.bInitialized = False
        self.bRunning     = True

        self.name = 'Flystate2Teensy'
        rospy.init_node(self.name)  # anonymous=True removed -> fixed topic name
        self.nodename  = rospy.get_name()
        self.namespace = rospy.get_namespace()

        # Gain can be set at launch (e.g. _gain:=150) or changed live per
        # trial via the command topic (see command_callback -> 'gain <value>').
        self.gain = rospy.get_param('~gain', GAIN)

        # Which pattern closed-loop drives: 'Q' (single stripe) or 'P' (stripe grid).
        pattern = rospy.get_param('~pattern', CLOSED_LOOP_PATTERN)
        self.pattern_prefix = pattern if pattern in ('P', 'Q') else CLOSED_LOOP_PATTERN
        if pattern not in ('P', 'Q'):
            rospy.logwarn("%s: invalid ~pattern param %r, defaulting to %r" %
                           (self.name, pattern, CLOSED_LOOP_PATTERN))

        # Coefficient vector — edit constants at top of file
        self.a = np.array([X0,
                           XL1, XL2, XLR,
                           XR1, XR2, XRR,
                           XHA, XHR,
                           XAA, XAR,
                           XXI], dtype=np.float32)

        # Integrator state
        self.stripe_offset = 0.0
        self.last_time     = None

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.pubTeensy = rospy.Publisher('/teensy/command', String, queue_size=1)

        self.subFlystate = rospy.Subscriber(
            '/kinefly/flystate',
            MsgFlystate, self.flystate_callback, queue_size=1000)

        self.subCommand = rospy.Subscriber(
            '%s/command' % self.nodename.rstrip('/'),
            String, self.command_callback, queue_size=1000)

        rospy.sleep(1)  # allow publishers/subscribers to connect

        self.bInitialized = True
        rospy.loginfo('%s: initialized.  Send "start" to begin.' % self.name)
        rospy.loginfo('%s: command topic is %s/command' % (self.name, self.nodename.rstrip('/')))
        rospy.loginfo('%s: initial gain = %.2f px/rad' % (self.name, self.gain))
        rospy.loginfo('%s: closed-loop pattern = %r' % (self.name, self.pattern_prefix))


    # ── State vector ──────────────────────────────────────────────────────────

    def state_from_flystate(self, flystate):
        l1 = flystate.left.angles[0]    if len(flystate.left.angles)    > 0 else 0.0
        l2 = flystate.left.angles[1]    if len(flystate.left.angles)    > 1 else 0.0
        lr = flystate.left.radii[0]     if len(flystate.left.radii)     > 0 else 0.0
        r1 = flystate.right.angles[0]   if len(flystate.right.angles)   > 0 else 0.0
        r2 = flystate.right.angles[1]   if len(flystate.right.angles)   > 1 else 0.0
        rr = flystate.right.radii[0]    if len(flystate.right.radii)    > 0 else 0.0
        ha = flystate.head.angles[0]    if len(flystate.head.angles)    > 0 else 0.0
        hr = flystate.head.radii[0]     if len(flystate.head.radii)     > 0 else 0.0
        aa = flystate.abdomen.angles[0] if len(flystate.abdomen.angles) > 0 else 0.0
        ar = flystate.abdomen.radii[0]  if len(flystate.abdomen.radii)  > 0 else 0.0
        xi = flystate.aux.intensity

        return np.array([1.0, l1, l2, lr, r1, r2, rr, ha, hr, aa, ar, xi],
                        dtype=np.float32)


    # ── Main callback ─────────────────────────────────────────────────────────

    def flystate_callback(self, flystate):
        if not self.bRunning:
            return

        now = rospy.Time.now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        state    = self.state_from_flystate(flystate)
        velocity = float(np.dot(self.a, state)) * self.gain  # px/s

        # Integrate and wrap into [0, DISPLAY_WIDTH)
        self.stripe_offset = (self.stripe_offset + velocity * dt) % DISPLAY_WIDTH
        if self.stripe_offset < 0:
            self.stripe_offset += DISPLAY_WIDTH

        offset_int = int(round(self.stripe_offset)) % DISPLAY_WIDTH
        self.pubTeensy.publish(String(data='%s%03d' % (self.pattern_prefix, offset_int)))


    # ── Command callback ──────────────────────────────────────────────────────

    def command_callback(self, command):
        cmd = command.data.strip()

        if cmd == 'start':
            self.stripe_offset = 0.0
            self.last_time     = None
            self.bRunning      = True
            rospy.loginfo('%s: started.' % self.name)

        elif cmd == 'stop':
            self.bRunning = False
            rospy.loginfo('%s: stopped.' % self.name)

        elif cmd == 'exit':
            self.bRunning = False
            self.pubTeensy.publish(String(data='0'))
            rospy.signal_shutdown('User requested exit.')

        elif cmd.lower().startswith('gain'):
            # e.g. 'gain 150' or 'gain150' -> set live gain (px/rad) for
            # subsequent trials without restarting the node.
            value_str = cmd[4:].strip()
            try:
                new_gain = float(value_str)
                self.gain = new_gain
                rospy.loginfo('%s: gain set to %.2f px/rad' % (self.name, self.gain))
            except ValueError:
                rospy.logwarn('%s: invalid gain command: %r (expected e.g. "gain 150")' % (self.name, cmd))

        elif cmd.lower().startswith('pattern'):
            # e.g. 'pattern P' or 'pattern Q' -- switch which closed-loop
            # pattern is driven, live, without restarting the node.
            value = cmd[7:].strip().upper()
            if value in ('P', 'Q'):
                self.pattern_prefix = value
                rospy.loginfo('%s: closed-loop pattern set to %r' % (self.name, value))
            else:
                rospy.logwarn('%s: invalid pattern command: %r (expected "pattern P" or "pattern Q")' % (self.name, cmd))

        elif cmd == 'help':
            rospy.logwarn('Commands on %s/command:' % self.nodename.rstrip('/'))
            rospy.logwarn('  start        Begin closed-loop (resets integrator)')
            rospy.logwarn('  stop         Stop and blank display')
            rospy.logwarn('  gain <value> Set gain (px/rad) live, e.g. "gain 150"')
            rospy.logwarn('  pattern <P|Q> Set closed-loop pattern live, e.g. "pattern P"')
            rospy.logwarn('  exit         Shutdown node')
            rospy.logwarn('  help         This message')
            rospy.logwarn('Other coefficients (XL1, XR1, etc.) are edited at top of script.')


    def run(self):
        rospy.spin()
        self.pubTeensy.publish(String(data='0'))


###############################################################################
if __name__ == '__main__':
    node = Flystate2Teensy()
    node.run()











# CIRCULAR PANELS WORKING

##!/usr/bin/env python3
#from __future__ import division
#import rospy
#import numpy as np
#from std_msgs.msg import String
#from Kinefly.msg import MsgFlystate


## ── Experiment parameters — edit these ───────────────────────────────────────
##
## Arena: 3 × 64px HUB75 panels = 192px wide, 32px tall
## Visual angle coverage: ~270° around the fly
## px/degree: 192 / 270 = 0.71
##
## Target: pattern moves at ~30°/s for a moderate WBA of 0.2 rad
##   -> 30 deg/s x 0.71 px/deg = 21 px/s at WBA=0.2 rad
##   -> gain = 21 / 0.2 = 105 -> rounded to 100
##
#DISPLAY_WIDTH  = 192    # px  -- must match kMatrixWidth in .ino
#DISPLAY_HEIGHT = 32     # px  -- informational only

#GAIN =  200.0   # px/rad -- overall velocity scale; tune this first
#CLOSED_LOOP_PATTERN = 'Q'   # 'Q' = single stripe (pattern 7), 'P' = stripe grid (pattern 6)
#                            # -- change here, or override with _pattern:=P at launch,
#                            # or live with the 'pattern <P|Q>' command.
#X0   =    0.0   # px/s   -- baseline drift (0 = no open-loop component)
#XL1  =    1.0   # left  wing angle coefficient  (WBA = L - R)
#XR1  =   -1.0   # right wing angle coefficient
#XL2  =    0.0   # left  wing minor angle
#XLR  =    0.0   # left  wing radius
#XR2  =    0.0   # right wing minor angle
#XRR  =    0.0   # right wing radius
#XHA  =    0.0   # head angle
#XHR  =    0.0   # head radius
#XAA  =    0.0   # abdomen angle
#XAR  =    0.0   # abdomen radius
#XXI  =    0.0   # aux intensity
## ─────────────────────────────────────────────────────────────────────────────


################################################################################
################################################################################
#class Flystate2Teensy:
#    """
#    Closed-loop bridge: Kinefly flystate -> Teensy LED panel stripe velocity.

#    Velocity mode: stripe_offset += dot(a, state) * gain * dt
#    Offset wraps into [0, DISPLAY_WIDTH) and is sent as "Q<NNN>" (single
#    stripe, pattern 7) so closed-loop and the static breaks both use the
#    same single-stripe stimulus.

#    Topics
#    ------
#    Subscribes:
#        <namespace>/flystate        (Kinefly/MsgFlystate)
#        /Flystate2Teensy/command    (std_msgs/String)  start|stop|exit|help
#    Publishes:
#        /teensy/command             (std_msgs/String)  "Q<NNN>"

#    NOTE: node name is fixed (anonymous=False) so the command topic
#    (/Flystate2Teensy/command) is stable across runs and other nodes/scripts
#    can reliably publish 'start'/'stop' to it.
#    """

#    def __init__(self):
#        self.bInitialized = False
#        self.bRunning     = True

#        self.name = 'Flystate2Teensy'
#        rospy.init_node(self.name)  # anonymous=True removed -> fixed topic name
#        self.nodename  = rospy.get_name()
#        self.namespace = rospy.get_namespace()

#        # Gain can be set at launch (e.g. _gain:=150) or changed live per
#        # trial via the command topic (see command_callback -> 'gain <value>').
#        self.gain = rospy.get_param('~gain', GAIN)

#        # Which pattern closed-loop drives: 'Q' (single stripe) or 'P' (stripe grid).
#        pattern = rospy.get_param('~pattern', CLOSED_LOOP_PATTERN)
#        self.pattern_prefix = pattern if pattern in ('P', 'Q') else CLOSED_LOOP_PATTERN
#        if pattern not in ('P', 'Q'):
#            rospy.logwarn("%s: invalid ~pattern param %r, defaulting to %r" %
#                           (self.name, pattern, CLOSED_LOOP_PATTERN))

#        # Coefficient vector — edit constants at top of file
#        self.a = np.array([X0,
#                           XL1, XL2, XLR,
#                           XR1, XR2, XRR,
#                           XHA, XHR,
#                           XAA, XAR,
#                           XXI], dtype=np.float32)

#        # Integrator state
#        self.stripe_offset = 0.0
#        self.last_time     = None

#        # ── Publishers / Subscribers ──────────────────────────────────────────
#        self.pubTeensy = rospy.Publisher('/teensy/command', String, queue_size=1)

#        self.subFlystate = rospy.Subscriber(
#            '/kinefly/flystate',
#            MsgFlystate, self.flystate_callback, queue_size=1000)

#        self.subCommand = rospy.Subscriber(
#            '%s/command' % self.nodename.rstrip('/'),
#            String, self.command_callback, queue_size=1000)

#        rospy.sleep(1)  # allow publishers/subscribers to connect

#        self.bInitialized = True
#        rospy.loginfo('%s: initialized.  Send "start" to begin.' % self.name)
#        rospy.loginfo('%s: command topic is %s/command' % (self.name, self.nodename.rstrip('/')))
#        rospy.loginfo('%s: initial gain = %.2f px/rad' % (self.name, self.gain))
#        rospy.loginfo('%s: closed-loop pattern = %r' % (self.name, self.pattern_prefix))


#    # ── State vector ──────────────────────────────────────────────────────────

#    def state_from_flystate(self, flystate):
#        l1 = flystate.left.angles[0]    if len(flystate.left.angles)    > 0 else 0.0
#        l2 = flystate.left.angles[1]    if len(flystate.left.angles)    > 1 else 0.0
#        lr = flystate.left.radii[0]     if len(flystate.left.radii)     > 0 else 0.0
#        r1 = flystate.right.angles[0]   if len(flystate.right.angles)   > 0 else 0.0
#        r2 = flystate.right.angles[1]   if len(flystate.right.angles)   > 1 else 0.0
#        rr = flystate.right.radii[0]    if len(flystate.right.radii)    > 0 else 0.0
#        ha = flystate.head.angles[0]    if len(flystate.head.angles)    > 0 else 0.0
#        hr = flystate.head.radii[0]     if len(flystate.head.radii)     > 0 else 0.0
#        aa = flystate.abdomen.angles[0] if len(flystate.abdomen.angles) > 0 else 0.0
#        ar = flystate.abdomen.radii[0]  if len(flystate.abdomen.radii)  > 0 else 0.0
#        xi = flystate.aux.intensity

#        return np.array([1.0, l1, l2, lr, r1, r2, rr, ha, hr, aa, ar, xi],
#                        dtype=np.float32)


#    # ── Main callback ─────────────────────────────────────────────────────────

#    def flystate_callback(self, flystate):
#        if not self.bRunning:
#            return

#        now = rospy.Time.now()
#        if self.last_time is None:
#            self.last_time = now
#            return
#        dt = (now - self.last_time).to_sec()
#        self.last_time = now

#        if dt <= 0.0 or dt > 0.5:
#            return

#        state    = self.state_from_flystate(flystate)
#        velocity = float(np.dot(self.a, state)) * self.gain  # px/s

#        # Integrate and wrap into [0, DISPLAY_WIDTH)
#        self.stripe_offset = (self.stripe_offset + velocity * dt) % DISPLAY_WIDTH
#        if self.stripe_offset < 0:
#            self.stripe_offset += DISPLAY_WIDTH

#        offset_int = int(round(self.stripe_offset)) % DISPLAY_WIDTH
#        self.pubTeensy.publish(String(data='%s%03d' % (self.pattern_prefix, offset_int)))


#    # ── Command callback ──────────────────────────────────────────────────────

#    def command_callback(self, command):
#        cmd = command.data.strip()

#        if cmd == 'start':
#            self.stripe_offset = 0.0
#            self.last_time     = None
#            self.bRunning      = True
#            rospy.loginfo('%s: started.' % self.name)

#        elif cmd == 'stop':
#            self.bRunning = False
#            rospy.loginfo('%s: stopped.' % self.name)

#        elif cmd == 'exit':
#            self.bRunning = False
#            self.pubTeensy.publish(String(data='0'))
#            rospy.signal_shutdown('User requested exit.')

#        elif cmd.lower().startswith('gain'):
#            # e.g. 'gain 150' or 'gain150' -> set live gain (px/rad) for
#            # subsequent trials without restarting the node.
#            value_str = cmd[4:].strip()
#            try:
#                new_gain = float(value_str)
#                self.gain = new_gain
#                rospy.loginfo('%s: gain set to %.2f px/rad' % (self.name, self.gain))
#            except ValueError:
#                rospy.logwarn('%s: invalid gain command: %r (expected e.g. "gain 150")' % (self.name, cmd))

#        elif cmd.lower().startswith('pattern'):
#            # e.g. 'pattern P' or 'pattern Q' -- switch which closed-loop
#            # pattern is driven, live, without restarting the node.
#            value = cmd[7:].strip().upper()
#            if value in ('P', 'Q'):
#                self.pattern_prefix = value
#                rospy.loginfo('%s: closed-loop pattern set to %r' % (self.name, value))
#            else:
#                rospy.logwarn('%s: invalid pattern command: %r (expected "pattern P" or "pattern Q")' % (self.name, cmd))

#        elif cmd == 'help':
#            rospy.logwarn('Commands on %s/command:' % self.nodename.rstrip('/'))
#            rospy.logwarn('  start        Begin closed-loop (resets integrator)')
#            rospy.logwarn('  stop         Stop and blank display')
#            rospy.logwarn('  gain <value> Set gain (px/rad) live, e.g. "gain 150"')
#            rospy.logwarn('  pattern <P|Q> Set closed-loop pattern live, e.g. "pattern P"')
#            rospy.logwarn('  exit         Shutdown node')
#            rospy.logwarn('  help         This message')
#            rospy.logwarn('Other coefficients (XL1, XR1, etc.) are edited at top of script.')


#    def run(self):
#        rospy.spin()
#        self.pubTeensy.publish(String(data='0'))


################################################################################
#if __name__ == '__main__':
#    node = Flystate2Teensy()
#    node.run()
