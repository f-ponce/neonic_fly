#!/usr/bin/env python3
"""
Randomized motion-pattern + loom trial sequence for
patterns_091326_geometrycorrected.ino, each trial followed by closed loop.

Uses the N<pattern>/S<speed>/P<position> protocol. Motion is in real
degrees, not pixels -- S/P take signed floats (e.g. "S45.5", "P-12.3").
Patterns 1 (stripe grid), 2 (single stripe), and 9 (starfield) share one
motion state on the Teensy, so direction is just the sign of the speed
sent.

Pattern map (see the .ino for the authoritative version):
  1      stripe grid
  2      single stripe
  3-5    loom, L/C/R, Kim et al. 2023 speed (h/v=26ms, duration ~3.0s)
  6-8    loom, L/C/R, Ros et al. 2024 speed (h/v=100ms, duration ~11.5s)
  9      starfield: grayscale 3D point cloud
  10-13  diagnostics (N10-N13 only, no single-digit room left)

STRIPE_SPEEDS_DEG_S is derived from Drosophila optomotor tuning
literature (Theobald et al. 2010; Duistermars et al. 2007) at the stripe
grid's own 30 deg spatial period. STARFIELD_SPEEDS_DEG_S borrows the
stripe grid's MEDIUM value as an interim stand-in; not independently
sourced for a random-dot stimulus. CLOSED_LOOP_GAIN comes from Giraldo et
al. 2018 (see flystate2teensy.py).

Sequence:
    2s panels OFF, then all motion-section trials in one randomly
    shuffled order:
      - MOTION trials (stripe grid x speeds x directions, starfield x
        directions), each repeated TRIAL_REPEATS times
      - "PROPER" CL trials (dedicated closed-loop-only, no open-loop
        movement), each repeated CL_ONLY_REPEATS times

    Each MOTION trial (pattern 1, 2, or 9):
        1s  static (frozen, at STATIC_POSITION_DEG)
        10s moving (that trial's speed + direction, open loop)
        1s  static (frozen, at STATIC_POSITION_DEG)
        5s  in-between closed loop (always single stripe -- see the TWO
            KINDS OF CLOSED LOOP block)

    Each "PROPER" CL trial (CL_ONLY_PATTERNS: single stripe or stripe grid):
        1s  static
        10s closed loop, driving that trial's own pattern -- this is the
            trial's main content, playing the role a moving phase plays
            in a motion trial
        1s  static
        5s  in-between closed loop (same standard tail as every trial)

    Then the loom section (patterns 3-8), each repeated LOOM_TRIAL_REPEATS
    times, shuffled. Each loom trial:
        3s  static before (pattern 14: loom background only, the exact
            buffer the loom itself uses, no disc)
        loom's own derived duration (~3.0s Kim / ~11.5s Ros)
        3s  static after (same background, no pattern switch)
        5s  in-between closed loop

Avoiding drift in closed loop: patterns 1/2/9 share one motion state on
the Teensy. A leftover nonzero speed when closed loop starts would let
the Teensy's own integrator keep running on top of the bridge's own
P<offset>/S<velocity> updates. This script sends 'S0' before every
static freeze and before every closed-loop handoff; the bridge sends its
own 'S0' on every start/pattern switch as a second line of defense.

Requires:
    - flystate2teensy.py (degrees-based) started WITHOUT anonymous=True
      (fixed command topic)
    - patterns_091326_geometrycorrected.ino running on the Teensy
    - teensy_serial.py with the N/S/P whitelist (S/P accept signed decimals)
"""
import random
import rospy
from std_msgs.msg import String
import math

# ── Experiment parameters ────────────────────────────────────────────────────
OFF_DURATION            = 2.0
STATIC_DURATION         = 1.0
MOVING_DURATION         = 10.0
CLOSED_LOOP_GAIN        = -300.0   # deg/rad; see flystate2teensy.py for the Giraldo et al. 2018 derivation.(low would be 270)
                                    # Shared by both kinds of closed loop below.
STATIC_POSITION_DEG     = 0.0      # deg, straight ahead / center

# deg/s. SLOW/MEDIUM/FAST derived from Drosophila optomotor tuning at the
# stripe grid's own 30 deg spatial period (15 deg stripe from Kim et al.
# 2023 + 15 deg gap): speed = temporal frequency (Hz) x spatial period
# (deg). Theobald et al. 2010 (J Exp Biol 213) report rotation-response
# temporal-frequency optima of 3-12 Hz at a saturating spatial wavelength
# of 30 deg -- our exact period. Duistermars et al. 2007 independently
# used 12 Hz as a tested large-field grating condition (alongside a 30
# deg-wide single stripe).
#   3 Hz  x 30 deg = 90  deg/s  (SLOW)
#   ~6 Hz x 30 deg = 180 deg/s  (MEDIUM)
#   12 Hz x 30 deg = 360 deg/s  (FAST)
STRIPE_SPEEDS_DEG_S    = {'SLOW': 30.0, 'MEDIUM': 90.0, 'FAST': 180.0}
#freq: 1, 3, 6, HZ @ {'SLOW': 30.0, 'MEDIUM': 90.0, 'FAST': 180.0}
STARFIELD_SPEEDS_DEG_S = {'MEDIUM': STRIPE_SPEEDS_DEG_S['MEDIUM']} 

DIRECTIONS = {'LEFT': -1, 'RIGHT': 1}     # sign multiplied onto the speed magnitude

STRIPE_GRID_PATTERN  = 1
SINGLE_STRIPE_PATTERN = 2
STARFIELD_PATTERN    = 9

TRIAL_REPEATS = 2   # each stripe/starfield speed+direction combo presented this many times.
                     # Independent from LOOM_TRIAL_REPEATS below.

# ═══════════════════════════════════════════════════════════════════════════
# TWO KINDS OF CLOSED LOOP:
#
# 1. IN-BETWEEN CL -- the short closed-loop tail attached to the end of
#    every other trial (motion, starfield, loom). Always single stripe,
#    always this short duration, regardless of the trial's own
#    pattern/speed. Not a dedicated trial -- the standard wrap-up.
# 2. PROPER CL TRIALS -- dedicated, standalone trials whose entire purpose
#    is testing closed loop itself, at the longer duration, with an
#    explicit pattern (single stripe or stripe grid). Real entries in the
#    shuffled trial list, mixed in with the open-loop motion trials.
# ═══════════════════════════════════════════════════════════════════════════
INBETWEEN_CL_DURATION_S = 5.0
INBETWEEN_CL_PATTERN    = SINGLE_STRIPE_PATTERN   # always single stripe, regardless of the preceding trial

CL_TRIAL_DURATION_S = 10.0
CL_ONLY_REPEATS  = 2
CL_ONLY_PATTERNS = {
    'CL SINGLE STRIPE': SINGLE_STRIPE_PATTERN,
    'CL STRIPE GRID':   STRIPE_GRID_PATTERN,
}

LOOM_PATTERNS        = [3, 4, 5, 6, 7, 8]  # L/C/R at Kim et al. 2023 speed, then L/C/R at Ros et al. 2024 speed
LOOM_TRIAL_REPEATS   = 2                   # each loom variant presented this many times.
                                            # Independent from TRIAL_REPEATS above.

# Mirrors the .ino's own loom-duration derivation exactly (same h/v
# values, same LOOM_START_VISIBLE_HALF_ANGLE_DEG threshold), so this
# script knows each pattern's real firmware duration rather than guessing
# a shared wait. If these values change in the .ino, change them here too.
LOOM_H_OVER_V_KIM_S            = 0.026  # must match the .ino
LOOM_H_OVER_V_ROS_S            = 0.100  # must match the .ino
LOOM_START_VISIBLE_HALF_ANGLE_DEG = 0.5  # must match the .ino

def _loom_duration_s(h_over_v):
    return h_over_v / math.tan(math.radians(LOOM_START_VISIBLE_HALF_ANGLE_DEG))

LOOM_DURATION_S = {
    3: _loom_duration_s(LOOM_H_OVER_V_KIM_S), 4: _loom_duration_s(LOOM_H_OVER_V_KIM_S), 5: _loom_duration_s(LOOM_H_OVER_V_KIM_S),
    6: _loom_duration_s(LOOM_H_OVER_V_ROS_S), 7: _loom_duration_s(LOOM_H_OVER_V_ROS_S), 8: _loom_duration_s(LOOM_H_OVER_V_ROS_S),
}

# Static bracketing around each loom trial. Both phases show pattern 14
# (loom background only, no disc) -- the exact same buffer the loom
# patterns display behind their disc, identical across all 6 loom
# patterns since it's built once at boot. This avoids a real mismatch a
# live-rendered starfield would have: overlapping stars at the same pixel
# get resolved differently by a live per-frame draw (last-drawn-wins)
# than by this precomputed background (brightest-wins), which at this
# star density is a visible difference, not a subtle one.
LOOM_STATIC_BEFORE_DURATION = 3.0
LOOM_STATIC_AFTER_DURATION  = 3.0

FLYSTATE_NODE_COMMAND_TOPIC = '/Flystate2Teensy/command'

LOOM_LABELS = {
    3: 'LOOM LEFT KIM2023',
    4: 'LOOM CENTER KIM2023',
    5: 'LOOM RIGHT KIM2023',
    6: 'LOOM LEFT ROS2024',
    7: 'LOOM CENTER ROS2024',
    8: 'LOOM RIGHT ROS2024',
}


def send_command(pub, cmd, label):
    msg = String()
    msg.data = cmd
    rospy.loginfo(f"Sending: {label} (cmd={cmd})")
    pub.publish(msg)


def wait(seconds, label):
    rospy.loginfo(f"Waiting {seconds}s: {label}")
    rospy.sleep(seconds)


def build_motion_trial_list():
    trials = []

    for speed_label, speed_deg_s in STRIPE_SPEEDS_DEG_S.items():
        for dir_label, sign in DIRECTIONS.items():
            for _ in range(TRIAL_REPEATS):
                trials.append({
                    'type': 'motion',
                    'pattern_num': STRIPE_GRID_PATTERN,
                    'speed_signed': sign * speed_deg_s,
                    'label': f'STRIPE GRID {dir_label} {speed_label}',
                })

    for speed_label, speed_deg_s in STARFIELD_SPEEDS_DEG_S.items():
        for dir_label, sign in DIRECTIONS.items():
            for _ in range(TRIAL_REPEATS):
                trials.append({
                    'type': 'motion',
                    'pattern_num': STARFIELD_PATTERN,
                    'speed_signed': sign * speed_deg_s,
                    'label': f'STARFIELD {dir_label} {speed_label}',
                })

    for cl_label, cl_pattern in CL_ONLY_PATTERNS.items():
        for _ in range(CL_ONLY_REPEATS):
            trials.append({
                'type': 'closed_loop_only',
                'pattern_num': cl_pattern,
                'label': cl_label,
            })

    random.shuffle(trials)  # single shuffled list, all motion trial types interleaved
    return trials


def build_loom_trial_list():
    trials = []
    for pattern_num in LOOM_PATTERNS:
        for _ in range(LOOM_TRIAL_REPEATS):
            trials.append({
                'type': 'loom',
                'pattern_num': pattern_num,
                'label': LOOM_LABELS[pattern_num],
                'loom_duration_s': LOOM_DURATION_S[pattern_num],
            })
    random.shuffle(trials)
    return trials


def run_closed_loop(closedloop_pub, pattern, duration):
    """No implicit defaults -- every caller states explicitly which kind
    of closed loop it means (see the TWO KINDS OF CLOSED LOOP block
    above), so there's no ambiguity about which pattern/duration is in
    effect at any call site."""
    send_command(closedloop_pub, f'gain {CLOSED_LOOP_GAIN}', f'Set gain {CLOSED_LOOP_GAIN}')
    send_command(closedloop_pub, f'pattern {pattern}', f'Set pattern {pattern}')
    send_command(closedloop_pub, 'start', 'Closed-loop START')
    wait(duration, f'closed-loop (gain={CLOSED_LOOP_GAIN}, pattern={pattern}, duration={duration}s)')
    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')


def run_motion_trial(teensy_pub, closedloop_pub, trial, idx, total):
    rospy.loginfo(f"--- Trial {idx}/{total}: {trial['label']} ---")

    send_command(teensy_pub, f"N{trial['pattern_num']}", f"Select pattern {trial['pattern_num']}")
    send_command(teensy_pub, 'S0', 'Zero speed (freeze)')
    send_command(teensy_pub, f'P{STATIC_POSITION_DEG}', 'Static position (pre)')
    wait(STATIC_DURATION, 'static (pre)')

    send_command(teensy_pub, f"S{trial['speed_signed']}", f"Speed {trial['speed_signed']} deg/s")
    wait(MOVING_DURATION, f"moving ({trial['label']})")

    # Freeze before the static break AND before handing off to closed loop.
    # This S0 is what stops the Teensy from continuing to obey
    # trial['speed_signed'] through the static-break wait below.
    send_command(teensy_pub, 'S0', 'Zero speed (freeze)')
    send_command(teensy_pub, f'P{STATIC_POSITION_DEG}', 'Static position (post)')
    wait(STATIC_DURATION, 'static (post)')

    run_closed_loop(closedloop_pub, INBETWEEN_CL_PATTERN, INBETWEEN_CL_DURATION_S)


def run_closed_loop_only_trial(teensy_pub, closedloop_pub, trial, idx, total):
    """A 'PROPER' CL trial (see the TWO KINDS OF CLOSED LOOP block above)
    -- same four-phase shape as every other trial type (static pre / main
    content / static post / in-between CL tail), just with no open-loop
    movement phase: the trial's own main content IS the longer closed-loop
    duration, driving whichever pattern this specific trial names."""
    rospy.loginfo(f"--- Trial {idx}/{total}: {trial['label']} ---")

    send_command(teensy_pub, f"N{trial['pattern_num']}", f"Select pattern {trial['pattern_num']}")
    send_command(teensy_pub, 'S0', 'Zero speed (freeze)')
    send_command(teensy_pub, f'P{STATIC_POSITION_DEG}', 'Static position (pre)')
    wait(STATIC_DURATION, 'static (pre)')

    run_closed_loop(closedloop_pub, trial['pattern_num'], CL_TRIAL_DURATION_S)

    # Static (post): same as every other trial type -- freeze and return
    # to the reference position after the main content phase (here,
    # wherever the fly's own closed-loop steering left it), before the
    # standard in-between CL tail.
    send_command(teensy_pub, 'S0', 'Zero speed (freeze)')
    send_command(teensy_pub, f'P{STATIC_POSITION_DEG}', 'Static position (post)')
    wait(STATIC_DURATION, 'static (post)')

    run_closed_loop(closedloop_pub, INBETWEEN_CL_PATTERN, INBETWEEN_CL_DURATION_S)


def run_loom_trial(teensy_pub, closedloop_pub, trial, idx, total):
    rospy.loginfo(f"--- Loom trial {idx}/{total}: {trial['label']} ---")

    # Static before: pattern 14 (loom background only, no disc) -- the
    # exact same buffer the loom itself displays behind its disc, not a
    # live re-derived approximation. No motion state to reset (pattern 14
    # has none), so this is a single command.
    send_command(teensy_pub, "N14", "Static loom background (pre)")
    wait(LOOM_STATIC_BEFORE_DURATION, f"static before loom ({trial['label']})")

    # The loom itself: runs for exactly its own derived duration (see
    # LOOM_DURATION_S), growing from imperceptible to impact, then
    # dropping to background-only automatically once impact passes.
    send_command(teensy_pub, f"N{trial['pattern_num']}", trial['label'])
    wait(trial['loom_duration_s'], f"loom playing ({trial['label']})")

    # Static after: the loom pattern is already showing background-only
    # at this point (past impact) -- pixel-identical to the "before"
    # phase above, so nothing needs to change here at all, just wait.
    wait(LOOM_STATIC_AFTER_DURATION, f"static after loom ({trial['label']})")

    run_closed_loop(closedloop_pub, INBETWEEN_CL_PATTERN, INBETWEEN_CL_DURATION_S)


def main():
    rospy.init_node('experiment_patterns_091326')

    teensy_pub     = rospy.Publisher('/teensy/command', String, queue_size=10)
    closedloop_pub = rospy.Publisher(FLYSTATE_NODE_COMMAND_TOPIC, String, queue_size=10)

    rospy.sleep(1.0)  # allow publishers to connect

    send_command(closedloop_pub, 'stop', 'Closed-loop STOP (safety reset)')
    send_command(teensy_pub, 'S0', 'Zero speed (safety reset)')

     #--- COMMENTED OUT FOR LOOM-ONLY TESTING ---
    motion_trials = build_motion_trial_list()
    rospy.loginfo(f"Starting randomized motion section: {len(motion_trials)} trials (order shuffled).")
    rospy.loginfo("Order: " + ', '.join(t['label'] for t in motion_trials))

    send_command(teensy_pub, 'N0', 'OFF')
    wait(OFF_DURATION, 'initial off period')

    # --- COMMENTED OUT FOR LOOM-ONLY TESTING ---
    for i, trial in enumerate(motion_trials, start=1):
         if trial['type'] == 'motion':
             run_motion_trial(teensy_pub, closedloop_pub, trial, i, len(motion_trials))
         elif trial['type'] == 'closed_loop_only':
             run_closed_loop_only_trial(teensy_pub, closedloop_pub, trial, i, len(motion_trials))

    # ── Loom section ──────────────────────────────────────────────────────────
    loom_trials = build_loom_trial_list()
    rospy.loginfo(f"Starting loom section: {len(loom_trials)} trials (order shuffled).")
    rospy.loginfo("Order: " + ', '.join(t['label'] for t in loom_trials))

    for i, trial in enumerate(loom_trials, start=1):
        run_loom_trial(teensy_pub, closedloop_pub, trial, i, len(loom_trials))

    send_command(closedloop_pub, 'stop', 'Closed-loop STOP')
    send_command(teensy_pub, 'N0', 'OFF')
    rospy.loginfo("Experiment complete.")


if __name__ == '__main__':
    main()
