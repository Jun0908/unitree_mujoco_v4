#!/usr/bin/env python3
"""
Keyframe-based boxing combo controller for Unitree G1 (29 DOF).
Combo: 1-2-3 (jab, cross, hook) + 1(body)-1-2 (body jab, head jab, cross)

Guard design: HIGH chin-guard — fist at z≈1.41 (face level), x≈0.16 (pulled in).
Each punch follows: LOAD → torso-leads → HIT → fast-retract.
Right arm stays at guard during all left punches, and vice versa.

Torso rotation leads arm extension by ~100 ms.

Sign conventions (G1 MuJoCo):
  waist_yaw  (12) : + = left shoulder forward (CCW from above)
  waist_pitch(14) : + = forward lean
  L_sh_pitch (15) : NEGATIVE = arm forward  ← opposite intuition!
  L_sh_roll  (16) : + = abduction (outward), - = adduction (toward bag)
  L_elbow    (18) : + = bent  (0 = straight)
  R_sh_pitch (22) : NEGATIVE = arm forward  (same convention as left)
  R_sh_roll  (23) : + = adduction, - = abduction
  R_elbow    (25) : + = bent
"""
import sys
import time

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# ── runtime ────────────────────────────────────────────────────────
G1_N   = 29
DT     = 0.01
DOMAIN = 1
IFACE  = "lo"
LOOP   = True

KP = [80,60,60,120,40,40, 80,60,60,120,40,40, 80,50,50,
      60,50,40,40,20,20,20, 60,50,40,40,20,20,20]
KD = [2,1,1,3,1,1, 2,1,1,3,1,1, 2,1,1, 2,1,1,1,1,1,1, 2,1,1,1,1,1,1]

# ── motor indices ──────────────────────────────────────────────────
LHP=0;LHR=1;LHY=2;LKN=3;LAP=4;LAR=5
RHP=6;RHR=7;RHY=8;RKN=9;RAP=10;RAR=11
WY=12;WR=13;WP=14
LSP=15;LSR=16;LSY=17;LEL=18
RSP=22;RSR=23;RSY=24;REL=25

# ── IK-calibrated arm poses (MuJoCo FK verified) ───────────────────
# Guard: fist ≈ (0.264, ±0.035, 1.289) — chin height, arm forward not raised
_LSP_G, _LSR_G, _LEL_G = -1.800, +0.100, 0.300   # left  guard
_RSP_G, _RSR_G, _REL_G = -1.800, -0.100, 0.300   # right guard (mirror)

# Left jab hit (WY=+0.22): fist ≈ (0.310, 0.080, 1.158), dist-to-bag 0.237 m
_LSP_J, _LSR_J, _LEL_J = -1.360, -0.150, 0.250

# Left body jab (WY=+0.10, WP=0.33): fist ≈ (0.361, 0.061, 1.003)
_LSP_B, _LSR_B, _LEL_B = -1.043, -0.200, 0.250

# Left hook: elbow stays at 90° throughout; torso rotation drives the arc
_LSP_HL, _LSR_HL, _LEL_HL = -1.800, +0.900, 1.570   # cocked (WY=+0.10)
_LSP_HC, _LSR_HC, _LEL_HC = -2.000, +0.200, 1.570   # contact (WY=+0.52), fist ≈ (0.308, 0.229, 1.195)

# Right cross hit (WY=−0.45): fist ≈ (0.286, −0.149, 1.155)
_RSP_C, _RSR_C, _REL_C = -1.360, +0.150, 0.250

# ── base guard pose ────────────────────────────────────────────────
_G = np.zeros(G1_N)
_G[LHP]=-0.43; _G[LKN]=0.50; _G[LAP]=0.22   # left  leg (bent knee)
_G[RHP]=-0.43; _G[RKN]=0.50; _G[RAP]=0.22   # right leg
_G[WP] = 0.12                                  # slight forward lean
_G[LSP]=_LSP_G; _G[LSR]=_LSR_G; _G[LEL]=_LEL_G   # left  arm guard
_G[RSP]=_RSP_G; _G[RSR]=_RSR_G; _G[REL]=_REL_G   # right arm guard


def P(d: dict) -> np.ndarray:
    """Build pose from guard base + overrides dict."""
    q = _G.copy()
    for k, v in d.items():
        q[k] = v
    return q


# ── easing ─────────────────────────────────────────────────────────
def _slow(t): t=max(0.,min(1.,t)); return t*t*(3-2*t)
def _pout(t): t=max(0.,min(1.,t)); u=1-t; return 1-u*u*u
def _pin(t):  t=max(0.,min(1.,t)); return t*t*t
def _qout(t): t=max(0.,min(1.,t)); u=1-t; return 1-u*u


# ── keyframe sequence ──────────────────────────────────────────────
# Each punch: LOAD (torso only) → torso-leads (arm still at guard)
#             → HIT (arm snaps) → RETRACT (return to guard fast)
# Right arm stays at guard during left punches; left stays during right cross.

KEYFRAMES = [

    # ── guard / fighter's bob ──────────────────────────────────────
    (0.00, _G.copy(), _slow),
    (0.40, P({WP:0.14, WY:+0.04}), _slow),
    (0.65, P({WP:0.10, WY:-0.03}), _slow),

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # SET 1 — 1-2-3
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    # ── 1. LEFT JAB ───────────────────────────────────────────────
    # right arm stays at _G guard throughout
    (0.73, P({WY:-0.10, WP:0.13}), _slow),          # LOAD: pull left shoulder back
    (0.83, P({WY:+0.08, WP:0.14}), _pout),          # torso leads (arm still at guard)
    (0.93, P({WY:+0.22, WP:0.15,                    # JAB HIT: arm snaps out
              LSP:_LSP_J, LSR:_LSR_J, LEL:_LEL_J}), _pin),
    (1.07, _G.copy(), _qout),                        # RETRACT fast

    # ── 2. RIGHT CROSS ────────────────────────────────────────────
    # left arm stays at _G guard throughout
    (1.14, P({WY:+0.06, WP:0.14}), _slow),          # LOAD: cock right shoulder back
    (1.24, P({WY:-0.22, WP:0.15,                    # torso drives right shoulder forward
              LSP:_LSP_G, LSR:_LSR_G, LEL:_LEL_G}), _pout),
    (1.33, P({WY:-0.45, WP:0.16,                    # CROSS HIT: right arm snaps out
              LSP:_LSP_G, LSR:_LSR_G, LEL:_LEL_G,
              RSP:_RSP_C, RSR:_RSR_C, REL:_REL_C}), _pin),
    (1.47, _G.copy(), _qout),                        # RETRACT fast

    # ── 3. LEFT HOOK ──────────────────────────────────────────────
    # elbow stays bent (LEL=1.57); torso rotation drives the arc
    (1.56, P({WY:+0.10, WP:0.14,                    # cock arm — out to left side
              LSP:_LSP_HL, LSR:_LSR_HL, LEL:_LEL_HL}), _slow),
    (1.66, P({WY:+0.35, WP:0.14,                    # torso fires hard
              LSP:_LSP_HL, LSR:_LSR_HL, LEL:_LEL_HL}), _pout),
    (1.75, P({WY:+0.52, WP:0.14,                    # HOOK HIT: arm sweeps into bag
              LSP:_LSP_HC, LSR:_LSR_HC, LEL:_LEL_HC}), _pin),
    (1.88, P({WY:+0.22, WP:0.13}), _qout),          # partial guard retract
    (2.05, _G.copy(), _slow),                        # full guard

    # ── reset ──────────────────────────────────────────────────────
    (2.55, _G.copy(), _slow),

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # SET 2 — 1(body)-1-2
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    (2.62, _G.copy(), _slow),
    (3.00, P({WP:0.26, WY:0.04, LKN:0.62, RKN:0.62,
              LAP:0.28, RAP:0.28}), _slow),                        # sink stance

    # ── 4. LEFT BODY JAB ──────────────────────────────────────────
    (3.08, P({WY:-0.08, WP:0.28, LKN:0.62, RKN:0.62,
              LAP:0.28, RAP:0.28}), _slow),                        # LOAD
    (3.18, P({WY:+0.10, WP:0.30, LKN:0.62, RKN:0.62,
              LAP:0.28, RAP:0.28}), _pout),                        # torso leads
    (3.27, P({WY:+0.10, WP:0.33,                                   # BODY JAB HIT (low)
              LSP:_LSP_B, LSR:_LSR_B, LEL:_LEL_B,
              LKN:0.65, RKN:0.65, LAP:0.30, RAP:0.30}), _pin),
    (3.39, P({WY:+0.04, WP:0.22, LKN:0.56, RKN:0.56,
              LAP:0.23, RAP:0.23}), _qout),                        # retract + rise

    # ── 5. LEFT HEAD JAB ──────────────────────────────────────────
    (3.44, P({WY:-0.10, WP:0.15, LKN:0.50, RKN:0.50}), _slow),   # LOAD
    (3.54, P({WY:+0.08, WP:0.15, LKN:0.50, RKN:0.50}), _pout),   # torso leads
    (3.63, P({WY:+0.22, WP:0.15,                                   # HEAD JAB HIT
              LSP:_LSP_J, LSR:_LSR_J, LEL:_LEL_J,
              LKN:0.50, RKN:0.50, LAP:0.22, RAP:0.22}), _pin),
    (3.74, _G.copy(), _qout),                                      # RETRACT fast

    # ── 6. RIGHT CROSS ────────────────────────────────────────────
    (3.80, P({WY:+0.06, WP:0.14}), _slow),                        # LOAD
    (3.90, P({WY:-0.22, WP:0.16,                                   # torso leads
              LSP:_LSP_G, LSR:_LSR_G, LEL:_LEL_G}), _pout),
    (3.99, P({WY:-0.45, WP:0.17,                                   # CROSS HIT
              LSP:_LSP_G, LSR:_LSR_G, LEL:_LEL_G,
              RSP:_RSP_C, RSR:_RSR_C, REL:_REL_C}), _pin),
    (4.10, _G.copy(), _qout),                                      # RETRACT

    # ── final guard ────────────────────────────────────────────────
    (4.18, _G.copy(), _slow),
    (4.23, _G.copy(), _slow),
]

DURATION = KEYFRAMES[-1][0]


def interpolate(t: float) -> np.ndarray:
    t = max(0.0, min(DURATION, t))
    for i in range(len(KEYFRAMES) - 1):
        t0, q0, _   = KEYFRAMES[i]
        t1, q1, fn  = KEYFRAMES[i + 1]
        if t0 <= t < t1:
            return q0 + fn((t - t0) / (t1 - t0)) * (q1 - q0)
    return KEYFRAMES[-1][1].copy()


# ── DDS controller ─────────────────────────────────────────────────
class BoxingComboController:
    def __init__(self):
        self.low_state    = None
        self.have_state   = False
        self.mode_machine = 5
        self.crc          = CRC()
        self.cmd          = unitree_hg_msg_dds__LowCmd_()
        self.publisher    = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()
        self.subscriber   = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._state_cb, 10)

    def _state_cb(self, msg: LowState_):
        self.low_state = msg
        if not self.have_state:
            self.mode_machine = msg.mode_machine
            self.have_state   = True
            print("State received — starting boxing combo.")

    def _send(self, q: np.ndarray):
        self.cmd.mode_pr      = 0
        self.cmd.mode_machine = self.mode_machine
        for i in range(G1_N):
            m       = self.cmd.motor_cmd[i]
            m.mode  = 1
            m.q     = float(q[i])
            m.dq    = 0.0
            m.kp    = KP[i]
            m.kd    = KD[i]
            m.tau   = 0.0
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.publisher.Write(self.cmd)

    def run(self):
        print("Waiting for rt/lowstate …")
        while not self.have_state:
            time.sleep(0.1)

        combo   = 0
        t_start = time.perf_counter()
        while True:
            now     = time.perf_counter()
            elapsed = now - t_start
            ct      = elapsed % DURATION if LOOP else min(elapsed, DURATION)

            if LOOP and elapsed > 0 and (elapsed % DURATION) < DT:
                combo += 1
                print(f"── combo #{combo} ──")

            q = interpolate(ct)
            self._send(q)

            if self.low_state and int(elapsed) != int(max(elapsed - DT, 0)):
                print(
                    f"boxing t={elapsed:6.2f}s  ct={ct:4.2f}s "
                    f"WY={q[WY]:+.2f}  LSP={q[LSP]:+.2f}  RSP={q[RSP]:+.2f}  "
                    f"L_el={q[LEL]:+.2f}  R_el={q[REL]:+.2f}"
                )

            sleep = DT - (time.perf_counter() - now)
            if sleep > 0:
                time.sleep(sleep)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(DOMAIN, IFACE)
    BoxingComboController().run()
