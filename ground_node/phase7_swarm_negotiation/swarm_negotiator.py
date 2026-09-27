"""
=============================================================================
  Sentinel Swarm — Phase 7: Cooperative Swarm Negotiation Engine
  Ground Node (Raspberry Pi 4 + XBee PRO S2C)
=============================================================================

SWARM NEGOTIATION PROTOCOL:
  Each robot independently runs this code. They are symmetric peers — no
  hardcoded master or slave. The "leader" is dynamically elected every
  tick using a weighted voting protocol over the XBee network.

ELECTION ALGORITHM (Weighted Borda Count Vote):
  Every robot broadcasts a "vote packet" containing:
    - confidence:  YOLO detection confidence (0.0 - 1.0)
    - freshness:   how recently the detection was made (decays over time)
    - target_size: normalized bounding box area (bigger = closer = better view)
  
  Each robot computes a SCORE for itself and all peers:
    SCORE = (0.6 * confidence) + (0.3 * freshness) + (0.1 * target_size)
  
  The robot with the highest SCORE becomes the LEADER for that tick.
  The leader's target coordinates are used by ALL robots to steer their motors.
  This means the robot with the "best view" automatically leads the pursuit.

HANDOFF SCENARIOS HANDLED:
  1. Robot A tracking → A loses sight → B takes over instantly
  2. Both robots lose sight → both stop. Watchdog triggers scan rotation.
  3. Robot A and B both see target → whoever has HIGHER confidence leads.
  4. A has stale detection, B gets fresh one → B's score overtakes A → B leads.
  5. XBee link drops → robot falls back to own camera only (safe degraded mode).

COMPATIBLE WITH: phase4_tracking/ground_tracker.py
  Phase 7 adds SwarmNegotiator as a drop-in layer on top of SwarmState.
  The control_loop in ground_tracker.py calls state.best_target() exactly
  as before — Phase 7 internally handles the leadership voting transparently.
"""

import time
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Dict
from enum import Enum

log = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

FRESHNESS_DECAY_HZ    = 2.0     # Confidence decays to 0 after 1/DECAY_HZ seconds
STALE_TIMEOUT_S       = 1.5     # After this, a peer's detection is considered gone
ELECTION_INTERVAL_S   = 0.1     # Re-elect leader every 100ms
SCAN_ROTATION_TIMEOUT = 3.0     # Seconds with no target before starting scan sweep

WEIGHT_CONFIDENCE  = 0.60
WEIGHT_FRESHNESS   = 0.30
WEIGHT_TARGET_SIZE = 0.10


# ─────────────────────────────────────────────────────────────────────────────
# Data Structures
# ─────────────────────────────────────────────────────────────────────────────

class RobotRole(Enum):
    LEADER   = "LEADER"     # Has best view, commands the swarm direction
    FOLLOWER = "FOLLOWER"   # Defers to leader's target coordinates
    SCANNING = "SCANNING"   # No target anywhere in swarm → rotating to search
    SOLO     = "SOLO"       # XBee dead → using own camera only


@dataclass
class PeerDetection:
    """A single detection broadcast received from a peer over XBee."""
    robot_id    : str
    cx          : float          # Normalized x center (0.0 left, 1.0 right)
    cy          : float          # Normalized y center
    confidence  : float          # Raw YOLO confidence
    target_size : float          # Normalized bbox area (w*h / frame_area)
    received_at : float = field(default_factory=time.time)

    @property
    def freshness(self) -> float:
        """Decays from 1.0 to 0.0 over STALE_TIMEOUT_S seconds."""
        age = time.time() - self.received_at
        return max(0.0, 1.0 - (age / STALE_TIMEOUT_S))

    @property
    def is_stale(self) -> bool:
        return self.freshness == 0.0

    def score(self) -> float:
        """Weighted Borda Count score for leader election."""
        return (WEIGHT_CONFIDENCE  * self.confidence +
                WEIGHT_FRESHNESS   * self.freshness   +
                WEIGHT_TARGET_SIZE * self.target_size)


@dataclass
class SelfDetection:
    """Own YOLO detection, mirroring PeerDetection structure."""
    robot_id    : str
    cx          : float
    cy          : float
    confidence  : float
    target_size : float
    detected_at : float = field(default_factory=time.time)

    @property
    def freshness(self) -> float:
        age = time.time() - self.detected_at
        return max(0.0, 1.0 - (age / STALE_TIMEOUT_S))

    @property
    def is_stale(self) -> bool:
        return self.freshness == 0.0

    def score(self) -> float:
        return (WEIGHT_CONFIDENCE  * self.confidence +
                WEIGHT_FRESHNESS   * self.freshness   +
                WEIGHT_TARGET_SIZE * self.target_size)

    def to_xbee_packet(self) -> dict:
        """Serialize to compact XBee JSON packet."""
        return {
            'r'  : self.robot_id,
            'cx' : round(self.cx, 3),
            'cy' : round(self.cy, 3),
            'cf' : round(self.confidence, 3),
            'sz' : round(self.target_size, 4),
            'st' : 'T',                         # T = Tracking, L = Lost
        }


@dataclass
class LeaderDecision:
    """The result of a leader election tick."""
    leader_id   : str
    role        : RobotRole
    source_cx   : float     # The target X coord to steer towards (from leader)
    source_cy   : float
    confidence  : float
    score       : float


# ─────────────────────────────────────────────────────────────────────────────
# Swarm Negotiator (Phase 7 Core)
# ─────────────────────────────────────────────────────────────────────────────

class SwarmNegotiator:
    """
    Phase 7 Cooperative Swarm Negotiation Engine.

    Drop-in layer on top of the existing SwarmState. The control loop
    calls get_steering_target() instead of state.best_target() to get
    the negotiated, swarm-wide best target.

    Usage:
        negotiator = SwarmNegotiator(robot_id='A', xbee=xbee_transport)
        negotiator.register_self_detection(cx, cy, confidence, target_size)
        target = negotiator.get_steering_target()  # Returns (cx, error_x, role) or None
    """

    def __init__(self, robot_id: str, xbee, xbee_link_alive_fn=None):
        """
        robot_id: 'A' or 'B' (or any string ID)
        xbee:     XBeeTransport instance from shared/xbee_transport.py
        xbee_link_alive_fn: callable → bool (checks if XBee link is alive)
        """
        self.robot_id        = robot_id
        self.xbee            = xbee
        self.link_alive_fn   = xbee_link_alive_fn or (lambda: True)

        self._self_det : Optional[SelfDetection]         = None
        self._peers    : Dict[str, PeerDetection]        = {}
        self._lock     = threading.Lock()

        self._current_role    = RobotRole.SOLO
        self._current_leader  = robot_id
        self._last_any_target = 0.0    # Timestamp of last detection anywhere in swarm
        self._scan_angle      = 0.0    # For scan sweep behavior

        # Start background election thread
        t = threading.Thread(target=self._election_loop, daemon=True)
        t.start()

        log.info(f"[Phase7] SwarmNegotiator started for Robot {robot_id}")

    # ── Public API ────────────────────────────────────────────────────────────

    def register_self_detection(self, cx: float, cy: float,
                                 confidence: float, target_size: float):
        """
        Call this every time YOLO detects a person.
        Thread-safe. Also broadcasts the detection to peers via XBee.
        """
        det = SelfDetection(
            robot_id    = self.robot_id,
            cx          = cx,
            cy          = cy,
            confidence  = confidence,
            target_size = target_size,
        )
        with self._lock:
            self._self_det        = det
            self._last_any_target = time.time()

        # Broadcast to swarm over XBee
        self.xbee.send(det.to_xbee_packet())

    def register_self_lost(self):
        """
        Call this when YOLO finds no person in the current frame.
        Broadcasts a LOST packet to peers.
        """
        with self._lock:
            self._self_det = None
        self.xbee.send({'r': self.robot_id, 'st': 'L'})

    def ingest_xbee_packet(self, packet: dict):
        """
        Call this for every packet received from XBee.
        Filters own echoes, updates peer detection table.
        """
        if not packet or packet.get('r') == self.robot_id:
            return  # Ignore own echoes

        peer_id = packet.get('r')
        status  = packet.get('st')

        with self._lock:
            if status == 'T':
                self._peers[peer_id] = PeerDetection(
                    robot_id    = peer_id,
                    cx          = packet.get('cx', 0.5),
                    cy          = packet.get('cy', 0.5),
                    confidence  = packet.get('cf', 0.0),
                    target_size = packet.get('sz', 0.01),
                )
                self._last_any_target = time.time()
                log.debug(f"[Phase7] Peer {peer_id} tracking: "
                          f"cx={packet.get('cx'):.3f} conf={packet.get('cf'):.2f}")

            elif status == 'L':
                # Peer lost the target → remove from peer table
                self._peers.pop(peer_id, None)
                log.debug(f"[Phase7] Peer {peer_id} lost target.")

    def get_steering_target(self):
        """
        Returns the negotiated steering target for the control loop.

        Returns:
            (cx, error_x, role, leader_id, score) tuple if a target exists
            None if the entire swarm has no valid target
        """
        decision = self._last_decision
        if decision is None:
            return None
        return decision

    @property
    def current_role(self) -> RobotRole:
        return self._current_role

    # ── Election Loop (Background Thread) ─────────────────────────────────────

    def _election_loop(self):
        """Runs every 100ms. Elects a leader and caches the decision."""
        self._last_decision = None
        while True:
            time.sleep(ELECTION_INTERVAL_S)
            self._last_decision = self._run_election()

    def _run_election(self) -> Optional[LeaderDecision]:
        """
        Core election algorithm. Returns a LeaderDecision or None.
        """
        with self._lock:
            # 1. Check XBee link status
            xbee_live = self.link_alive_fn()

            # 2. Build candidate table (self + live peers)
            candidates: Dict[str, tuple] = {}

            if self._self_det and not self._self_det.is_stale:
                candidates[self.robot_id] = (
                    self._self_det.score(),
                    self._self_det.cx,
                    self._self_det.cy,
                    self._self_det.confidence,
                )

            if xbee_live:
                for peer_id, peer in list(self._peers.items()):
                    if peer.is_stale:
                        del self._peers[peer_id]   # Purge stale peers
                        continue
                    candidates[peer_id] = (
                        peer.score(),
                        peer.cx,
                        peer.cy,
                        peer.confidence,
                    )

        # 3. No candidates at all → check scan condition
        if not candidates:
            time_since_any = time.time() - self._last_any_target
            if time_since_any > SCAN_ROTATION_TIMEOUT and self._last_any_target > 0:
                self._current_role = RobotRole.SCANNING
                log.info(f"[Phase7] ⟳ No target for {time_since_any:.1f}s — SCANNING")
            else:
                self._current_role = RobotRole.SOLO if not xbee_live else RobotRole.FOLLOWER
            return None

        # 4. Elect: highest score wins
        leader_id = max(candidates, key=lambda k: candidates[k][0])
        best_score, cx, cy, conf = candidates[leader_id]

        # 5. Assign role
        if not xbee_live:
            role = RobotRole.SOLO
        elif leader_id == self.robot_id:
            role = RobotRole.LEADER
        else:
            role = RobotRole.FOLLOWER

        self._current_role   = role
        self._current_leader = leader_id

        # error_x: negative = target left of center, positive = right
        error_x = cx - 0.5

        if role == RobotRole.LEADER:
            log.debug(f"[Phase7] 👑 LEADER (self) | score={best_score:.3f} "
                      f"conf={conf:.2f} cx={cx:.3f}")
        else:
            log.debug(f"[Phase7] 🔄 FOLLOWER → leader={leader_id} | "
                      f"score={best_score:.3f} conf={conf:.2f}")

        return LeaderDecision(
            leader_id  = leader_id,
            role       = role,
            source_cx  = cx,
            source_cy  = cy,
            confidence = conf,
            score      = best_score,
        )


# ─────────────────────────────────────────────────────────────────────────────
# Phase 7 Integration Helpers (Wrap existing ground_tracker.py)
# ─────────────────────────────────────────────────────────────────────────────

ROLE_COLORS = {
    RobotRole.LEADER   : (0, 255, 128),   # Bright green
    RobotRole.FOLLOWER : (255, 200, 0),   # Yellow
    RobotRole.SCANNING : (0, 165, 255),   # Orange
    RobotRole.SOLO     : (128, 128, 128), # Grey
}

ROLE_LABELS = {
    RobotRole.LEADER   : "LEADER",
    RobotRole.FOLLOWER : "FOLLOWER",
    RobotRole.SCANNING : "SCANNING",
    RobotRole.SOLO     : "SOLO MODE",
}


def annotate_frame_phase7(frame, robot_id: str, decision: Optional[LeaderDecision],
                           self_conf: float = 0.0):
    """
    Draws Phase 7 HUD overlay on the camera frame.
    Shows role badge, leader ID, score, and target crosshair.
    """
    import cv2
    h, w = frame.shape[:2]

    if decision is None:
        role   = RobotRole.SCANNING
        color  = ROLE_COLORS[role]
        label  = f"Bot {robot_id} | {ROLE_LABELS[role]} | No target"
    else:
        role   = decision.role
        color  = ROLE_COLORS[role]
        leader = decision.leader_id
        label  = (f"Bot {robot_id} | {ROLE_LABELS[role]}"
                  f" | Leader={leader}"
                  f" | score={decision.score:.2f}"
                  f" | conf={decision.confidence:.2f}")

        # Draw target crosshair at leader's reported position
        tx = int(decision.source_cx * w)
        ty = int(decision.source_cy * h)
        cv2.drawMarker(frame, (tx, ty), color,
                       markerType=cv2.MARKER_CROSS,
                       markerSize=30, thickness=2)
        cv2.circle(frame, (tx, ty), 20, color, 2)

    # Role badge background
    cv2.rectangle(frame, (0, 0), (w, 22), (0, 0, 0), -1)
    cv2.putText(frame, label, (4, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    # Own confidence bar (bottom strip)
    bar_w = int(self_conf * w)
    cv2.rectangle(frame, (0, h - 6), (bar_w, h), color, -1)

    return frame
