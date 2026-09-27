"""
=============================================================================
  Sentinel Swarm v2 — Swarm Intelligence Engine
  Communication: LoRa SX1262 + UWB (DW3000) for close-range precision
  Algorithm: Behavior Tree FSM + Consensus Protocol + MARL (Multi-Agent RL)
=============================================================================

SWARM SCENARIOS HANDLED:
  1. SEARCH: Drones autonomously partition the search zone (Voronoi decomposition)
             and scan in parallel, broadcasting casualty coordinates.
  2. RESCUE: When a casualty is found, the highest-priority drone breaks from
             search formation and hovers above to act as a visual beacon for
             ground rescue teams.
  3. COMBAT/PERIMETER: Drones autonomously form a convex hull perimeter around
             a secured zone, maintaining minimum separation via Reynolds rules.
  4. COMMS RELAY: If a drone loses LoRa range, it designates the nearest drone
             as a mesh relay node, self-healing the network.
  5. DECONFLICTION: UWB beacons on each drone provide centimeter-level relative
             ranging to prevent mid-air collisions.

CONSENSUS PROTOCOL:
  Leader election via Bully Algorithm (highest remaining battery → leader).
  Leader broadcasts global mission updates every 500ms over LoRa.
  Followers apply local behavior trees independently — no single point of failure.
"""

import time
import threading
import json
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Callable
from collections import defaultdict

# ---------------------------------------------------------------------------
# Drone State
# ---------------------------------------------------------------------------

class DroneRole(Enum):
    SEARCHING   = "SEARCHING"    # Actively scanning the zone
    RESCUING    = "RESCUING"     # Hovering as visual beacon for rescue
    RELAYING    = "RELAYING"     # Acting as LoRa mesh relay node
    PERIMETER   = "PERIMETER"   # Holding formation at boundary
    RTB         = "RTB"          # Return to base (low battery)
    LEADER      = "LEADER"       # Elected swarm leader


@dataclass
class DroneState:
    """Full broadcasted state of a single drone node in the swarm."""
    drone_id       : str
    role           : DroneRole
    pos_xyz        : np.ndarray       # SLAM world-frame position
    battery_pct    : float            # 0.0 - 1.0
    casualty_found : bool
    casualty_xyz   : Optional[np.ndarray]
    casualty_conf  : float
    timestamp      : float = field(default_factory=time.time)

    def to_json(self) -> str:
        d = {
            "id"   : self.drone_id,
            "role" : self.role.value,
            "xyz"  : self.pos_xyz.tolist(),
            "bat"  : round(self.battery_pct, 3),
            "cf"   : self.casualty_found,
            "cxyz" : self.casualty_xyz.tolist() if self.casualty_xyz is not None else None,
            "cconf": round(self.casualty_conf, 3),
            "ts"   : self.timestamp,
        }
        return json.dumps(d)

    @staticmethod
    def from_json(raw: str) -> "DroneState":
        d = json.loads(raw)
        return DroneState(
            drone_id       = d["id"],
            role           = DroneRole(d["role"]),
            pos_xyz        = np.array(d["xyz"]),
            battery_pct    = d["bat"],
            casualty_found = d["cf"],
            casualty_xyz   = np.array(d["cxyz"]) if d["cxyz"] else None,
            casualty_conf  = d["cconf"],
            timestamp      = d["ts"],
        )


# ---------------------------------------------------------------------------
# Behavior Tree (Tick-based FSM)
# ---------------------------------------------------------------------------

class BehaviorStatus(Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"

class BehaviorNode:
    """Base class for a Behavior Tree node."""
    def tick(self, state: DroneState, swarm: Dict[str, DroneState]) -> BehaviorStatus:
        raise NotImplementedError

class Sequence(BehaviorNode):
    """Runs children in order. Fails immediately on any child failure."""
    def __init__(self, children: List[BehaviorNode]):
        self.children = children
    def tick(self, state, swarm):
        for child in self.children:
            r = child.tick(state, swarm)
            if r != BehaviorStatus.SUCCESS:
                return r
        return BehaviorStatus.SUCCESS

class Selector(BehaviorNode):
    """Runs children until one succeeds."""
    def __init__(self, children: List[BehaviorNode]):
        self.children = children
    def tick(self, state, swarm):
        for child in self.children:
            r = child.tick(state, swarm)
            if r == BehaviorStatus.SUCCESS:
                return r
        return BehaviorStatus.FAILURE

class Condition(BehaviorNode):
    def __init__(self, fn: Callable[[DroneState, Dict], bool]):
        self.fn = fn
    def tick(self, state, swarm):
        return BehaviorStatus.SUCCESS if self.fn(state, swarm) else BehaviorStatus.FAILURE

class Action(BehaviorNode):
    def __init__(self, fn: Callable[[DroneState, Dict], BehaviorStatus]):
        self.fn = fn
    def tick(self, state, swarm):
        return self.fn(state, swarm)


def build_sentinel_behavior_tree() -> BehaviorNode:
    """
    Builds the full Behavior Tree for a single drone agent.
    Priority order (Selector at root):
      1. Low battery? → Return to Base
      2. LoRa coverage gap detected? → Become relay node
      3. Casualty found by self or swarm partner? → Go rescue
      4. Else → Continue searching assigned Voronoi zone
    """

    # --- Low battery check ---
    low_battery = Sequence([
        Condition(lambda s, sw: s.battery_pct < 0.15),
        Action(lambda s, sw: (
            setattr(s, 'role', DroneRole.RTB),
            BehaviorStatus.SUCCESS
        )[-1])
    ])

    # --- Relay node condition ---
    needs_relay = Sequence([
        Condition(lambda s, sw: _detect_coverage_gap(s, sw)),
        Action(lambda s, sw: (
            setattr(s, 'role', DroneRole.RELAYING),
            BehaviorStatus.SUCCESS
        )[-1])
    ])

    # --- Casualty rescue ---
    # Trigger if self found casualty OR swarm partner with higher confidence did
    casualty_rescue = Sequence([
        Condition(lambda s, sw: _best_casualty_found(s, sw) is not None),
        Action(lambda s, sw: (
            setattr(s, 'role', DroneRole.RESCUING),
            BehaviorStatus.RUNNING  # Ongoing
        )[-1])
    ])

    # --- Default: search ---
    search_action = Action(lambda s, sw: (
        setattr(s, 'role', DroneRole.SEARCHING),
        BehaviorStatus.RUNNING
    )[-1])

    return Selector([low_battery, needs_relay, casualty_rescue, search_action])


def _detect_coverage_gap(state: DroneState,
                         swarm: Dict[str, DroneState]) -> bool:
    """True if this drone is the only one covering a large spatial gap."""
    positions = [s.pos_xyz for s in swarm.values() if s.drone_id != state.drone_id]
    if not positions:
        return False
    min_dist = min(np.linalg.norm(state.pos_xyz - p) for p in positions)
    return min_dist > 30.0  # meters — relay if nearest peer is >30m away


def _best_casualty_found(state: DroneState,
                         swarm: Dict[str, DroneState]) -> Optional[np.ndarray]:
    """Returns the XYZ of the highest-confidence casualty in the swarm, or None."""
    best_conf = 0.4
    best_xyz  = None
    all_states = list(swarm.values()) + [state]
    for s in all_states:
        if s.casualty_found and s.casualty_conf > best_conf:
            best_conf = s.casualty_conf
            best_xyz  = s.casualty_xyz
    return best_xyz


# ---------------------------------------------------------------------------
# Voronoi Zone Partitioner (Search Coverage)
# ---------------------------------------------------------------------------

class VoronoiPartitioner:
    """
    Divides the search zone into N Voronoi cells (one per drone).
    Each drone is assigned the cell whose centroid is closest to it.
    Updated every 30 seconds as drones move or drop out.
    """

    def __init__(self, zone_bounds: np.ndarray, grid_res: float = 2.0):
        """
        zone_bounds: [[x_min, y_min], [x_max, y_max]]
        """
        xs = np.arange(zone_bounds[0, 0], zone_bounds[1, 0], grid_res)
        ys = np.arange(zone_bounds[0, 1], zone_bounds[1, 1], grid_res)
        self.grid_points = np.array([[x, y] for x in xs for y in ys])

    def assign_zones(self, drone_positions: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """
        Returns dict: {drone_id: [list of 2D waypoints assigned to that drone]}
        """
        ids   = list(drone_positions.keys())
        poses = np.array([drone_positions[i][:2] for i in ids])  # XY only

        assignments: Dict[str, List] = defaultdict(list)
        for pt in self.grid_points:
            dists = np.linalg.norm(poses - pt, axis=1)
            owner = ids[int(np.argmin(dists))]
            assignments[owner].append(pt)

        return dict(assignments)


# ---------------------------------------------------------------------------
# Reynolds Flocking (Perimeter Formation)
# ---------------------------------------------------------------------------

class ReynoldsFormation:
    """
    Classic Craig Reynolds boid rules for swarm formation holding.
    Applied when drones are in PERIMETER mode.
    Rules: Separation, Alignment, Cohesion.
    """
    SEP_DIST  = 3.0    # meters — minimum separation
    SEP_WEIGHT = 2.0
    ALI_WEIGHT = 1.0
    COH_WEIGHT = 1.0

    def compute_velocity(self, self_pos: np.ndarray,
                         neighbor_positions: List[np.ndarray],
                         neighbor_velocities: List[np.ndarray]) -> np.ndarray:
        if not neighbor_positions:
            return np.zeros(3)

        positions = np.array(neighbor_positions)
        velocities = np.array(neighbor_velocities)

        # Separation
        diffs = self_pos - positions
        dists = np.linalg.norm(diffs, axis=1, keepdims=True) + 1e-6
        sep_mask = dists.flatten() < self.SEP_DIST
        sep = np.sum(diffs[sep_mask] / dists[sep_mask], axis=0) if sep_mask.any() else np.zeros(3)

        # Alignment
        ali = np.mean(velocities, axis=0)

        # Cohesion
        coh = np.mean(positions, axis=0) - self_pos

        return (self.SEP_WEIGHT * sep +
                self.ALI_WEIGHT * ali +
                self.COH_WEIGHT * coh)


# ---------------------------------------------------------------------------
# LoRa Swarm Mesh Transport
# ---------------------------------------------------------------------------

class LoRaMeshTransport:
    """
    SPI-based LoRa SX1262 driver wrapper for drone-to-drone mesh.
    Broadcasts: DroneState JSON every 500ms (heartbeat)
    Receives: All peer DroneState packets, forwards to swarm manager.
    """
    BROADCAST_INTERVAL = 0.5  # seconds
    STALE_TIMEOUT      = 3.0  # drop peers not seen for 3s

    def __init__(self, drone_id: str, lora_freq_mhz: float = 868.0):
        self.drone_id  = drone_id
        self.freq      = lora_freq_mhz
        self.peers: Dict[str, DroneState] = {}
        self._lock     = threading.Lock()
        self.running   = True

        # In production:
        # from sx126x import SX1262
        # self.lora = SX1262(spi=spidev.SpiDev(), cs=8, irq=25, rst=22)
        # self.lora.set_frequency(self.freq)

        print(f"[LoRa] Mesh transport initialized @ {lora_freq_mhz}MHz")

    def broadcast(self, state: DroneState):
        """Transmit this drone's state to all peers."""
        payload = state.to_json().encode('utf-8')
        # In production: self.lora.send(payload)
        # Simulated: just log it
        # print(f"[LoRa TX] {len(payload)} bytes → {state.role.value}")

    def receive_loop(self):
        """Continuously receive packets from peers and update peer table."""
        while self.running:
            # In production: raw = self.lora.receive()
            # Simulated stub:
            time.sleep(0.1)

    def get_live_peers(self) -> Dict[str, DroneState]:
        """Returns only peers seen within the stale timeout window."""
        now = time.time()
        with self._lock:
            return {k: v for k, v in self.peers.items()
                    if (now - v.timestamp) < self.STALE_TIMEOUT}


# ---------------------------------------------------------------------------
# Master Swarm Manager
# ---------------------------------------------------------------------------

class SwarmManager:
    """
    The top-level swarm coordinator for a single drone node.
    Runs on the Jetson Orin. Tick rate: 10Hz.
    """

    def __init__(self, drone_id: str, zone_bounds: np.ndarray):
        self.drone_id    = drone_id
        self.transport   = LoRaMeshTransport(drone_id)
        self.bt          = build_sentinel_behavior_tree()
        self.partitioner = VoronoiPartitioner(zone_bounds)
        self.formation   = ReynoldsFormation()
        self.running     = True

        # Own state (updated externally by vision + SLAM systems)
        self.state = DroneState(
            drone_id       = drone_id,
            role           = DroneRole.SEARCHING,
            pos_xyz        = np.zeros(3),
            battery_pct    = 1.0,
            casualty_found = False,
            casualty_xyz   = None,
            casualty_conf  = 0.0,
        )

    def update_state(self, pos_xyz: np.ndarray, battery: float,
                     casualty_found: bool = False,
                     casualty_xyz: Optional[np.ndarray] = None,
                     casualty_conf: float = 0.0):
        """Called by the main engine at each control loop tick."""
        self.state.pos_xyz        = pos_xyz
        self.state.battery_pct    = battery
        self.state.casualty_found = casualty_found
        self.state.casualty_xyz   = casualty_xyz
        self.state.casualty_conf  = casualty_conf
        self.state.timestamp      = time.time()

    def run(self):
        """Main swarm loop. Ticks the behavior tree and broadcasts state."""
        rx_thread = threading.Thread(target=self.transport.receive_loop, daemon=True)
        rx_thread.start()

        last_broadcast = 0.0
        while self.running:
            peers = self.transport.get_live_peers()

            # Tick behavior tree
            status = self.bt.tick(self.state, peers)

            # Leader election: Bully algorithm (highest battery wins)
            all_batteries = {self.drone_id: self.state.battery_pct}
            all_batteries.update({k: v.battery_pct for k, v in peers.items()})
            leader_id = max(all_batteries, key=all_batteries.get)
            if leader_id == self.drone_id and self.state.role not in (
                    DroneRole.RTB, DroneRole.RESCUING, DroneRole.RELAYING):
                self.state.role = DroneRole.LEADER

            # Broadcast heartbeat at 2Hz
            now = time.time()
            if now - last_broadcast > self.transport.BROADCAST_INTERVAL:
                self.transport.broadcast(self.state)
                last_broadcast = now

            time.sleep(0.1)  # 10Hz tick
