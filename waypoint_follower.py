"""
waypoint_follower.py
--------------------
An agent that follows the CARLA global route planner using PID control.

Usage:
    python waypoint_follower.py [--host HOST] [--port PORT]
                                [--speed SPEED_KMH]
                                [--map MAP_NAME]
                                [--debug]

The agent:
  1. Spawns an ego vehicle at a random spawn point.
  2. Generates a random destination on the same map.
  3. Computes a route via the CARLA global-route planner.
  4. Follows the route using LongitudinalPID + LateralPID from
     pid_controller.py.
  5. Repeats from step 2 when the destination is reached.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time

try:
    import carla
except ImportError:
    sys.exit("ERROR: CARLA Python API not found.  pip install carla>=0.9.13")

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except ImportError:
    GlobalRoutePlanner = None  # handled at runtime

from pid_controller import LongitudinalPID, LateralPID


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _get_speed_kmh(vehicle) -> float:
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)


def _distance(loc_a, loc_b) -> float:
    return math.sqrt(
        (loc_a.x - loc_b.x) ** 2
        + (loc_a.y - loc_b.y) ** 2
        + (loc_a.z - loc_b.z) ** 2
    )


def _draw_waypoints(world, waypoints, life_time: float = 5.0) -> None:
    """Draw waypoints in the CARLA spectator view (debug mode)."""
    debug = world.debug
    for wp, _ in waypoints:
        debug.draw_point(
            wp.transform.location + carla.Location(z=0.5),
            size=0.05,
            color=carla.Color(0, 255, 0),
            life_time=life_time,
        )


# ---------------------------------------------------------------------------
# Agent
# ---------------------------------------------------------------------------
class WaypointFollower:
    """
    Simple PID-based waypoint following agent.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The ego vehicle actor.
    world : carla.World
    target_speed : float
        Cruise speed in km/h.
    sampling_resolution : float
        Metres between successive route waypoints.
    waypoint_reach_threshold : float
        Distance (m) at which a waypoint is considered reached.
    debug : bool
        Draw waypoints in the simulator.
    """

    def __init__(
        self,
        vehicle,
        world,
        target_speed: float = 30.0,
        sampling_resolution: float = 2.0,
        waypoint_reach_threshold: float = 3.0,
        debug: bool = False,
    ) -> None:
        self.vehicle = vehicle
        self.world = world
        self.target_speed = target_speed
        self.sampling_resolution = sampling_resolution
        self.reach_threshold = waypoint_reach_threshold
        self.debug = debug

        self._lon_pid = LongitudinalPID()
        self._lat_pid = LateralPID()
        self._route: list = []
        self._current_wp_index: int = 0

        if GlobalRoutePlanner is None:
            sys.exit(
                "ERROR: CARLA agents library not found on PYTHONPATH.\n"
                "Add PythonAPI/carla/agents to PYTHONPATH."
            )
        self._planner = GlobalRoutePlanner(
            world.get_map(), self.sampling_resolution
        )

    # ------------------------------------------------------------------
    def set_destination(self, destination: carla.Location) -> None:
        """Plan a route from the vehicle's current location to *destination*."""
        origin = self.vehicle.get_location()
        self._route = self._planner.trace_route(origin, destination)
        self._current_wp_index = 0
        self._lon_pid.reset()
        self._lat_pid.reset()
        print(
            f"[WaypointFollower] New route: {len(self._route)} waypoints "
            f"→ ({destination.x:.1f}, {destination.y:.1f})"
        )
        if self.debug:
            _draw_waypoints(self.world, self._route)

    # ------------------------------------------------------------------
    def run_step(self) -> bool:
        """
        Apply one control step.

        Returns
        -------
        bool
            True if the destination has been reached.
        """
        if not self._route or self._current_wp_index >= len(self._route):
            self.vehicle.apply_control(
                carla.VehicleControl(throttle=0.0, brake=1.0)
            )
            return True

        # -- advance waypoint index ------------------------------------
        current_location = self.vehicle.get_location()
        target_wp, _ = self._route[self._current_wp_index]
        while (
            _distance(current_location, target_wp.transform.location)
            < self.reach_threshold
            and self._current_wp_index < len(self._route) - 1
        ):
            self._current_wp_index += 1
            target_wp, _ = self._route[self._current_wp_index]

        # -- PID -------------------------------------------------------
        now = time.time()
        speed = _get_speed_kmh(self.vehicle)
        throttle, brake = self._lon_pid.run_step(self.target_speed, speed, now)
        steer = self._lat_pid.run_step(
            target_wp.transform.location,
            self.vehicle.get_transform(),
            now,
        )

        control = carla.VehicleControl(
            throttle=float(throttle),
            steer=float(steer),
            brake=float(brake),
        )
        self.vehicle.apply_control(control)

        # -- destination reached? --------------------------------------
        # Use 2× the normal waypoint threshold for the final waypoint because
        # the route planner may place the last waypoint slightly beyond the
        # road edge; a looser check prevents the agent from overshooting.
        last_wp, _ = self._route[-1]
        destination_threshold = self.reach_threshold * 2
        if (
            _distance(current_location, last_wp.transform.location)
            < destination_threshold
        ):
            return True
        return False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(
    host: str = "127.0.0.1",
    port: int = 2000,
    target_speed: float = 30.0,
    map_name: str | None = None,
    debug: bool = False,
) -> None:
    client = carla.Client(host, port)
    client.set_timeout(15.0)

    if map_name:
        world = client.load_world(map_name)
    else:
        world = client.get_world()

    print(f"[waypoint_follower] Map: {world.get_map().name}")

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        sys.exit("ERROR: No spawn points on this map.")

    # Spawn ego vehicle
    transform = random.choice(spawn_points)
    vehicle = world.spawn_actor(vehicle_bp, transform)
    print(f"[waypoint_follower] Spawned {vehicle.type_id}  id={vehicle.id}")

    agent = WaypointFollower(
        vehicle,
        world,
        target_speed=target_speed,
        debug=debug,
    )

    try:
        while True:
            destination = random.choice(spawn_points).location
            agent.set_destination(destination)

            while True:
                world.tick()
                reached = agent.run_step()
                if reached:
                    print("[waypoint_follower] Destination reached – choosing new one.")
                    break
                time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[waypoint_follower] Interrupted by user.")
    finally:
        vehicle.destroy()
        print("[waypoint_follower] Done.")


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CARLA Waypoint Follower")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument(
        "--speed", default=30.0, type=float, dest="target_speed",
        help="Cruise speed in km/h (default: 30)",
    )
    parser.add_argument("--map", default=None, dest="map_name")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    main(args.host, args.port, args.target_speed, args.map_name, args.debug)
