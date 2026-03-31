"""
spawn_vehicles.py
-----------------
Spawn a configurable number of NPC traffic vehicles in the CARLA world
and optionally set them on autopilot.

Usage:
    python spawn_vehicles.py [--host HOST] [--port PORT]
                             [--n COUNT]
                             [--tm-port TM_PORT]
                             [--no-autopilot]
                             [--filter BLUEPRINT_FILTER]

Press Ctrl-C to destroy all spawned vehicles and exit.
"""

from __future__ import annotations

import argparse
import random
import sys
import time

try:
    import carla
except ImportError:
    sys.exit("ERROR: CARLA Python API not found.  pip install carla>=0.9.13")


# ---------------------------------------------------------------------------
def main(
    host: str = "127.0.0.1",
    port: int = 2000,
    n_vehicles: int = 30,
    tm_port: int = 8000,
    autopilot: bool = True,
    blueprint_filter: str = "vehicle.*",
) -> None:
    client = carla.Client(host, port)
    client.set_timeout(10.0)

    world = client.get_world()
    traffic_manager = client.get_trafficmanager(tm_port)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    traffic_manager.set_synchronous_mode(False)

    blueprint_library = world.get_blueprint_library()
    blueprints = [
        bp for bp in blueprint_library.filter(blueprint_filter)
        if int(bp.get_attribute("number_of_wheels")) >= 4  # cars only
    ]
    if not blueprints:
        sys.exit(f"ERROR: No blueprints match filter '{blueprint_filter}'.")

    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)

    n_to_spawn = min(n_vehicles, len(spawn_points))
    if n_to_spawn < n_vehicles:
        print(
            f"[spawn_vehicles] WARNING: Only {len(spawn_points)} spawn points "
            f"available; spawning {n_to_spawn} vehicles."
        )

    spawned: list = []
    batch = []
    for i in range(n_to_spawn):
        bp = random.choice(blueprints)
        if bp.has_attribute("color"):
            color = random.choice(bp.get_attribute("color").recommended_values)
            bp.set_attribute("color", color)
        bp.set_attribute("role_name", "autopilot")

        batch.append(
            carla.command.SpawnActor(bp, spawn_points[i]).then(
                carla.command.SetAutopilot(
                    carla.command.FutureActor, autopilot, tm_port
                )
            )
        )

    results = client.apply_batch_sync(batch, False)
    for result in results:
        if result.error:
            print(f"[spawn_vehicles] Spawn error: {result.error}")
        else:
            spawned.append(result.actor_id)

    print(f"[spawn_vehicles] Spawned {len(spawned)} / {n_to_spawn} vehicles.")
    print("[spawn_vehicles] Press Ctrl-C to destroy vehicles and exit.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n[spawn_vehicles] Destroying {len(spawned)} vehicles …")
        client.apply_batch_sync(
            [carla.command.DestroyActor(vid) for vid in spawned], True
        )
        print("[spawn_vehicles] Done.")


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Spawn NPC vehicles in CARLA")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("-n", "--n", default=30, type=int, dest="n_vehicles",
                        help="Number of vehicles to spawn (default: 30)")
    parser.add_argument("--tm-port", default=8000, type=int,
                        help="Traffic Manager port (default: 8000)")
    parser.add_argument("--no-autopilot", action="store_false", dest="autopilot",
                        help="Do not enable autopilot on spawned vehicles")
    parser.add_argument("--filter", default="vehicle.*", dest="blueprint_filter",
                        help="Blueprint filter string (default: vehicle.*)")
    args = parser.parse_args()
    main(
        args.host, args.port, args.n_vehicles,
        args.tm_port, args.autopilot, args.blueprint_filter,
    )
