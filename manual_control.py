"""
manual_control.py
-----------------
Keyboard-driven manual control for a CARLA vehicle using pygame.

Usage:
    python manual_control.py [--host HOST] [--port PORT]

Keyboard shortcuts:
    W / S       – throttle / brake
    A / D       – steer left / steer right
    Q           – toggle reverse gear
    Space       – handbrake
    R           – reset traffic lights
    ESC         – quit
"""

import argparse
import sys

try:
    import carla
except ImportError:
    sys.exit(
        "ERROR: CARLA Python API not found.\n"
        "Install it with:  pip install carla>=0.9.13"
    )

try:
    import pygame
    from pygame.locals import (
        K_a, K_d, K_s, K_w, K_q, K_r, K_SPACE, K_ESCAPE,
        KEYDOWN, QUIT,
    )
except ImportError:
    sys.exit("ERROR: pygame not found.  pip install pygame")

import numpy as np


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
FPS = 30
THROTTLE_STEP = 0.05
STEER_STEP = 0.04
BRAKE_STEP = 0.1
STEER_DECAY = 0.85  # auto-center multiplier per frame


# ---------------------------------------------------------------------------
# Sensor callbacks
# ---------------------------------------------------------------------------
def _rgb_callback(image, surface_holder: list) -> None:
    """Convert a CARLA RGB image to a pygame surface."""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    array = array[:, :, :3][:, :, ::-1]  # BGRA -> RGB
    surface_holder[0] = pygame.surfarray.make_surface(array.swapaxes(0, 1))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(host: str = "127.0.0.1", port: int = 2000) -> None:
    client = carla.Client(host, port)
    client.set_timeout(10.0)

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # -- spawn ego vehicle --------------------------------------------------
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        sys.exit("ERROR: No spawn points found on this map.")

    transform = spawn_points[0]
    vehicle = world.spawn_actor(vehicle_bp, transform)
    print(f"[manual_control] Spawned {vehicle.type_id}  id={vehicle.id}")

    # -- attach RGB camera --------------------------------------------------
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(WINDOW_WIDTH))
    camera_bp.set_attribute("image_size_y", str(WINDOW_HEIGHT))
    camera_bp.set_attribute("fov", "90")

    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    surface_holder: list = [None]
    camera.listen(lambda img: _rgb_callback(img, surface_holder))

    # -- pygame setup -------------------------------------------------------
    pygame.init()
    display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("CARLA Manual Control")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 18)

    throttle = 0.0
    steer = 0.0
    brake = 0.0
    reverse = False
    hand_brake = False

    try:
        while True:
            clock.tick(FPS)

            for event in pygame.event.get():
                if event.type == QUIT:
                    return
                if event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        return
                    if event.key == K_q:
                        reverse = not reverse
                        print(f"[manual_control] reverse={'ON' if reverse else 'OFF'}")
                    if event.key == K_r:
                        world.reset_all_traffic_lights()

            keys = pygame.key.get_pressed()
            if keys[K_w]:
                throttle = min(throttle + THROTTLE_STEP, 1.0)
                brake = 0.0
            elif keys[K_s]:
                brake = min(brake + BRAKE_STEP, 1.0)
                throttle = 0.0
            else:
                throttle = max(throttle - THROTTLE_STEP, 0.0)
                brake = max(brake - BRAKE_STEP, 0.0)

            if keys[K_a]:
                steer = max(steer - STEER_STEP, -1.0)
            elif keys[K_d]:
                steer = min(steer + STEER_STEP, 1.0)
            else:
                steer *= STEER_DECAY
                if abs(steer) < 0.01:
                    steer = 0.0

            hand_brake = bool(keys[K_SPACE])

            control = carla.VehicleControl(
                throttle=float(throttle),
                steer=float(steer),
                brake=float(brake),
                reverse=reverse,
                hand_brake=hand_brake,
            )
            vehicle.apply_control(control)

            # -- render ------------------------------------------------------
            if surface_holder[0] is not None:
                display.blit(surface_holder[0], (0, 0))

            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5

            hud_lines = [
                f"Speed : {speed_kmh:5.1f} km/h",
                f"Throttle: {throttle:.2f}  Brake: {brake:.2f}  Steer: {steer:.2f}",
                f"Reverse : {'ON' if reverse else 'OFF'}   Hand-brake: {'ON' if hand_brake else 'OFF'}",
                "W/S=throttle/brake  A/D=steer  Q=reverse  Space=handbrake  ESC=quit",
            ]
            for i, line in enumerate(hud_lines):
                surf = font.render(line, True, (255, 255, 255))
                display.blit(surf, (10, 10 + i * 22))

            pygame.display.flip()

    finally:
        print("[manual_control] Cleaning up …")
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        pygame.quit()


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CARLA Manual Control")
    parser.add_argument("--host", default="127.0.0.1", help="CARLA server host")
    parser.add_argument("--port", default=2000, type=int, help="CARLA server port")
    args = parser.parse_args()
    main(args.host, args.port)
