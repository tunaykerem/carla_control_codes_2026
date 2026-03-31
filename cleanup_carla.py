#!/usr/bin/env python3
import carla
import argparse

def main():
    argparser = argparse.ArgumentParser(description='CARLA Cleanup Script')
    argparser.add_argument('--host', default='127.0.0.1', help='IP of the host server')
    argparser.add_argument('--port', default=2000, type=int, help='TCP port to listen to')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    actors = world.get_actors()
    
    # Destroy all sensors
    sensors = [x for x in actors if 'sensor' in x.type_id]
    for s in sensors:
        print(f"Destroying sensor: {s.type_id} ({s.id})")
        s.destroy()

    # Destroy all vehicles (optional: you might want to keep some, but for total cleanup:)
    vehicles = [x for x in actors if 'vehicle' in x.type_id]
    for v in vehicles:
        print(f"Destroying vehicle: {v.type_id} ({v.id})")
        v.destroy()

    print(f"Cleanup finished. Removed {len(sensors)} sensors and {len(vehicles)} vehicles.")

if __name__ == '__main__':
    main()
