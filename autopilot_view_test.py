import carla
import time
import random

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()
blueprints = world.get_blueprint_library()

vehicle_bps = blueprints.filter("vehicle.*")
spawn_points = world.get_map().get_spawn_points()

if not spawn_points:
    raise RuntimeError("No spawn points found.")

vehicle = None

try:
    spawn_point = random.choice(spawn_points)
    vehicle_bp = random.choice(vehicle_bps)

    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    if vehicle is None:
        raise RuntimeError("Failed to spawn vehicle.")

    print("Spawned vehicle:", vehicle.type_id)
    print("Spawn location:", vehicle.get_location())

    # Move spectator camera above and behind the vehicle
    spectator = world.get_spectator()
    vehicle_transform = vehicle.get_transform()

    cam_location = vehicle_transform.location + carla.Location(x=-8, z=4)
    cam_rotation = carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw)

    spectator.set_transform(carla.Transform(cam_location, cam_rotation))

    tm = client.get_trafficmanager()
    vehicle.set_autopilot(True, tm.get_port())

    print("Autopilot enabled. Watch the simulator window for 60 seconds...")
    time.sleep(60)

finally:
    if vehicle is not None:
        vehicle.destroy()
        print("Vehicle destroyed.")