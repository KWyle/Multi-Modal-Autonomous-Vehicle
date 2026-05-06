import carla
import random
import time
import numpy as np
import cv2
from ultralytics import YOLO

actors = []
latest_frame = None

left_ultra_distance = None
right_ultra_distance = None
left_ultra_detected = False
right_ultra_detected = False


def process_image(image):
    global latest_frame
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    latest_frame = array[:, :, :3].copy()  # BGRA -> BGR


def left_obstacle_callback(event):
    global left_ultra_distance, left_ultra_detected
    left_ultra_distance = event.distance
    left_ultra_detected = True


def right_obstacle_callback(event):
    global right_ultra_distance, right_ultra_detected
    right_ultra_distance = event.distance
    right_ultra_detected = True


def region_of_interest(img):
    height, width = img.shape[:2]
    mask = np.zeros_like(img)

    polygon = np.array([[
        (int(0.10 * width), height),
        (int(0.45 * width), int(0.60 * height)),
        (int(0.55 * width), int(0.60 * height)),
        (int(0.90 * width), height)
    ]], dtype=np.int32)

    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(img, mask)


def make_line_points(y1, y2, line_params):
    slope, intercept = line_params
    if abs(slope) < 1e-3:
        return None

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return (x1, y1, x2, y2)


def average_lane_line(lines, height):
    if not lines:
        return None

    slopes = []
    intercepts = []

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        if x2 == x1:
            continue

        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1

        slopes.append(slope)
        intercepts.append(intercept)

    if not slopes:
        return None

    slope_avg = np.mean(slopes)
    intercept_avg = np.mean(intercepts)

    y1 = height
    y2 = int(height * 0.60)
    return make_line_points(y1, y2, (slope_avg, intercept_avg))


def detect_lanes(frame):
    height, width = frame.shape[:2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    roi = region_of_interest(edges)

    lines = cv2.HoughLinesP(
        roi,
        rho=2,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=40,
        maxLineGap=80
    )

    left_lines = []
    right_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            if x2 == x1:
                continue

            slope = (y2 - y1) / (x2 - x1)

            if abs(slope) < 0.4:
                continue

            if slope < 0:
                left_lines.append(line)
            else:
                right_lines.append(line)

    left_lane = average_lane_line(left_lines, height)
    right_lane = average_lane_line(right_lines, height)

    lane_overlay = frame.copy()
    center_offset = None

    if left_lane is not None:
        x1, y1, x2, y2 = left_lane
        cv2.line(lane_overlay, (x1, y1), (x2, y2), (0, 255, 0), 5)

    if right_lane is not None:
        x1, y1, x2, y2 = right_lane
        cv2.line(lane_overlay, (x1, y1), (x2, y2), (0, 255, 0), 5)

    if left_lane is not None and right_lane is not None:
        lx1, _, _, _ = left_lane
        rx1, _, _, _ = right_lane
        lane_center_bottom = (lx1 + rx1) // 2
        image_center = width // 2
        center_offset = lane_center_bottom - image_center

        cv2.line(
            lane_overlay,
            (lane_center_bottom, height),
            (lane_center_bottom, int(height * 0.75)),
            (255, 0, 0),
            3
        )
        cv2.line(
            lane_overlay,
            (image_center, height),
            (image_center, int(height * 0.75)),
            (0, 0, 255),
            3
        )

    output = cv2.addWeighted(frame, 0.8, lane_overlay, 1.0, 0)
    return output, center_offset

def spawn_traffic(world, bp_lib, traffic_manager, actors, num_vehicles=25, max_attempts=200):
    spawn_points = world.get_map().get_spawn_points()
    vehicle_blueprints = bp_lib.filter("vehicle.*")

    spawned = 0
    attempts = 0

    while spawned < num_vehicles and attempts < max_attempts:
        spawn_point = random.choice(spawn_points)
        bp = random.choice(vehicle_blueprints)

        if bp.id == "vehicle.tesla.model3":
            attempts += 1
            continue

        npc = world.try_spawn_actor(bp, spawn_point)
        attempts += 1

        if npc is None:
            continue

        npc.set_autopilot(True, traffic_manager.get_port())
        traffic_manager.vehicle_percentage_speed_difference(npc, 20.0)
        traffic_manager.distance_to_leading_vehicle(npc, 2.0)

        actors.append(npc)
        spawned += 1

    print(f"Spawned {spawned} traffic vehicles after {attempts} attempts.")

def spawn_pedestrians(world, bp_lib, actors, num_walkers=40):
    walker_bps = bp_lib.filter("walker.pedestrian.*")

    spawn_points = []
    for _ in range(num_walkers):
        loc = world.get_random_location_from_navigation()
        if loc is not None:
            spawn_points.append(carla.Transform(loc))

    walkers = []
    controllers = []

    # Spawn walkers
    for spawn_point in spawn_points:
        bp = random.choice(walker_bps)
        walker = world.try_spawn_actor(bp, spawn_point)

        if walker is not None:
            walkers.append(walker)
            actors.append(walker)

    # Spawn controllers
    controller_bp = bp_lib.find("controller.ai.walker")

    for walker in walkers:
        controller = world.spawn_actor(controller_bp, carla.Transform(), walker)
        controllers.append(controller)
        actors.append(controller)

    # Start walking
    for controller in controllers:
        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())
        controller.set_max_speed(1 + random.random())  # 1–2 m/s

    print(f"Spawned {len(walkers)} pedestrians.")

def main():
    global latest_frame
    global left_ultra_distance, right_ultra_distance
    global left_ultra_detected, right_ultra_detected

    model = YOLO("yolov8n.pt")

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found.")

    vehicle_bp = random.choice(bp_lib.filter("vehicle.tesla.model3"))
    spawn_point = random.choice(spawn_points)

    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    if vehicle is None:
        raise RuntimeError("Failed to spawn vehicle.")
    actors.append(vehicle)

    tm = client.get_trafficmanager()
    tm.set_global_distance_to_leading_vehicle(2.5)
    tm.global_percentage_speed_difference(10.0)

    vehicle.set_autopilot(True, tm.get_port())
    spawn_traffic(world, bp_lib, tm, actors, num_vehicles=25)
    spawn_pedestrians(world, bp_lib, actors, num_walkers=40)

    # RGB camera
    camera_bp = bp_lib.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", "1280")
    camera_bp.set_attribute("image_size_y", "720")
    camera_bp.set_attribute("fov", "90")

    camera_transform = carla.Transform(
        carla.Location(x=1.5, z=2.4),
        carla.Rotation(pitch=-10)
    )

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actors.append(camera)
    camera.listen(process_image)

    # "Ultrasonic-style" front-left obstacle sensor
    obstacle_bp = bp_lib.find("sensor.other.obstacle")
    obstacle_bp.set_attribute("distance", "3.0")
    obstacle_bp.set_attribute("hit_radius", "0.25")
    obstacle_bp.set_attribute("only_dynamics", "false")
    obstacle_bp.set_attribute("sensor_tick", "0.05")

    left_sensor_transform = carla.Transform(
        carla.Location(x=2.2, y=-0.8, z=0.8),
        carla.Rotation(yaw=-25)
    )
    left_sensor = world.spawn_actor(
        obstacle_bp,
        left_sensor_transform,
        attach_to=vehicle
    )
    actors.append(left_sensor)
    left_sensor.listen(left_obstacle_callback)

    # "Ultrasonic-style" front-right obstacle sensor
    right_sensor_transform = carla.Transform(
        carla.Location(x=2.2, y=0.8, z=0.8),
        carla.Rotation(yaw=25)
    )
    right_sensor = world.spawn_actor(
        obstacle_bp,
        right_sensor_transform,
        attach_to=vehicle
    )
    actors.append(right_sensor)
    right_sensor.listen(right_obstacle_callback)

    spectator = world.get_spectator()

    frame_count = 0
    last_yolo_boxes = None
    prev_time = time.time()
    last_left_event_time = 0.0
    last_right_event_time = 0.0

    print("Lane + YOLO + side-front ultrasonic sensors running. Press Q to quit.")

    try:
        while True:
            
            # THIRD-PERSON VIEW
            vehicle_tf = vehicle.get_transform()
            vehicle_loc = vehicle_tf.location
            forward = vehicle_tf.get_forward_vector()

            spectator_location = carla.Location(
                x=vehicle_loc.x - forward.x * 10.0,
                y=vehicle_loc.y - forward.y * 10.0,
                z=vehicle_loc.z + 4.0
            )

            spectator_rotation = carla.Rotation(
                pitch=-15.0,
                yaw=vehicle_tf.rotation.yaw,
                roll=0.0
            )

            spectator.set_transform(
                carla.Transform(spectator_location, spectator_rotation)
            )

            '''
            # FIRST-PERSON VIEW
            vehicle_tf = vehicle.get_transform()
            vehicle_loc = vehicle_tf.location
            forward = vehicle_tf.get_forward_vector()

            spectator_location = carla.Location(
                x=vehicle_loc.x + forward.x * 2.5,
                y=vehicle_loc.y + forward.y * 2.5,
                z=vehicle_loc.z + 1.2
            )

            spectator_rotation = carla.Rotation(
                pitch=-5.0,
                yaw=vehicle_tf.rotation.yaw,
                roll=0.0
            )

            spectator.set_transform(
                carla.Transform(spectator_location, spectator_rotation)
            )
            '''

            if left_ultra_detected:
                last_left_event_time = time.time()
                left_ultra_detected = False

            if right_ultra_detected:
                last_right_event_time = time.time()
                right_ultra_detected = False

            left_active = (time.time() - last_left_event_time) < 0.3
            right_active = (time.time() - last_right_event_time) < 0.3

            if latest_frame is not None:
                frame = latest_frame.copy()

                lane_frame, offset = detect_lanes(frame)

                if frame_count % 3 == 0:
                    results = model.predict(
                        source=frame,
                        imgsz=480,
                        conf=0.35,
                        verbose=False
                    )
                    last_yolo_boxes = results[0]

                annotated = lane_frame
                if last_yolo_boxes is not None:
                    annotated = last_yolo_boxes.plot(img=lane_frame.copy())

                if offset is not None:
                    cv2.putText(
                        annotated,
                        f"Lane center offset: {offset:+d} px",
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2
                    )
                else:
                    cv2.putText(
                        annotated,
                        "Lane center offset: N/A",
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2
                    )

                left_text = (
                    f"Left ultrasonic: {left_ultra_distance:.2f} m"
                    if left_active and left_ultra_distance is not None
                    else "Left ultrasonic: clear"
                )
                right_text = (
                    f"Right ultrasonic: {right_ultra_distance:.2f} m"
                    if right_active and right_ultra_distance is not None
                    else "Right ultrasonic: clear"
                )

                cv2.putText(
                    annotated,
                    left_text,
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0) if left_active else (200, 200, 200),
                    2
                )
                cv2.putText(
                    annotated,
                    right_text,
                    (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0) if right_active else (200, 200, 200),
                    2
                )

                now = time.time()
                fps = 1.0 / max(now - prev_time, 1e-6)
                prev_time = now

                cv2.putText(
                    annotated,
                    f"FPS: {fps:.1f}",
                    (20, 135),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

                cv2.imshow("CARLA Lane + YOLO + Ultrasonic", annotated)
                frame_count += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            time.sleep(0.01)

    finally:
        cv2.destroyAllWindows()
        print("Cleaning up actors...")
        for actor in actors:
            if actor.is_alive:
                actor.destroy()


if __name__ == "__main__":
    main()