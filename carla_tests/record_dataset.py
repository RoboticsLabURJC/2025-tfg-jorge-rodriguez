import glob
import os
import sys

try:
    sys.path.append(glob.glob('/home/jrguezg/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import numpy as np
import pandas as pd


save_path = "_firstdataset"
img_path = os.path.join(save_path, "images")
os.makedirs(save_path, exist_ok=True)

frames = []
steerings = []


def camera_callback(image):
    steering = vehicle.get_control().steer
    frame = image.frame

    image.save_to_disk(os.path.join(img_path, f"{frame:06d}.png"))
    frames.append(frame)
    steerings.append(steering)


def save_data():

    df = pd.DataFrame({
        "frame": frames,
        "image": [f"{f:06d}.png" for f in frames],
        "steering": steerings
    })
    df.to_csv(os.path.join(save_path, "labels.csv"), index=False)


def main_fun():
    global vehicle

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    client.replay_file("dataset_record.log", 0.0, 0.0, 0)

    world.tick()
    time.sleep(1.0)

    vehicles = world.get_actors().filter("vehicle.*")
    vehicle = vehicles[0]

    blueprint_library = world.get_blueprint_library()
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", "800")
    camera_bp.set_attribute("image_size_y", "600")
    camera_bp.set_attribute("fov", "90")

    camera_transform = carla.Transform(
        carla.Location(x=1.5, z=1.7)
    )

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(camera_callback)

    try:
        while True:
            world.tick()
    except KeyboardInterrupt:
        print("[INFO] Finalizando...")
    finally:
        camera.stop()
        camera.destroy()
        save_data()

        # Restaurar settings
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        

if __name__ == '__main__':
    main_fun()