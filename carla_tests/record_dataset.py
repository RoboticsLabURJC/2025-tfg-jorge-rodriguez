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
os.makedirs(save_path, exist_ok=True)

frames = []
steerings = []


def camera_callback(image, vehicle):

    steering = vehicle.get_control().steer
    frame = image.frame


    image.save_to_disk(os.path.join(save_path, f"frame_{frame}.png"))
    frames.append(frame)
    steerings.append(steering)


def save_data():

    df = pd.DataFrame({
        "frame": frames,
        "steering": steerings
    })

    df.to_csv(os.path.join(save_path, "labels.csv"), index=False)


def main_fun():

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        world = client.get_world()

        bp = world.get_blueprint_library().filter('charger_2020')[0]
        spawn_points = world.get_map().get_spawn_points()
        vehicle = world.spawn_actor(bp, spawn_points[0])
        vehicle.set_autopilot(True)

        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')

        camera_transform = carla.Transform(
            carla.Location(x=1.5, y=0.0, z=1.4),
            carla.Rotation(pitch=0)
        )

        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

        camera.listen(lambda image: camera_callback(image, vehicle))

        start_time = time.time()
        while time.time() - start_time < 180:
            world.tick()

    finally:
        save_data()
        camera.destroy()
        vehicle.destroy()
        

if __name__ == '__main__':
    main_fun()