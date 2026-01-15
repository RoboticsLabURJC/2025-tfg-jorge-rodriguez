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
import pygame

def main_fun():

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        world = client.get_world()

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("Not detected")
            return
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        bp = world.get_blueprint_library().filter('charger_2020')[0]
        spawn_points = world.get_map().get_spawn_points()
        vehicle = world.spawn_actor(bp, spawn_points[0])

        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')

        camera_transform = carla.Transform(
            carla.Location(x=1.5, y=0.0, z=1.4),
            carla.Rotation(pitch=0)
        )

        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

        client.start_recorder("dataset_record.log")

        clock = pygame.time.Clock()

        while True:

            spectator = world.get_spectator()

            vehicle_transform = vehicle.get_transform()
            location = vehicle_transform.location + carla.Location(x=0.0, z=3.0)
            rotation = carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw, roll=0)

            spectator.set_transform(carla.Transform(location, rotation))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            steer = joystick.get_axis(0)
            brake = max(0.0, (joystick.get_axis(2) + 1) / 2)
            throttle = max(0.0, (joystick.get_axis(5) + 1) / 2)

            control = carla.VehicleControl()
            control.steer = steer
            control.throttle = throttle
            control.brake = brake

            vehicle.apply_control(control)

            clock.tick(30)

    except KeyboardInterrupt:
        print("\nTerminando el programa")

    finally:
        client.stop_recorder()
        camera.destroy()
        vehicle.destroy()

if __name__ == '__main__':
    main_fun()