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


save_path = "_out"
os.makedirs(save_path, exist_ok=True)


def camera_callback(image, display_name="RGB CAMERA"):
    image.save_to_disk(os.path.join(save_path, f"frame_{image.frame:06d}.png"))

def lidar_callback(point_cloud):

    data = np.frombuffer(point_cloud.raw_data, dtype=np.float32)
    data = data.reshape((-1, 4))

    filename = os.path.join(save_path, f"lidar_{point_cloud.frame:06d}.txt")

    np.savetxt(filename, data, fmt="%.4f")

def imu_callback(data):
    print(f"acc -> {data.accelerometer}")
    print(f"gyro -> {data.gyroscope}")

def gnss_callback(position):
    print(f"latitude -> {position.latitude} - longitude -> {position.longitude} - altitude -> {position.altitude}")

def main_fun():

    actor_list = []
    start_time = time.time()

    try:

        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        world = client.get_world()
        
        bp = world.get_blueprint_library().filter('charger_2020')[0]
        vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
        actor_list.append(vehicle)

        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')

        camera_transform = carla.Transform(
            carla.Location(x=1.5, y=0.0, z=1.4),
            carla.Rotation(pitch=0)
        )

        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')

        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50')
        lidar_bp.set_attribute('rotation_frequency', '10')
        lidar_bp.set_attribute('points_per_second', '50000')

        lidar_transform = carla.Transform(carla.Location(x=0, z=2.5))

        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

        imu_bp = world.get_blueprint_library().find('sensor.other.imu')

        imu_bp.set_attribute('noise_accel_stddev_x', '0.0')
        imu_bp.set_attribute('noise_accel_stddev_y', '0.0')
        imu_bp.set_attribute('noise_accel_stddev_z', '0.0')
        imu_bp.set_attribute('noise_gyro_stddev_x', '0.0')
        imu_bp.set_attribute('noise_gyro_stddev_y', '0.0')
        imu_bp.set_attribute('noise_gyro_stddev_z', '0.0')

        imu_transform = carla.Transform(carla.Location(x=0, z=0))

        imu = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)

        gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')

        gnss_bp.set_attribute('noise_alt_bias', '0.0')
        gnss_bp.set_attribute('noise_alt_stddev', '0.0')
        gnss_bp.set_attribute('noise_lat_bias', '0.0')
        gnss_bp.set_attribute('noise_lat_stddev', '0.0')
        gnss_bp.set_attribute('noise_lon_bias', '0.0')
        gnss_bp.set_attribute('noise_lon_stddev', '0.0')
        gnss_bp.set_attribute('noise_seed', '0.0')
        gnss_bp.set_attribute('sensor_tick', '0.0')

        gnss_transform = carla.Transform(carla.Location(x=0, z=0))

        gnss = world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle)

        camera.listen(lambda data: camera_callback(data, "RGB CAMERA"))
        lidar.listen(lambda pointcloud: lidar_callback(pointcloud))
        imu.listen(lambda data: imu_callback(data))
        gnss.listen(lambda position: gnss_callback(position))

        while time.time() - start_time < 10:
            vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-0.2))
            time.sleep(0.05)

    finally:
        gnss.destroy()
        camera.destroy()
        lidar.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        imu.destroy()

if __name__ == '__main__':

    main_fun()