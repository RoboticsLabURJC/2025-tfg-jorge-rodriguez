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

import torch
import torch.nn as nn
import torchvision.transforms as T
from PIL import Image
import carla
import random
import time
import numpy as np
import pandas as pd
import pygame

class PilotNet(nn.Module):
    def __init__(self):
        super().__init__()

        self.conv = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2),
            nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2),
            nn.ReLU(),
            nn.Conv2d(36, 48, 5, stride=2),
            nn.ReLU(),
            nn.Conv2d(48, 64, 3),
            nn.ReLU(),
            nn.Conv2d(64, 64, 3),
            nn.ReLU(),
        )

        with torch.no_grad():
            dummy = torch.zeros(1, 3, 66, 200)
            n_features = self.conv(dummy).view(1, -1).size(1)

        self.fc = nn.Sequential(
            nn.Flatten(),
            nn.Linear(n_features, 100),
            nn.ReLU(),
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, 10),
            nn.ReLU(),
            nn.Linear(10, 1)
        )

    def forward(self, x):
        x = self.conv(x)
        return self.fc(x)

device = torch.device("cpu")

model = PilotNet().to(device)
model.load_state_dict(torch.load("pilotnet_weights.pth", map_location=device))
model.eval()  # Muy importante para inferencia

preprocess = T.Compose([
    T.Resize((66, 200)),             # Igual que en entrenamiento
    T.ToTensor(),
    T.Normalize(mean=[0.5, 0.5, 0.5],
                std=[0.5, 0.5, 0.5])
])

latest_frame = None

def predict_steering(image: Image.Image) -> float:

    img_tensor = preprocess(image).unsqueeze(0).to(device)
    
    with torch.no_grad():
        steering = model(img_tensor)
    
    return steering.item()

def camera_callback(image, vehicle):
    global latest_frame

    array = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    array = array[:, :, :3]
    pil_image = Image.fromarray(array)

    steer_value = predict_steering(pil_image)
    control = carla.VehicleControl()
    control.throttle = 0.5
    control.steer = steer_value
    vehicle.apply_control(control)

    latest_frame = pil_image

def main_fun():
    global vehicle

    pygame.init()
    display_size = (800, 600)
    display = pygame.display.set_mode(display_size)
    pygame.display.set_caption("CARLA PilotNet Camera")

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    client.load_world_if_different('Town01')

    world = client.get_world()

    world.set_weather(carla.WeatherParameters.ClearNoon)

    bp = world.get_blueprint_library().filter('charger_2020')[0]
    spawn_points = world.get_map().get_spawn_points()
    vehicle = world.spawn_actor(bp, spawn_points[0])

    blueprint_library = world.get_blueprint_library()
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", "800")
    camera_bp.set_attribute("image_size_y", "600")
    camera_bp.set_attribute("fov", "90")

    camera_transform = carla.Transform(
        carla.Location(x=1.5, z=2.4)
    )

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: camera_callback(image, vehicle))

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt()
                
            if latest_frame is not None:
                img = latest_frame.convert("RGB")
                raw_str = img.tobytes()
                pg_image = pygame.image.fromstring(raw_str, img.size, "RGB")
                display.blit(pg_image, (0,0))
                pygame.display.flip()
            world.tick()
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("CTRL+C PRESSED - GOODBYE!")

    finally:
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        pygame.quit()
        

if __name__ == '__main__':
    main_fun()