import os
import sys
import importlib.util
import numpy as np
import pickle as pkl
import swift
import spatialmath as sm
import spatialgeometry as sg
from roboticstoolbox.models.URDF.Panda import Panda
from ek505_ik import categorize

# RGBA colors for the success categories of Question report 1.3
CATEGORY_COLORS = {"both succeed": [0, 1, 0, 1],     # green
                   "gradient only": [0, 0, 1, 1],    # blue
                   "RTB only": [1, 0.5, 0, 1],       # orange
                   "both fail": [1, 0, 0, 1]}        # red


def visualize(sim, robot, results_filename, camera_position, camera_look_at,
              sphere_radius=0.05, dt=0.02, hold_time=2.0):
    """
    In the Swift scene sim, show the robot moving through the gradient descent
    iterations for the same representative target used in Question report 1.5,
    with all targets shown as spheres colored by success category.
    """
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), results_filename),
              'rb') as f:
        data = pkl.load(f)

    # Same representative target as plot_convergence (first where both succeed)
    categories = categorize(data)
    idx = int(np.flatnonzero(categories["both succeed"])[0])
    h_joint = data["history_joint"][idx]
    print(f"{robot.name}: animating target {idx} = {np.round(data['targets'][idx], 3)}"
          f" ({len(h_joint)} iterations)")

    # Add the robot at the initial configuration and frame it with the camera
    robot.q = h_joint[0]
    sim.add(robot)
    sim.set_camera_pose(camera_position, camera_look_at)

    # Add all targets as small spheres colored by category
    spheres = []
    for name, mask in categories.items():
        for p in data["targets"][mask]:
            spheres.append(sg.Sphere(sphere_radius, pose=sm.SE3(p),
                                     color=CATEGORY_COLORS[name]))
            sim.add(spheres[-1])

    # Hold the initial pose, step through the gradient descent joint history,
    # then hold the final pose
    for _ in range(int(hold_time / dt)):
        sim.step(dt)
    for q in h_joint:
        robot.q = q
        sim.step(dt)
    for _ in range(int(hold_time / dt)):
        sim.step(dt)

    # Clear the scene for the next robot
    sim.remove(robot)
    for sphere in spheres:
        sim.remove(sphere)


def main():

    # Import SCARA model
    models_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               '..', 'staples', 'ek505_models.py')
    spec = importlib.util.spec_from_file_location("models", models_path)
    models = importlib.util.module_from_spec(spec)
    sys.modules["models"] = models
    spec.loader.exec_module(models)

    # Create a Swift simulator shared by both robots
    sim = swift.Swift()
    sim.launch(realtime=True)
    input("Start the screen recording, then press Enter to animate...")

    # Video 1.1: SCARA (reach about 1.95 m), then video 1.2: Panda (reach under 1 m)
    visualize(sim, models.scara, "scara_ik_results.pkl",
              camera_position=[3.0, 3.0, 2.5], camera_look_at=[0, 0, 0.2],
              sphere_radius=0.05)
    visualize(sim, Panda(), "panda_ik_results.pkl",
              camera_position=[1.6, 1.4, 1.2], camera_look_at=[0.4, 0, 0.4],
              sphere_radius=0.02)

    # Keep the script alive so the local server doesn't shut down, then
    # disconnect from Swift so its server threads stop and the script exits
    input("Press Enter to close the simulator...")
    sim.close()


if __name__ == '__main__':
    main()
