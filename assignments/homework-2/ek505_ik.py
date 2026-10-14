import os
import sys
import importlib.util
import numpy as np
import pickle as pkl
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox.models.URDF.Panda import Panda


def ikine_comparison_test(robot, targets, q0, results_filename, step_size=0.1, tol=1e-3,
                          max_iters=1000):
    """
    Run position-only inverse kinematics on a fixed set of target positions
    using both the gradient descent solver from Question code 1.1 and the
    Robotics Toolbox inverse kinematics method .ikine_LM. Save the results to
    a results_filename pickle file.

    targets is a nb_targets x 3 array, one target position per row.
    """

    q0 = np.array(q0, dtype=float)
    q_gradient = []
    q_rtb = []
    success_gradient = []
    success_rtb = []
    error_gradient = []
    error_rtb = []
    history_cost = []
    history_joint = []

    for p_des in targets:
        # Run the gradient descent solver
        q_sol, flag_status, h_cost, h_joint = ikine_position_gradient(
            robot, p_des, q0, step_size=step_size, tol=tol, max_iters=max_iters,
            joint_limits=True)

        # Run the Robotics Toolbox inverse kinematics solver. Its tol is applied
        # to E = 0.5*||e||^2, so convert it to match tol on ||e||.
        iksol = robot.ikine_LM(SE3.Trans(p_des), q0=q0, ilimit=max_iters, slimit=1,
                               tol=0.5 * tol**2, mask=[1, 1, 1, 0, 0, 0],
                               joint_limits=True)

        q_gradient.append(q_sol)
        q_rtb.append(iksol.q)
        success_gradient.append(flag_status)
        success_rtb.append(bool(iksol.success))
        error_gradient.append(np.linalg.norm(robot.fkine(q_sol).t - p_des))
        error_rtb.append(np.linalg.norm(robot.fkine(iksol.q).t - p_des))
        history_cost.append(h_cost)
        history_joint.append(h_joint)

    data = {
        "robot_name": robot.name,
        "targets": targets,
        "q0": q0,
        "step_size": step_size,
        "tol": tol,
        "max_iters": max_iters,
        "q_gradient": np.array(q_gradient),
        "q_rtb": np.array(q_rtb),
        "success_gradient": np.array(success_gradient),
        "success_rtb": np.array(success_rtb),
        "error_gradient": np.array(error_gradient),
        "error_rtb": np.array(error_rtb),
        "history_cost": history_cost,    # list of [nb_iterations] arrays, one per target
        "history_joint": history_joint,  # list of [nb_iterations x n] arrays, one per target
    }

    results_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), results_filename)
    with open(results_path, 'wb') as f:
        pkl.dump(data, f)


def ikine_position_gradient(robot, p_des, q0, step_size=0.1, tol=1e-3, max_iters=1000,
                            joint_limits=True):
    """
    Compute a joint configuration whose end-effector position is close to a
    desired target position by using gradient descent on the position error.

    The update is q <- q - step_size * Jp^T (p(q) - p_des), the gradient of
    0.5*||p(q) - p_des||^2. history_cost stores the position error norm.
    """

    q_sol = np.array(q0, dtype=float)
    history_cost = []
    history_joint = []
    if joint_limits:
        qlims = robot.qlim

    for _ in range(max_iters):
        err = robot.fkine(q_sol).t - p_des
        history_cost.append(np.linalg.norm(err))
        history_joint.append(q_sol.copy())
        if np.linalg.norm(err) < tol:
            break
        grad = robot.jacob0(q_sol)[:3, :].T @ err
        q_sol = q_sol - step_size * grad
        if joint_limits:
            q_sol = np.clip(q_sol, qlims[0, :], qlims[1, :])

    flag_status = bool(np.linalg.norm(robot.fkine(q_sol).t - p_des) < tol)

    return q_sol, flag_status, np.array(history_cost), np.array(history_joint)


def scara_targets(nb_targets):
    """
    Sample targets (nb_targets x 3) in a cylindrical shell around the SCARA base 
    with joint limits considered.
    """
    r_min, r_max = 0.7, 1.85
    z_min, z_max = 0.02, 0.23
    r = np.sqrt(r_min**2 + (r_max**2 - r_min**2) * np.random.rand(nb_targets))
    theta = 2 * np.pi * np.random.rand(nb_targets)
    z = z_min + (z_max - z_min) * np.random.rand(nb_targets)
    return np.column_stack([r * np.cos(theta), r * np.sin(theta), z])


def panda_targets(nb_targets):
    """
    Sample targets (nb_targets x 3) in a box in front of the Panda
    with joint limits considered.
    """
    box_min = np.array([0.3, -0.4, 0.1])
    box_max = np.array([0.7, 0.4, 0.7])
    return box_min + (box_max - box_min) * np.random.rand(nb_targets, 3)


def categorize(data):
    """
    Return boolean masks for the four success categories of Question report 1.3.
    """
    g = data["success_gradient"]
    r = data["success_rtb"]
    return {"both succeed": g & r, "gradient only": g & ~r,
            "RTB only": ~g & r, "both fail": ~g & ~r}


def plot_convergence(robot, data, idx):
    """
    Plot position error norm and joint values versus iteration for target idx,
    with the RTB final solution shown as horizontal reference lines.
    """
    h_cost = data["history_cost"][idx]
    h_joint = data["history_joint"][idx]
    q_rtb = data["q_rtb"][idx]

    fig, (ax_err, ax_q) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    ax_err.semilogy(h_cost, label="Gradient descent")
    ax_err.axhline(data["error_rtb"][idx], color='C1', linestyle='--', label="RTB final")
    ax_err.axhline(data["tol"], color='k', linestyle=':', label="tol")
    ax_err.set_ylabel("Position error norm [m]")
    ax_err.set_title(f"{robot.name}: target {idx} = {np.round(data['targets'][idx], 3)}")
    ax_err.legend()
    ax_err.grid()

    for j in range(robot.n):
        ax_q.plot(h_joint[:, j], color=f"C{j}", label=f"q{j + 1}")
        ax_q.axhline(q_rtb[j], color=f"C{j}", linestyle='--')
    ax_q.set_xlabel("Iteration")
    ax_q.set_ylabel("Joint value (solid: gradient, dashed: RTB)")
    ax_q.legend(ncol=robot.n)
    ax_q.grid()

    fig.tight_layout()
    fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             f"{robot.name.lower()}_convergence.png"))


def main():

    # Import SCARA model
    models_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               '..', 'homework-1', 'ek505_models.py')
    spec = importlib.util.spec_from_file_location("models", models_path)
    models = importlib.util.module_from_spec(spec)
    sys.modules["models"] = models
    spec.loader.exec_module(models)
    scara = models.scara

    # Import Panda model
    panda = Panda()

    # Assign variables for testing
    nb_targets = 20
    np.random.seed(0)

    # Run tests
    for robot, sample_targets, results_filename in [
            (scara, scara_targets, "scara_ik_results.pkl"),
            (panda, panda_targets, "panda_ik_results.pkl")]:
        q0 = np.zeros(robot.n)  # initial guess
        targets = sample_targets(nb_targets)
        ikine_comparison_test(robot, targets, q0, results_filename)

        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               results_filename), 'rb') as f:
            data = pkl.load(f)

        # Success table
        print(f"\n{robot.name}")
        categories = categorize(data)
        for name, mask in categories.items():
            print(f"  {name:14s} {100 * mask.sum() / nb_targets:5.1f}%")

        # Average final position error where both methods succeed
        both = categories["both succeed"]
        if both.any():
            print(f"  avg final error gradient: {data['error_gradient'][both].mean():.3e} m")
            print(f"  avg final error RTB:      {data['error_rtb'][both].mean():.3e} m")

        # Convergence plots for one representative target (first where both succeed)
        if both.any():
            plot_convergence(robot, data, int(np.flatnonzero(both)[0]))

    plt.show()


if __name__ == '__main__':
    main()
