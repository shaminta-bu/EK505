import os
import sys
import importlib.util
import numpy as np
import pickle as pkl
from spatialmath import SE3
from roboticstoolbox.models.URDF.Panda import Panda


def ikine_comparison_test(robot, targets, q0, results_filename, step_size=0.1, tol=1e-3,
                          max_iters=1000):
    """
    Run position-only inverse kinematics on a fixed set of target positions
    using both the gradient descent solver from Question~ q:ik gradient and
    the Robotics Toolbox inverse kinematics method .ikine_LM. Save the
    results to a   results_filename pickle file.

    targets is a 3 x N array, one target position per column.
    """

    q_sol_all = []

    for p_des in targets.T:
        # Run the gradient descent solver
        q_sol, flag_status, history_cost, history_joint = ikine_position_gradient(
            robot, p_des, q0, step_size=step_size, tol=tol, max_iters=max_iters,
            joint_limits=True)

        # Run the Robotics Toolbox inverse kinematics solver (position only via mask,
        # a single search from q0 so it is comparable to gradient descent)
        iksol = robot.ikine_LM(SE3.Trans(p_des), q0=q0, ilimit=max_iters, slimit=1,
                               tol=tol, mask=[1, 1, 1, 0, 0, 0], joint_limits=True)

        # Compile results into dictionary
        q_sol_all.append({
            "target": p_des,
            "q0": q0,
            "q_ek505": q_sol,
            "q_rtb": iksol.q,
            "success_ek505": flag_status,
            "success_rtb": iksol.success,
            "residual_ek505": history_cost[-1] if len(history_cost) > 0 else None,
            "residual_rtb": iksol.residual,
            "history_ek505": {"cost": history_cost, "joint": history_joint},
            })

    results_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), results_filename)
    with open(results_path, 'wb') as f:
        pkl.dump(q_sol_all, f)

    return q_sol_all


def ikine_position_gradient(robot, p_des, q0, step_size=0.1, tol=1e-3, max_iters=1000,
                            joint_limits=False):
    """
    Compute a joint configuration whose end-effector position is close to a
    desired target position by using gradient descent on the position error.
    """

    b = p_des
    x = np.array(q0, dtype=float)
    history_cost = []
    history_joint = []
    flag_status = False
    if joint_limits:
        qlims = robot.qlim

    for _ in range(max_iters):
        f = robot.fkine(x).t
        err = f - b
        cost = 0.5 * np.linalg.norm(err)**2
        history_cost.append(cost)
        history_joint.append(x.copy())
        if np.linalg.norm(err) < tol:
            flag_status = True
            break
        grad = robot.jacob0(x)[:3, :].T @ err
        x = x - grad * step_size
        if joint_limits:
            x = np.clip(x, qlims[0, :], qlims[1, :])
    q_sol = x

    return q_sol, flag_status, history_cost, history_joint


def random_reachable_targets(robot, nb_targets, rng):
    """
    Generate target positions (3 x nb_targets) guaranteed to be reachable by
    taking the forward kinematics of random joint configurations within limits.
    """
    qlims = robot.qlim
    q_rand = rng.uniform(qlims[0, :], qlims[1, :], size=(nb_targets, robot.n))
    return np.column_stack([robot.fkine(q).t for q in q_rand])


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
    rng = np.random.default_rng(0)

    # Run tests
    for robot, results_filename in [(panda, "panda_results.pkl"),
                                    (scara, "scara_results.pkl")]:
        q0 = np.zeros(robot.n)  # initial guess
        p_targets = random_reachable_targets(robot, nb_targets, rng)  # 3 rows, 20 columns
        results = ikine_comparison_test(robot, p_targets, q0, results_filename)
        n_ek505 = sum(r["success_ek505"] for r in results)
        n_rtb = sum(r["success_rtb"] for r in results)
        print(f"{robot.name}: gradient {n_ek505}/{nb_targets}, ikine_LM {n_rtb}/{nb_targets}")


if __name__ == '__main__':
    main()
