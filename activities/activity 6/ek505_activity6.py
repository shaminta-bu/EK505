import numpy as np
import cvxpy as cp

from ek505_activity_utilities import spline_plot


def acceleration_cost(C):
    """
    Acceleration cost for one cubic Bernstein spline.

    Parameters
    ----------
    C : cvxpy Variable or Expression, shape (4, 2)

    Returns
    -------
    cost : cvxpy Expression
    """
    # TODO: define a and b
    a = C[2] - 2 * C[1] + C[0]
    b = C[3] - 2 * C[2] + C[1]

    # TODO: return the CVXPY-friendly cost
    return cp.sum_squares(a) + cp.sum_squares(b)


def main():
    # Problem data

    # TODO: define start and goal
    p_start = np.array([0, 0])
    p_goal = np.array([3, 3])

    # Decision variables

    # TODO: define CVXPY variables for the two splines
    C_A = cp.Variable((4, 2))
    C_B = cp.Variable((4, 2))

    # Objective

    # TODO: define total cost
    cost = acceleration_cost(C_A) + acceleration_cost(C_B)

    # Constraints

    constraints = []

    # Boundary constraints

    # TODO: p_A(0) = p_start
    constraints += [
        C_A[0] == p_start
    ]

    # TODO: p_B(1) = p_goal
    constraints += [
        C_B[3] == p_goal    
    ]

    # Zero-velocity constraints

    # TODO: p_A_dot(0) = 0
    constraints += [
        3 * (C_A[1] - C_A[0]) == 0
    ]

    # TODO: p_B_dot(1) = 0
    constraints += [
        3 * (C_B[3] - C_B[2]) == 0
    ]

    # Continuity constraints

    # TODO: position continuity p_A(1) = p_B(0)
    constraints += [
        C_A[3] == C_B[0]
    ]

    # TODO: velocity continuity p_A_dot(1) = p_B_dot(0)
    constraints += [
        3 * (C_A[3] - C_A[2]) == 3 * (C_B[1] - C_B[0])
    ]

    # Corridor constraints for spline A:
    # R_A = {0 <= x <= 3, 0 <= y <= 1}

    # TODO: add x-bounds and y-bounds for C_A
    constraints += [
        C_A[:, 0] >= 0,  # x >= 0
        C_A[:, 0] <= 3,  # x <= 3
        C_A[:, 1] >= 0,  # y >= 0
        C_A[:, 1] <= 1   # y <= 1
    ]

    # Corridor constraints for spline B:
    # R_B = {2 <= x <= 3, 0 <= y <= 3}

    # TODO: add x-bounds and y-bounds for C_B
    constraints += [
        C_B[:, 0] >= 2,  # x >= 2
        C_B[:, 0] <= 3,  # x <= 3
        C_B[:, 1] >= 0,  # y >= 0
        C_B[:, 1] <= 3   # y <= 3
    ]

    # Solve

    # TODO: define and solve the CVXPY problem
    prob = cp.Problem(cp.Minimize(cost), constraints)

    prob.solve()

    # Print output

    # TODO: print status, optimal cost, C_A.value, and C_B.value
    print("Status:", prob.status)
    print("Optimal cost:", prob.value)
    print("C_A.value:\n", C_A.value)
    print("C_B.value:\n", C_B.value)

    # Plot

    # TODO: call the provided plotting function
    spline_plot(C_A.value, C_B.value)


if __name__ == "__main__":
    main()
