"""
Utilities for EK505 assignments
"""


def print_matrix(matrix, alignment="right"):
    """
    Prints a SymPy Matrix or NumPy array consistently by converting all elements
    to strings.

    Parameters:
    - matrix: A SymPy Matrix/Array or NumPy ndarray (1D or 2D).
    - alignment: 'right', 'left', or 'center' alignment for the columns.
    """
    # Ensure the input has a shape attribute (standard for NumPy and SymPy)
    if not hasattr(matrix, "shape"):
        raise TypeError(
            "Input must have a 'shape' attribute (e.g., SymPy Matrix or NumPy array)."
        )

    shape = matrix.shape
    if len(shape) == 1:
        rows, cols = 1, shape[0]
        is_1d = True
    elif len(shape) == 2:
        rows, cols = shape
        is_1d = False
    else:
        raise ValueError("Only 1D or 2D arrays/matrices are supported.")

    # Part 1: Convert all elements to strings and find the maximum length
    string_grid = []
    max_len = 0

    for r in range(rows):
        row_strings = []
        for c in range(cols):
            # Handle indexing differences between 1D and 2D
            val = matrix[c] if is_1d else matrix[r, c]

            # Convert to string and track the maximum length
            val_str = str(val)
            row_strings.append(val_str)
            max_len = max(max_len, len(val_str))

        string_grid.append(row_strings)

    # Part 2: Print the array using max_len as the column width
    for row in string_grid:
        formatted_row = []
        for s in row:
            # Align the string based on user preference
            if alignment == "left":
                formatted_row.append(s.ljust(max_len))
            elif alignment == "center":
                formatted_row.append(s.center(max_len))
            else:
                formatted_row.append(s.rjust(max_len))

        # Join elements with a couple of spaces and print
        row_str = "  ".join(formatted_row)
        print(f"  [ {row_str} ]")


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def spline_evaluate(C, num_points=100):
    """
    Evaluate a cubic Bernstein spline.

    Parameters
    ----------
    C : ndarray, shape (4, 2)
        Control points.
    num_points : int
        Number of points used to sample the spline.

    Returns
    -------
    P : ndarray, shape (num_points, 2)
        Points along the spline.
    """
    ts = np.linspace(0, 1, num_points)
    P = np.zeros((num_points, 2))

    for i, t in enumerate(ts):
        B = np.array(
            [
                (1 - t) ** 3,
                3 * t * (1 - t) ** 2,
                3 * t**2 * (1 - t),
                t**3,
            ]
        )
        P[i, :] = B @ C

    return P


def spline_plot(C_A, C_B):
    """
    Provided plotting function.

    Students do not need to modify this function.
    """
    num_points = 100

    P_A = spline_evaluate(C_A, num_points=num_points)
    P_B = spline_evaluate(C_B, num_points=num_points)

    t_A = np.linspace(0, 1, num_points)
    t_B = np.linspace(1, 2, num_points)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax_path = axes[0]
    ax_time = axes[1]

    rect_A = Rectangle(
        (0, 0),
        3,
        1,
        facecolor="tab:blue",
        alpha=0.15,
        edgecolor="tab:blue",
        linewidth=2,
    )
    rect_B = Rectangle(
        (2, 0),
        1,
        3,
        facecolor="tab:green",
        alpha=0.20,
        edgecolor="tab:green",
        linewidth=2,
    )

    ax_path.add_patch(rect_A)
    ax_path.add_patch(rect_B)

    ax_path.plot(P_A[:, 0], P_A[:, 1], "b-", linewidth=2, label="Spline A")
    ax_path.plot(P_B[:, 0], P_B[:, 1], "g-", linewidth=2, label="Spline B")

    ax_path.plot(
        C_A[:, 0],
        C_A[:, 1],
        "bo--",
        alpha=0.7,
        label="Control points A",
    )
    ax_path.plot(
        C_B[:, 0],
        C_B[:, 1],
        "go--",
        alpha=0.7,
        label="Control points B",
    )

    ax_path.plot(C_A[0, 0], C_A[0, 1], "ro", markersize=8, label="Start")
    ax_path.plot(C_B[-1, 0], C_B[-1, 1], "mo", markersize=8, label="Goal")

    ax_path.set_aspect("equal", adjustable="box")
    ax_path.set_xlim(-0.25, 3.25)
    ax_path.set_ylim(-0.25, 3.25)
    ax_path.set_xlabel("x")
    ax_path.set_ylabel("y")
    ax_path.grid(True)
    ax_path.legend(loc="best")
    ax_path.set_title("Path in the L-shaped corridor")

    ax_time.plot(t_A, P_A[:, 0], "b-", linewidth=2, label=r"$x_A(t)$")
    ax_time.plot(t_A, P_A[:, 1], "b--", linewidth=2, label=r"$y_A(t)$")
    ax_time.plot(t_B, P_B[:, 0], "g-", linewidth=2, label=r"$x_B(t)$")
    ax_time.plot(t_B, P_B[:, 1], "g--", linewidth=2, label=r"$y_B(t)$")

    ax_time.axvline(1, color="k", linestyle=":", linewidth=1)
    ax_time.text(1.02, 0.05, "switch", transform=ax_time.get_xaxis_transform())

    ax_time.set_xlabel("global time")
    ax_time.set_ylabel("coordinate value")
    ax_time.set_xlim(0, 2)
    ax_time.grid(True)
    ax_time.legend(loc="best")
    ax_time.set_title("Coordinates as functions of time")

    fig.tight_layout()
    plt.show()
