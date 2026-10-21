from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Configuration space limits
MAP_WIDTH = 12.0
MAP_HEIGHT = 10.0

# All obstacles radius and centers
OBSTACLE_RADIUS = 2.0
OBSTACLE_CENTERS = [
    (1.0, 5.0),
    (4.0, 5.0),
    (6.5, 2.5),
    (4.0, 9.5),
    (7.0, 6),
    (10.5, 3.75),
]


def is_state_valid(state):
    # TODO: a function that returns False if state falls in one of the spheres
    return True


def plan(coord_start, coord_goal, flag_simplify=True):
    # Define a 2-D state space and its bounds.
    # TODO: Define configuration space as RealVectorStateSpace
    space = None
    # TODO: Define bounds for this space
    bounds = None
    # Add bounds to the space
    space.setBounds(bounds)

    # Create start/goal states
    # TODO: create (allocate) state and set it
    start = None
    goal = None

    if not is_state_valid(start):
        print("Invalid start state")
        return None

    if not is_state_valid(goal):
        print("Invalid end state")
        return None

    # Create a SimpleSetup object
    setup = None
    setup.setStateValidityChecker(is_state_valid)
    setup.setStartAndGoalStates(start, goal)
    # Set panner
    # Other available planners are 'AORRTC','BFMT','BITstar','BKPIECE1',
    # 'FMT','InformedRRTstar','KPIECE1','LBKPIECE1','PRM','PRMstar',
    # 'RRT', 'RRTConnect', 'RRTstar', 'SORRTstar'
    setup.setPlanner(og.BITstar(setup.getSpaceInformation()))

    solved = setup.solve()

    if solved:
        if flag_simplify():
            setup.simplifySolution()
        states = setup.getSolutionPath().getStates()
    else:
        states = []
    return states


def plot_map_states(start, goal, states):

    # Plot the obstacles
    _, ax = plt.subplots()
    for cx, cy in OBSTACLE_CENTERS:
        ax.add_patch(Circle((cx, cy), OBSTACLE_RADIUS, color="tomato"))

    # Plot the path
    xs = [state[0] for state in states or []]
    ys = [state[1] for state in states or []]
    ax.plot(xs, ys, "b-o", label="planned path")
    ax.plot(start[0], start[1], "go", label="start")
    ax.plot(goal[0], goal[1], "ro", label="goal")

    # Setup limits and legend
    ax.set(
        xlim=(0, MAP_WIDTH),
        ylim=(0, MAP_HEIGHT),
        aspect="equal",
        xlabel="x",
        ylabel="y",
    )
    ax.grid(True)
    ax.legend()
    plt.show()


def main():
    # TODO: Try different coordinates
    coord_start = (1.0, 1.0)
    coord_goal = (1.0, 9.0)

    # TODO: Try with and without path simplification
    states = plan(coord_start, coord_goal, flag_simplify=True)
    plot_map_states(coord_start, coord_goal, states)


if __name__ == "__main__":
    main()
