import cvxpy as cp
import numpy as np

A = np.array([[1, 3, -2], [2, -2, 1], [-1, -3, 3]])
b = np.array([[4], [1], [2]])

x = cp.Variable((3, 1))
cost = cp.sum_squares(A @ x - b)
problem = cp.Problem(cp.Minimize(cost))
problem.solve()

print('Solution status:', problem.status)
print('Optimal value:', problem.value)