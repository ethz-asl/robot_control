import numpy as np

from copy import deepcopy
import quadprog
from control_barrier_functions.functions import *


def solve_qp(Q, q, G=None, h=None, A=None, b=None, debug_print=False):
    """
    Solves
      min 1\2 x^T Q x + q^T x
      s.t. G x <= h
      A x = b
    """
    qp_G = .5 * (q + Q.T)   # make sure P is symmetric
    qp_a = -q
    if A is not None:
        qp_C = -np.vstack([A, G]).T
        qp_b = -np.hstack([b, h])
        n_eq = A.shape[0]
    else:  # no equality constraint
        qp_C = -G.T
        qp_b = -h
        n_eq = 0

    if debug_print:
        print(f"""
1/2 x^T
{Q}
x + 
{q.transpose()} x 
s.t
{qp_C} x <= 
{qp_b}""")
    return quadprog.solve_qp(qp_G, qp_a, qp_C, qp_b, n_eq)[0]


class BarrierFunctionManager:
    def __init__(self):
        self.barrier_functions = []

    def add(self, bf : ZeroingBarrierFunction):
        self.barrier_functions.append(bf)

    def eval(self, x):
        h = self.barrier_functions[0].eval(x)
        for bf in self.barrier_functions[1:]:
            h = np.vstack([h, bf.eval(x)])
        return h

    def J(self, x):
        J = self.barrier_functions[0].J(x)
        for bf in self.barrier_functions[1:]:
            J = np.vstack([J, bf.J(x)])
        return J

    def solve(self, x, u):
        x = deepcopy(x)
        h = self.eval(x)
        G = -self.J(x)
        Q = np.eye(x.size)
        return solve_qp(Q=Q, q=u, G=G, h=h, debug_print=False)


if __name__ == "__main__":
    Q = np.eye(3, 3)
    q = np.zeros(3)
    G = -np.eye(3)
    h = np.array([1., -1., 2.])
    xs = solve_qp(Q, q, G, h)
    print(xs)
