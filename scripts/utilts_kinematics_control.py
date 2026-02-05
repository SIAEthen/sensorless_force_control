import cvxpy as cp
from cvxpy.settings import F
import numpy as np


def solve_first_HQP(J, umin, umax, rho1, rho2, solver=cp.OSQP):
    """
    Solve one generic layer:
        min rho * ||s||^2
        s.t. umin <= J zeta + s <= umax
             s >= 0
    """
    J = np.asarray(J)
    umin = np.asarray(umin).flatten()
    umax = np.asarray(umax).flatten()

    m, n = J.shape
    zeta = cp.Variable(n)
    s = cp.Variable(m)

    objective = cp.Minimize(rho1 * cp.sum_squares(zeta) + rho2 * cp.sum_squares(s))
    constraints = [
        J @ zeta + s >= umin,
        J @ zeta + s <= umax
    ]

    prob = cp.Problem(objective, constraints)
    prob.solve(solver=solver, warm_start=True, verbose=False)

    if prob.status not in ["optimal", "optimal_inaccurate"]:
        raise RuntimeError(f"QP layer failed: {prob.status}")

    return zeta.value, s.value, prob

def solve_second_HQP(
    J2, umin2, umax2, rho1, rho2,
    J1, umin1, umax1, s1_opt,
    solver=cp.OSQP
):
    """
    Second layer QP:
        min rho2 * ||s2||^2
        s.t.   umin2 <= J2*zeta + s2 <= umax2
               s2 >= 0

    Also add FIRST LAYER constraints but with FIXED s1_opt:
               umin1 <= J1*zeta + s1_opt <= umax1

    s1 is NOT optimized here (fixed).
    """

    J2 = np.asarray(J2)
    umin2 = np.asarray(umin2).flatten()
    umax2 = np.asarray(umax2).flatten()

    J1 = np.asarray(J1)
    umin1 = np.asarray(umin1).flatten()
    umax1 = np.asarray(umax1).flatten()
    s1_opt = np.asarray(s1_opt).flatten()

    m2, n = J2.shape
    m1, n1 = J1.shape
    assert n == n1

    # variables for second layer
    zeta = cp.Variable(n)
    s2 = cp.Variable(m2)

    # objective
    objective = cp.Minimize(rho1 * cp.sum_squares(zeta) + rho2 * cp.sum_squares(s2))

    constraints = []

    # Second-layer task
    constraints += [
        J2 @ zeta + s2 >= umin2,
        J2 @ zeta + s2 <= umax2
    ]

    # -------- Add FIRST LAYER constraints (with fixed s1_opt) -------
    constraints += [
        J1 @ zeta + s1_opt >= umin1,
        J1 @ zeta + s1_opt <= umax1
    ]
    # --------------------------------------------------------------

    prob = cp.Problem(objective, constraints)
    prob.solve(solver=solver, warm_start=True, verbose=False)

    if prob.status not in ["optimal", "optimal_inaccurate"]:
        raise RuntimeError(f"Second layer QP failed: {prob.status}")

    return zeta.value, s2.value, prob
def solve_third_HQP(
    J3, umin3, umax3, rho1, rho2,
    J2, umin2, umax2, s2_opt,
    J1, umin1, umax1, s1_opt,
    solver=cp.OSQP
):
    """
    Second layer QP:
        min rho2 * ||s2||^2
        s.t.   umin2 <= J2*zeta + s2 <= umax2
               s2 >= 0

    Also add FIRST LAYER constraints but with FIXED s1_opt:
               umin1 <= J1*zeta + s1_opt <= umax1

    s1 is NOT optimized here (fixed).
    """
    J3 = np.asarray(J3)
    umin3 = np.asarray(umin3).flatten()
    umax3 = np.asarray(umax3).flatten()

    J2 = np.asarray(J2)
    umin2 = np.asarray(umin2).flatten()
    umax2 = np.asarray(umax2).flatten()
    s2_opt = np.asarray(s2_opt).flatten()

    J1 = np.asarray(J1)
    umin1 = np.asarray(umin1).flatten()
    umax1 = np.asarray(umax1).flatten()
    s1_opt = np.asarray(s1_opt).flatten()

    m3, n3 = J3.shape
    m2, n2 = J2.shape
    m1, n1 = J1.shape
    assert n3 == n1

    # variables for second layer
    zeta = cp.Variable(n3)
    s3 = cp.Variable(m3)

    # objective
    objective = cp.Minimize(rho1 * cp.sum_squares(zeta) + rho2 * cp.sum_squares(s3))

    constraints = []

    constraints += [
        J3 @ zeta + s3 >= umin3,
        J3 @ zeta + s3 <= umax3
    ]
    # Second-layer task
    constraints += [
        J2 @ zeta + s2_opt >= umin2,
        J2 @ zeta + s2_opt <= umax2
    ]

    # -------- Add FIRST LAYER constraints (with fixed s1_opt) -------
    constraints += [
        J1 @ zeta + s1_opt >= umin1,
        J1 @ zeta + s1_opt <= umax1
    ]
    # --------------------------------------------------------------

    prob = cp.Problem(objective, constraints)
    prob.solve(solver=solver, warm_start=True, verbose=False)

    if prob.status not in ["optimal", "optimal_inaccurate"]:
        raise RuntimeError(f"Second layer QP failed: {prob.status}")

    return zeta.value, s3.value, prob

def solve_n_HQP(
    J_list,Umin,Umax,Rho1,Rho2,
    solver=cp.OSQP
):
    J_i = np.asarray(J_list[0])
    zeta_0 = 0 * J_i[0,:] 
    S_opt = []
    num_of_task = len(J_list)
   
    try:
        for i in range(num_of_task):
            J_i = np.asarray(J_list[i])
            umin_i = np.asarray(Umin[i]).flatten()
            umax_i = np.asarray(Umax[i]).flatten()
            rho1_i = Rho1[i]
            rho2_i = Rho2[i]
            m_i,n_i = J_i.shape
            zeta = cp.Variable(n_i)
            s_i = cp.Variable(m_i)
            assert rho1_i.shape[0]==n_i, "rho1 dimention error"
            assert rho2_i.shape[0]==m_i, "rho2 dimention error"             
            
            # objective
            objective = cp.Minimize( cp.sum_squares(rho1_i  @ zeta) +  cp.sum_squares(rho2_i @ s_i))
            constraints = []
            if i < 1:
                constraints +=  [np.asarray(J_list[i]) @ zeta + s_i >= umin_i,
                                    np.asarray(J_list[i]) @ zeta + s_i <= umax_i
                                    ]
            else:
                for j in range(i):
                    # used constraints with the optimized slack variables
                    constraints +=  [np.asarray(J_list[j]) @ zeta + np.asarray(S_opt[j]).flatten() >= np.asarray(Umin[j]).flatten(),
                                    np.asarray(J_list[j]) @ zeta +  np.asarray(S_opt[j]).flatten() <= np.asarray(Umax[j]).flatten()
                                    ]
                constraints +=  [J_i @ zeta + s_i >= umin_i,
                                J_i @ zeta +  s_i <= umax_i
                                    ]
            prob = cp.Problem(objective, constraints)
            prob.solve(solver=solver, polish=False, warm_start=True, verbose=False)

            if prob.status not in ["optimal", "optimal_inaccurate"]:
                raise RuntimeError(f"Second layer QP failed: {prob.status}")
            if(i==0):
                S_opt.append(0 * s_i.value)
            else:
                S_opt.append(s_i.value)
            if not (i<num_of_task-1):
                return zeta.value
    except AssertionError:
        print("Rho dimension wrong")
        return zeta_0
    except :
        print("No solution found, return zeros")
        return zeta_0

def example_run():
    n = 10
    m1 = 10
    m2 = 6
    rng = np.random.RandomState(0)

    # -------- FIRST LAYER (velocity bounds) ----------
    J1 = np.eye(n)
    vmin = -0.5 * np.ones(n)
    vmax =  0.5 * np.ones(n)
    rho = 1e3

    # zeta1, s1_opt, _ = solve_layer(J1, vmin, vmax, 0, rho)
    # print("First layer s1_opt:", s1_opt)

    # -------- SECOND LAYER (task J2*zeta = sigma) -----
    J2 = rng.randn(m2, n)
    sigma = rng.randn(m2)
    umin2 = sigma
    umax2 = sigma
    rho1 = 1.0
    rho2 = 1.0

    # zeta2, s2_opt, _ = solve_second_layer_with_first_constraints(
    #     J2, umin2, umax2, rho1, rho2,
    #     J1, vmin, vmax, s1_opt
    # )
    # print("Second layer zeta2:", zeta2)
    # print("Second layer s2_opt:", s2_opt)
    # print("Check J2*zeta2 =", J2 @ zeta2)
    rho1 = [np.eye(n),np.eye(n),np.eye(n)]
    rho2 = [np.eye(m1+1),np.eye(m2),np.eye(m2)]
    zeta2 = solve_n_HQP([J1,J2,J2],[vmin,umin2,umin2],[vmax,umax2,umax2],rho1,rho2)
    print(zeta2)
example_run()
# vmax = np.minimum(np.array([-0.43,0.1,-1.22,-3.14]),  np.array([1.54,1.45,0.645,3.14]))
# print(vmax)

