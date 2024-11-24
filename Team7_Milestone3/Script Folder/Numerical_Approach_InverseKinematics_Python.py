import numpy as np
from sympy import symbols, cos, sin, Matrix

# Define symbolic variables
q1_sym, q2_sym, q3_sym, q4_sym = symbols('q1_sym q2_sym q3_sym q4_sym')
l1, l2, l3, l4 = symbols('l1 l2 l3 l4')

# Desired position
Xd = -0.126644546635844
Yd = -0.0535444555078038
Zd = 0.271105807530105

# Link lengths
l1_val= 0.044
l2_val = 0.140
l3_val = 0.134
l4_val = 0.0

# Initial guesses for joint angles
q1_init = 0.1
q2_init = 0.1
q3_init = 0.1
q4_init = 1

def forward_kinematics(q, l1, l2, l3, l4):
    """
    Forward kinematics computation.

    :param q: Joint angles [q1, q2, q3, q4]
    :param l2: Link 2 length
    :param l3: Link 3 length
    :param l4: Link 4 length
    :return: End-effector position (X, Y, Z)
    """
    q1, q2, q3, q4 = q

    X_fk =l2*cos(q1)*cos(q2 + 1.5707963267949) - l3*sin(q3)*sin(q2 + 1.5707963267949)*cos(q1) + l3*cos(q1)*cos(q3)*cos(q2 + 1.5707963267949) + l4*(-sin(q3)*sin(q2 + 1.5707963267949)*cos(q1) + cos(q1)*cos(q3)*cos(q2 + 1.5707963267949))*cos(q4) + l4*(-sin(q3)*cos(q1)*cos(q2 + 1.5707963267949) - sin(q2 + 1.5707963267949)*cos(q1)*cos(q3))*sin(q4)
    
    
    Y_fk = l2*sin(q1)*cos(q2 + 1.5707963267949) - l3*sin(q1)*sin(q3)*sin(q2 + 1.5707963267949) + l3*sin(q1)*cos(q3)*cos(q2 + 1.5707963267949) + l4*(-sin(q1)*sin(q3)*sin(q2 + 1.5707963267949) + sin(q1)*cos(q3)*cos(q2 + 1.5707963267949))*cos(q4) + l4*(-sin(q1)*sin(q3)*cos(q2 + 1.5707963267949) - sin(q1)*sin(q2 + 1.5707963267949)*cos(q3))*sin(q4)
    
    
    Z_fk = l1 + l2*sin(q2 + 1.5707963267949) + l3*sin(q3)*cos(q2 + 1.5707963267949) + l3*sin(q2 + 1.5707963267949)*cos(q3) + l4*(-sin(q3)*sin(q2 + 1.5707963267949) + cos(q3)*cos(q2 + 1.5707963267949))*sin(q4) + l4*(sin(q3)*cos(q2 + 1.5707963267949) + sin(q2 + 1.5707963267949)*cos(q3))*cos(q4)


    return np.array([float(X_fk), float(Y_fk), float(Z_fk)])


def jacobian_matrix(q, l1, l2, l3, l4):
    """
    Compute the Jacobian matrix.

    :param q: Joint angles [q1, q2, q3, q4]
    :param l2: Link 2 length
    :param l3: Link 3 length
    :param l4: Link 4 length
    :return: Jacobian matrix
    """
    # Symbolic forward kinematics
    X_fk = l2*cos(q1_sym)*cos(q2_sym + 1.5707963267949) - l3*sin(q3_sym)*sin(q2_sym + 1.5707963267949)*cos(q1_sym) + l3*cos(q1_sym)*cos(q3_sym)*cos(q2_sym + 1.5707963267949) + l4*(-sin(q3_sym)*sin(q2_sym + 1.5707963267949)*cos(q1_sym) + cos(q1_sym)*cos(q3_sym)*cos(q2_sym + 1.5707963267949))*cos(q4_sym) + l4*(-sin(q3_sym)*cos(q1_sym)*cos(q2_sym + 1.5707963267949) - sin(q2_sym + 1.5707963267949)*cos(q1_sym)*cos(q3_sym))*sin(q4_sym)
    
    
    Y_fk = l2*sin(q1_sym)*cos(q2_sym + 1.5707963267949) - l3*sin(q1_sym)*sin(q3_sym)*sin(q2_sym + 1.5707963267949) + l3*sin(q1_sym)*cos(q3_sym)*cos(q2_sym + 1.5707963267949) + l4*(-sin(q1_sym)*sin(q3_sym)*sin(q2_sym + 1.5707963267949) + sin(q1_sym)*cos(q3_sym)*cos(q2_sym + 1.5707963267949))*cos(q4_sym) + l4*(-sin(q1_sym)*sin(q3_sym)*cos(q2_sym + 1.5707963267949) - sin(q1_sym)*sin(q2_sym + 1.5707963267949)*cos(q3_sym))*sin(q4_sym)
    
    
    Z_fk = l1 + l2*sin(q2_sym + 1.5707963267949) + l3*sin(q3_sym)*cos(q2_sym + 1.5707963267949) + l3*sin(q2_sym + 1.5707963267949)*cos(q3_sym) + l4*(-sin(q3_sym)*sin(q2_sym + 1.5707963267949) + cos(q3_sym)*cos(q2_sym + 1.5707963267949))*sin(q4_sym) + l4*(sin(q3_sym)*cos(q2_sym + 1.5707963267949) + sin(q2_sym + 1.5707963267949)*cos(q3_sym))*cos(q4_sym)


    # Symbolic Jacobian matrix
    J_sym = Matrix([X_fk, Y_fk, Z_fk]).jacobian([q1_sym, q2_sym, q3_sym, q4_sym])

    # Substitute numerical values
    J = J_sym.subs({q1_sym: q[0], q2_sym: q[1], q3_sym: q[2], q4_sym: q[3]})
    J = J.subs({l1: l1_val, l2: l2_val, l3: l3_val, l4: l4_val})
    return np.array(J.evalf(), dtype=float)


def inverse_kinematics_nr(Xd, Yd, Zd, l1 , l2, l3, l4, q_init):
    """
    Newton-Raphson inverse kinematics solver.

    :param Xd: Desired X position
    :param Yd: Desired Y position
    :param Zd: Desired Z position
    :param l2: Link 2 length
    :param l3: Link 3 length
    :param l4: Link 4 length
    :param q_init: Initial guesses for joint angles
    :return: Joint angles [q1, q2, q3, q4]
    """
    max_iter = 100
    tol = 1e-6
    q = np.array(q_init, dtype=float)

    for _ in range(max_iter):
        # Forward kinematics
        fk = forward_kinematics(q, l1, l2, l3, l4)

        # Compute error
        error = np.array([Xd, Yd, Zd]) - fk

        # Check convergence
        if np.linalg.norm(error) < tol:
            return q

        # Jacobian matrix
        J = jacobian_matrix(q, l1 , l2, l3, l4)
        

        # Newton-Raphson step
        delta_q = np.linalg.pinv(J) @ error
        q += delta_q

    raise ValueError("Inverse kinematics did not converge")


# Solve inverse kinematics
q_solution = inverse_kinematics_nr(Xd, Yd, Zd, l1_val, l2_val, l3_val, l4_val, [q1_init, q2_init, q3_init, q4_init])
print(f'Solution: q1={np.mod(q_solution[0], 2 * np.pi):.4f}, q2={np.mod(q_solution[1], 2 * np.pi):.4f}, q3={np.mod(q_solution[2], 2 * np.pi):.4f}, q4={np.mod(q_solution[3], 2 * np.pi):.4f}')
