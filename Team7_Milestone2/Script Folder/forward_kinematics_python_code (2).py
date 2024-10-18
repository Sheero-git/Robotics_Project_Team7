import sympy as sp
import numpy
import math
# Define the transformation matrix function using DH parameters
def transformation_func(theta, d, a, alpha):
    T = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                   [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                   [0, sp.sin(alpha), sp.cos(alpha), d],
                   [0, 0, 0, 1]])
    return T

# Define the forward kinematics function
def forward_kinematics_func():
    # Define symbolic variables for the angles and lengths
    #q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4')

    q1, q2, q3, q4 = 0,0,0,0

    #l1, l2, l3, l4 = sp.symbols('l1 l2 l3 l4') # Assign constant values for lengths
    l1, l2, l3, l4=0.044, 0.140, 0.134, 0
    # Create the transformation matrices using the DH parameters
    A = transformation_func(q1, l1, 0, sp.pi/2)
    B = transformation_func(math.pi/2+q2, 0, l2, 0)
    C = transformation_func(q3, 0, l3, 0)
    D = transformation_func(q4, 0, l4, 0)
    
    # Multiply the matrices to get the overall transformation matrix
    result = A * B * C * D
    print(result)
    # Extract the position of the end effector (X, Y, Z)
    last_column = result[:, -1]
    X = last_column[0]
    Y = last_column[1]
    Z = last_column[2]
    print("X=",X)
    print("Y",Y)
    print("Z",Z)
    return sp.Matrix([X, Y, Z])

# Call the forward kinematics function to get the end effector position
end_effector_position = forward_kinematics_func()
#print(end_effector_position)