import numpy as np
from sympy import symbols, cos, sin, Matrix

q1_sym, q2_sym, q3_sym, q4_sym = symbols('q1_sym q2_sym q3_sym q4_sym')
l1, l2, l3, l4 = symbols('l1 l2 l3 l4')
l1_val= 0.044
l2_val = 0.140
l3_val = 0.134
l4_val = 0.09
def sysCall_init():
    sim = require('sim')

    # Retrieve handles for the joints (replace 'Joint1', 'Joint2', etc., with actual names from the scene)
    self.joint1 = sim.getObjectHandle('/Joint1')  # Replace with the actual joint name
    self.joint2 = sim.getObjectHandle('/Joint2')  # Replace with the actual joint name
    self.joint3 = sim.getObjectHandle('/Joint3')  # Replace with the actual joint name
    self.joint4 = sim.getObjectHandle('/Joint4')  # Replace with the actual joint name
    self.EE = sim.getObjectHandle('/Gripper_respondable')
    self.B = sim.getObjectHandle('/Joint1')
    # Define multiple sets of target positions (in radians) for the arm
    #self.target_positions = [0, 0, 0, 0]  # Target positions for each joint in radians

    # Define movement speed (adjust for faster or slower movement)
    self.speed = 0.05

    # Initial positions
    self.current_positions = [
        sim.getJointPosition(self.joint1),
        sim.getJointPosition(self.joint2),
        sim.getJointPosition(self.joint3),
        sim.getJointPosition(self.joint4)
    ]


def sysCall_thread():
    sim = require('sim')

    # Move the joints smoothly towards their target positions
    while not sim.getSimulationStopping():
        self.target_positions = [0.4001,-0.4318,1.3774,0.3837]        
        for i in range(4):
            # Retrieve the current position for each joint
            current_position = sim.getJointPosition(getattr(self, f'joint{i+1}'))
            
                # Move the joint towards the target position
            if abs(current_position - self.target_positions[i]) > 0.01:  # Continue moving until close enough
                new_position = current_position + self.speed * (self.target_positions[i] - current_position)
                sim.setJointTargetPosition(getattr(self, f'joint{i+1}'), new_position)

            #sim.wait(0.05)  # Small delay to control the loop frequency
        i = 0.05
        while i <= 10:
            q1_t = (0.4001 + (0.003948 * (i*i)) - (0.0002632 * (i*i*i)))
            q2_t = (-0.4318 + (0.02451 * (i*i)) - (0.001634 * (i*i*i)))
            q3_t = (1.3774 + (0.0144 * (i*i)) - (0.00096 * (i*i*i)))
            q4_t = (0.3837 + (0.002121 * (i*i)) - (0.0001414 * (i*i*i)))
            q_solution = [q1_t, q2_t, q3_t, q4_t]
            self.target_positions = q_solution
            i += 0.05
            for j in range(4):
                # Retrieve the current position for each joint
                current_position = sim.getJointPosition(getattr(self, f'joint{j+1}'))
            
                # Move the joint towards the target position
                if abs(current_position - self.target_positions[j]) > 0.01:  # Continue moving until close enough
                    new_position = current_position + self.speed * (self.target_positions[j] - current_position)
                    sim.setJointTargetPosition(getattr(self, f'joint{j+1}'), new_position)

                #sim.wait(0.05)  # Small delay to control the loop frequency
        break
        pass
    
def sysCall_cleanup():
    sim = require('sim')  # Ensure to require sim in cleanup as well
    # Get the position of the gripper relative to the base
    gripper_position = sim.getObjectPosition(self.joint4, self.joint1)
    print("Gripper Position:", gripper_position)

# Additional callback functions (if needed) can go here
