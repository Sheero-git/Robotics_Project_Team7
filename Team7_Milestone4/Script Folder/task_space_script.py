import numpy as np
from sympy import symbols, cos, sin, Matrix
import pandas as pd

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
    input_file = "C:/Users/youss/Desktop/Optimization/q_task.csv"

    #Read the CSV file into a DataFrame
    self.df = pd.read_csv(input_file, header=None)


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

            sim.wait(0.05)  # Small delay to control the loop frequency
        for column_index, column in self.df.items():
            # Access the joint values in the column
            self.target_positions = column.values  # This will be a list of 4 elements if each row has 4
            print(self.target_positions)
            for j in range(4):
                # Retrieve the current position for each joint
                current_position = sim.getJointPosition(getattr(self, f'joint{j+1}'))
            
                # Move the joint towards the target position
                if abs(current_position - self.target_positions[j]) > 0.01:  # Continue moving until close enough
                    new_position = current_position + self.speed * (self.target_positions[j] - current_position)
                    sim.setJointTargetPosition(getattr(self, f'joint{j+1}'), new_position)

                sim.wait(0.05)  # Small delay to control the loop frequency
        pass
    
def sysCall_cleanup():
    sim = require('sim')  # Ensure to require sim in cleanup as well
    # Get the position of the gripper relative to the base
    gripper_position = sim.getObjectPosition(self.joint4, self.joint1)
    print("Gripper Position:", gripper_position)

# Additional callback functions (if needed) can go here
