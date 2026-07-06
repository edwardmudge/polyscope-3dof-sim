import polyscope.imgui as psim
import numpy as np

class UI_Menu:
    """
    [Frontend Interaction Layer]
    Responsibilities:
    1. Draw ImGui controls (Button, Slider, Input)
    2. Collect user input
    3. Call methods on self.content to execute operations
    """
    def __init__(self, content_instance):
        # Dependency injection: the UI holds a reference to the backend
        self.content = content_instance
        
        # Internal UI state (View State)
        self.theta1_deg = 0.0
        self.theta2_deg = 0.0
        self.theta3_deg = 0.0

        # Target position for inverse kinematics
        self.target_position = np.array([5.0, 0.0])
        self.target_phi_deg = 0.0
        

    def render(self):
        """This function must be called by Polyscope every frame"""
        
        # 1. Panel title
        psim.TextUnformatted("Planar 3R Arm Control")
        psim.Separator()

        # 2. Data loading section
        if psim.TreeNode("I/O Operations"):
            if psim.Button("Load Manipulator Data"):
                self.content.load_data()
            psim.TreePop()

        # 3. Joint controls section
        if psim.TreeNode("Joint Angles"):
            changed, new_angles = psim.SliderFloat3("Angles (deg)", (self.theta1_deg, self.theta2_deg, self.theta3_deg), -180, 180)

            if changed:
                theta1_deg, theta2_deg, theta3_deg = new_angles
                theta1_rad, theta2_rad, theta3_rad =  np.radians(theta1_deg), np.radians(theta2_deg), np.radians(theta3_deg)

                # Checks if the configuration is valid
                if self.content.valid_configuration(theta1_rad, theta2_rad, theta3_rad):
                    self.theta1_deg, self.theta2_deg, self.theta3_deg = theta1_deg, theta2_deg, theta3_deg
                    self.content.update_arm(theta1_rad, theta2_rad, theta3_rad)
            
            psim.TreePop()


        # 4. Inverse kinematics section
        if psim.TreeNode("Inverse Kinematics"):
            changed, self.target_position = psim.InputFloat2("Target (X, Z)", self.target_position)
            _, self.target_phi_deg = psim.SliderFloat("Tool Angle (deg)", self.target_phi_deg, -180, 180)

            if psim.Button("Solve IK"):
                self.content.run_algorithm(
                    self.target_position[0], 
                    self.target_position[1],
                    np.radians(self.target_phi_deg)
                    )

            psim.TreePop()