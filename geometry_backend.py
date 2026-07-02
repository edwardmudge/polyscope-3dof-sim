import polyscope as ps
import numpy as np

class VisContent:
    """
    [Backend Logic Layer]
    Responsibilities:
    1. Maintain geometric data (Mesh, Point Cloud)
    2. Execute geometric algorithms (Registration, Optimisation)
    3. Call Polyscope to register data (ps.register_...)
    """
    def __init__(self):
        # Sets manipulator initial parameters
        self.L1, self.L2, self.L3 = 4.0, 3.0, 2.0
        self.theta1, self.theta2, self.theta3 = 0.0, 0.0, 0.0
        self.ps_arm = None
        
        # Initialise the scene
        self.create_coordinate_frame()
        self.create_arm()

    def create_coordinate_frame(self, scale=1.0):
        """Initialise the base coordinate frame"""
        nodes = np.array([[0,0,0], [scale,0,0], [0,scale,0], [0,0,scale]])
        edges = np.array([[0,1], [0,2], [0,3]])
        
        ps_frame = ps.register_curve_network("Coordinate Frame", nodes, edges)
        
        # X=red, Y=green, Z=blue
        colors = np.array([[1,0,0], [0,1,0], [0,0,1]])
        ps_frame.add_color_quantity("axis_colors", colors, defined_on='edges', enabled=True)


    def forward_kinematics(self, theta1, theta2, theta3, L1, L2, L3):
        """
        Planar 3R mechanism forward kinematics.
        Angles in radians. Returns positions of joint1, joint2, and the
        end-effector. The base is implicitly at (0, 0).
        """
        # We can sum angles as the joints are all parallel to the z-axis
        a1 = theta1
        a2 = theta1 + theta2 
        a3 = theta1 + theta2 + theta3

        joint1 = np.array([L1 * np.cos(a1), L1 * np.sin(a1)])
        joint2 = joint1 + np.array([L2 * np.cos(a2), L2 * np.sin(a2)])
        end_effector = joint2 + np.array([L3 * np.cos(a3), L3 * np.sin(a3)])

        return joint1, joint2, end_effector


    def create_manipulator_geometry(self, theta1, theta2, theta3):
        base = np.array([0.0, 0.0])
        joint1, joint2, end_effector = self.forward_kinematics(theta1, theta2, theta3, self.L1, self.L2, self.L3)
        
        # Stack points into an Nx3 vector where z=0 for every node (planar)
        # Already arrays so no extra brackets needed
        points_2d = np.array([base, joint1, joint2, end_effector])

        # Creates a 4x1 array of zeros (shape function obtains the number of rows)
        z_column = np.zeros((points_2d.shape[0], 1))
        return np.hstack([points_2d, z_column])
    

    def create_arm(self):
        nodes = self.create_manipulator_geometry(self.theta1, self.theta2, self.theta3)
        self.ps_arm = ps.register_curve_network("Planar Arm", nodes, edges="line", radius = 0.02)
    
    
    def update_arm(self, theta1, theta2, theta3):
        self.theta1, self.theta2, self.theta3 = theta1, theta2, theta3
        nodes = self.create_manipulator_geometry(theta1, theta2, theta3)
        self.ps_arm.update_node_positions(nodes)


    def load_data(self):
        # This can later be replaced with trimesh.load()
        print("[Backend] Loading geometry...")
        
        
    def run_algorithm(self, param_a, param_b):
        """Example algorithm interface"""
        print(f"[Backend] Running Algorithm with params: {param_a}, {param_b}")
        # Once the computation is complete, call ps.register_... to update the display
