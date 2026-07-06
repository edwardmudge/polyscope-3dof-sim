import polyscope as ps
import numpy as np
import trimesh

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
        self.L1, self.L2, self.L3 = 3.65, 2.65, 1.8268
        self.theta1, self.theta2, self.theta3 = 0.0, 0.0, 0.0

        # Attribute to check if mesh has been loaded
        self.loaded = False

        # Initialise the scene
        self.create_coordinate_frame()


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
        Planar 3R mechanism forward kinematics
        Angles in radians. Returns positions of joint1, joint2, and the
        end-effector. The base is implicitly at (0, 0)
        """
        # We can sum angles as the joints are all parallel to the z-axis
        a1 = theta1
        a2 = theta1 + theta2 
        a3 = theta1 + theta2 + theta3

        joint1 = np.array([L1 * np.cos(a1), L1 * np.sin(a1)])
        joint2 = joint1 + np.array([L2 * np.cos(a2), L2 * np.sin(a2)])
        end_effector = joint2 + np.array([L3 * np.cos(a3), L3 * np.sin(a3)])

        return joint1, joint2, end_effector


    def valid_configuration(self, theta1, theta2, theta3):
        """Reject configurations where any joint dips below the floor"""
        joint1, joint2, end_effector = self.forward_kinematics(theta1, theta2, theta3, self.L1, self.L2, self.L3)

        # Column 1 (or index 1) is the height part (sine function in forward kinematics)
        heights = [joint1[1], joint2[1], end_effector[1]]

        # Joint 1 height 0 corresponds to world Z = BASE_HEIGHT_M (post BASE_TRANSLATION),
        # so the true floor (world Z = 0) sits at height -BASE_HEIGHT_M
        for h in heights:
            if h < -BASE_HEIGHT_M:
                return False

        return True
    
    
    def update_arm(self, theta1, theta2, theta3):
        """Checks if a configuration is valid then updates the position of the arm"""
        if not self.loaded:
            return

        if not self.valid_configuration(theta1, theta2, theta3):
            return

        self.theta1, self.theta2, self.theta3 = theta1, theta2, theta3

        # Applies transforms to each joint with corrections, uses transform arithmetic
        T1 = homogenous_transform_z(theta1, self.L1)        
        T2 = T1 @ homogenous_transform_z(theta2, self.L2)    
        T3 = T2 @ homogenous_transform_z(theta3, self.L3)  

        self.ps_link1.set_transform(BASE_TRANSLATION @ REMAP @ rotation_z(theta1))
        self.ps_link2.set_transform(BASE_TRANSLATION @ REMAP @ T1 @ rotation_z(theta2))
        self.ps_link3.set_transform(BASE_TRANSLATION @ REMAP @ T2 @ rotation_z(theta3))


    def load_mesh(self, filepath, name, pivot_offset_mm=(0.0, 0.0, 0.0)):
        """Load one STL, shift its pivot to the local origin, and scale mm to metres"""
        mesh = trimesh.load(filepath)
        # Shifts the mesh so the physical joint pivot (measured in the STL's original CAD frame) becomes the new local origin (subtract offset)
        mesh.apply_translation(-np.array(pivot_offset_mm, dtype=float))
        mesh.apply_scale(0.001)  # SolidWorks STL export is in mm - simulation units are metres
        return ps.register_surface_mesh(name, mesh.vertices, mesh.faces)


    def load_data(self):
        """Register base and links, fix the base in place, and pose the arm at its zero-angle rest state"""
        # Offsets based on the way the parts were modelled in SolidWorks
        self.ps_base = self.load_mesh("assets/geometry/Base.STL", "Base", pivot_offset_mm=(378.75, 277.70, 378.75))
        self.ps_link1 = self.load_mesh("assets/geometry/Link 1.STL", "Link 1", pivot_offset_mm=(176.75, 176.75, 202.00))
        self.ps_link2 = self.load_mesh("assets/geometry/Link 2.STL", "Link 2", pivot_offset_mm=(176.75, 176.75, 126.25))
        self.ps_link3 = self.load_mesh("assets/geometry/Link 3.STL", "Link 3", pivot_offset_mm=(176.75, 176.75, 303.00))

        # Base is fixed and never transformed again
        self.ps_base.set_transform(BASE_TRANSLATION @ REMAP)

        # Set the default angles of all links and update loaded attribute
        self.loaded = True
        self.update_arm(0.0, 0.0, 0.0)

        print("[Backend] Loading geometry...")
        
        
    def run_algorithm(self, target_x, target_y, phi=0.0):
        """Geometric inverse kinematics algorithm interface (Craig, Section 4.4)"""
        result = solve_ik_angles(target_x, target_y, self.L1, self.L2, self.L3, phi)

        if result is None:
            print("[Backend] Target unreachable.")
            return

        theta1, theta2, theta3 = result
        self.update_arm(theta1, theta2, theta3)

        print(f"[Backend] Running inverse kinematics algorithm with params: x = {target_x}, z = {target_y} and phi = {phi}.")
        


def rotation_z(theta):
    """Rotation-only transform about local Z (no translation)"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ])


def homogenous_transform_z(theta, L):
    """
    Homogenous transform for a revolute joint, derived from Craig 
    Rotate theta about local Z, then translate L along the new local X (using sine and cosine functions)
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, L * c],
        [s,  c, 0, L * s],
        [0,  0, 1, 0    ],
        [0,  0, 0, 1    ],
    ])


# Base's pivot (Link 1 mount point) sits 270mm (0.27m) above its physical bottom face,
# so lifting the whole assembly by this amount puts Base's true bottom on the
# floor (world Z = 0) instead of the pivot itself
BASE_HEIGHT_M = 0.27
BASE_TRANSLATION = np.eye(4)
BASE_TRANSLATION[2, 3] = BASE_HEIGHT_M


# Fixed correction matrix: remaps the arm's own local (X, Y) plane onto the world (X, Z),
# since the arm maths was setup to sweep the vertical plane rather than lying flat on the ground 
# It is a rotation of the coordinate system about X (Craig 2.77)
REMAP = np.array([
    [1, 0,  0, 0],
    [0, 0, -1, 0],
    [0, 1,  0, 0],
    [0, 0,  0, 1],

])

def solve_ik_angles(target_x, target_y, L1, L2, L3, phi = 0.0):
        """
        Geometric IK for the planar 3R arm (Craig, Section 4.4, eq. 4.29-4.34,
        adapted to strip out L3's contribution to position before solving).
        """
        xw = target_x - L3 * np.cos(phi)
        yw = target_y - L3 * np.sin(phi)

        c2 = (xw**2 + yw**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if not -1.0 <= c2 <= 1.0:
            return None # Unreachable position
        
        theta2 = np.arctan2(np.sqrt(1-c2**2), c2)
        beta = np.arctan2(yw, xw)
        psi = np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
        theta1 = beta - psi
        theta3 = phi-theta1-theta2

        return theta1, theta2, theta3
