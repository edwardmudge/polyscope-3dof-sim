import numpy as np
import polyscope as ps
from geometry_backend import VisContent
from gui_panel import UI_Menu

def main():
    # 1. Initialise the Polyscope system
    ps.init()
    ps.set_program_name("Geometry Processing Lab")
    ps.set_ground_plane_mode("tile") # Tile mode
    ps.set_up_dir("z_up")

    # Fix the scene extents and ground plane so they don't drift as the arm
    # moves - by default Polyscope recomputes both from the live bounding
    # box of registered structures, which changes shape every time a joint
    # angle changes.
    ps.set_automatically_compute_scene_extents(False)
    low = np.array((-8.5, -0.6, -0.1))
    high = np.array((8.5, 0.6, 8.5))
    ps.set_bounding_box(low, high)
    ps.set_length_scale(float(np.linalg.norm(high - low)))

    ps.set_ground_plane_height_mode('manual')
    ps.set_ground_plane_height(0.)

    # 2. Instantiate the backend (Model)
    backend = VisContent()

    # 3. Instantiate the frontend (View) and inject the backend into it
    ui = UI_Menu(backend)

    # 4. Define the main loop callback
    # Polyscope is a C++ binding and requires a zero-argument function as the callback
    def callback_loop():
        ui.render()

    # 5. Register the callback and start
    ps.set_user_callback(callback_loop)
    ps.show()

if __name__ == "__main__":
    main()