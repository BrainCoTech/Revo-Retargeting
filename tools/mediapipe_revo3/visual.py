"""Official Revo3 meshes viewed from the palm and the thumb side."""
import mujoco
import numpy as np


class RobotVisual:
    def __init__(self, path, joint_names):
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.addresses = [int(self.model.joint(name).qposadr[0]) for name in joint_names]
        self.renderer = mujoco.Renderer(self.model, height=360, width=310)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0, .015, .10]
        self.camera.distance = .42
        self.camera.elevation = 0
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0

    def render(self, q):
        self.data.qpos[self.addresses] = q
        mujoco.mj_forward(self.model, self.data)
        views = []
        # Official right hand: palm faces +X, thumb lies on the +Y side.
        # MuJoCo azimuth points camera-to-target, so these view from +X/+Y.
        for azimuth in [180, 270]:
            self.camera.azimuth = azimuth
            self.renderer.update_scene(self.data, camera=self.camera, scene_option=self.option)
            views.append(self.renderer.render()[:, :, ::-1].copy())
        return views

    def close(self):
        self.renderer.close()
