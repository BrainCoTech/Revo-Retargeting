"""MuJoCo position-servo validation, independent of any hardware transport.

The URDF supplies geometry, mass, inertia and position ranges. Servo effort
saturation is an explicit simulation setting, defaulting to 0.35 N m. Official
URDF effort/velocity metadata are zero and are not actuator settings. Simulation
effort is neither a validated physical-hand limit nor a motor-current conversion.
Contacts remain disabled; this model cannot validate collision-free grasps.

The controller is a modest 1 N m/rad position gain with a velocity damping
estimate based on each diagonal entry of the zero-pose mass matrix. Joint
damping adds 0.002 N m s/rad. Masses and inertias are left unchanged. The
implicitfast integrator treats velocity feedback implicitly. See:
https://mujoco.readthedocs.io/en/latest/XMLreference.html#actuator-position

Soft limit parameters are explicit simulation choices, not measured hard-stop
compliance: 4 ms time constant, 1 mrad activation margin, and high impedance.
Actual excursions are reported, with 1 mrad used as a numerical/soft-stop
tolerance. Measured qpos is never clipped or overwritten outside reset().
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


class ServoSimulation:
    """Fixed-base, gravity-on, contact-disabled 21-joint simulation.

    ``step`` holds its position reference over ``dt`` and integrates at most
    ``timestep`` per physics step, using a smaller equal substep if needed.
    Inputs/outputs follow ``joint_names`` in the supplied RobotModel's order.
    The public ``model`` and ``data`` may be passed to the MuJoCo viewer;
    ``q_indices`` maps the named vector to MuJoCo qpos. ``report`` is a fresh
    JSON-serializable statistics dictionary accumulated since the last reset.
    """

    POSITION_GAIN = 1.0
    JOINT_DAMPING = 0.002
    SOFT_LIMIT_TOLERANCE = 0.001

    def __init__(self, robot, timestep=0.002, effort_limit_nm=0.35):
        self.timestep = self._positive_time(timestep, "timestep")
        if self.timestep > 0.002:
            raise ValueError("Physics timestep must not exceed 0.002 seconds")
        self.joint_names = tuple(robot.joint_names)
        self.lower = np.asarray(robot.lower, dtype=float).copy()
        self.upper = np.asarray(robot.upper, dtype=float).copy()
        if len(self.joint_names) != 21:
            raise ValueError("ServoSimulation expects the 21-joint Revo3 model")

        urdf = ET.parse(robot.urdf_path).getroot()
        self.urdf_effort_limit = np.array([
            float(urdf.find(f"joint[@name='{name}']/limit").attrib["effort"])
            for name in self.joint_names
        ])
        if np.any(~np.isfinite(self.urdf_effort_limit)) or np.any(self.urdf_effort_limit < 0):
            raise ValueError("URDF effort metadata must be finite and nonnegative")
        self.effort_limit = np.full(21, self._positive_time(effort_limit_nm, "simulation effort limit"))

        # A separate data buffer avoids changing the caller's kinematic state.
        initial_data = mujoco.MjData(robot.model)
        mujoco.mj_forward(robot.model, initial_data)
        mass = np.zeros((robot.model.nv, robot.model.nv))
        try:
            mujoco.mj_fullM(robot.model, initial_data, mass)
        except TypeError:  # MuJoCo < 3.10 has the older fullM signature.
            mujoco.mj_fullM(robot.model, mass, initial_data.qM)
        source_ids = np.array([
            mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
        ])
        if np.any(source_ids < 0):
            raise ValueError("Named URDF joints are absent from the MuJoCo model")
        source_dofs = robot.model.jnt_dofadr[source_ids]
        self.inertia_diagonal = np.diag(mass)[source_dofs].copy()
        if np.any(self.inertia_diagonal <= 0) or np.any(~np.isfinite(self.inertia_diagonal)):
            raise ValueError("The imported moving joints need positive finite inertia")
        self.velocity_gain = 2 * np.sqrt(self.POSITION_GAIN * self.inertia_diagonal)

        xml = ET.parse(robot.xml_path).getroot()
        if xml.find("actuator") is not None:
            raise ValueError("Expected the independent kinematic XML without actuators")
        option = xml.find("option")
        if option is None:
            option = ET.SubElement(xml, "option")
        option.attrib.update({
            "timestep": str(self.timestep), "integrator": "implicitfast",
            "gravity": "0 0 -9.81",
        })
        flag = option.find("flag")
        if flag is None:
            flag = ET.SubElement(option, "flag")
        flag.set("contact", "disable")
        actuators = ET.SubElement(xml, "actuator")
        self.actuator_names = tuple(f"servo_{name}" for name in self.joint_names)
        for index, name in enumerate(self.joint_names):
            joint = xml.find(f".//worldbody//joint[@name='{name}']")
            if joint is None:
                raise ValueError(f"Missing named joint in XML: {name}")
            joint.attrib.update({
                "damping": str(self.JOINT_DAMPING),
                "solreflimit": "0.004 1", "solimplimit": "0.99 0.999 0.001",
                "margin": "0.001",
            })
            ET.SubElement(actuators, "position", {
                "name": self.actuator_names[index], "joint": name, "gear": "1",
                "kp": str(self.POSITION_GAIN), "kv": str(self.velocity_gain[index]),
                "ctrllimited": "true", "forcelimited": "true",
                "ctrlrange": f"{self.lower[index]:.16g} {self.upper[index]:.16g}",
                "forcerange": f"{-self.effort_limit[index]:.16g} {self.effort_limit[index]:.16g}",
            })
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        joint_ids = np.array([
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
        ])
        self.q_indices = self.model.jnt_qposadr[joint_ids].copy()
        self.dof_indices = self.model.jnt_dofadr[joint_ids].copy()
        self.actuator_indices = np.array([
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.actuator_names
        ])
        if self.model.nu != 21 or np.any(self.actuator_indices < 0):
            raise ValueError("Expected exactly 21 named position actuators")
        if not np.array_equal(self.model.actuator_trnid[self.actuator_indices, 0], joint_ids):
            raise ValueError("Actuator-to-joint transmission mapping is inconsistent")
        self.reset(np.clip(np.zeros(21), self.lower, self.upper))

    @staticmethod
    def _positive_time(value, name):
        if isinstance(value, (bool, np.bool_)) or np.ndim(value) != 0:
            raise ValueError(f"{name} must be a finite positive scalar")
        try:
            result = float(value)
        except (ValueError, TypeError) as exc:
            raise ValueError(f"{name} must be a finite positive scalar") from exc
        if not np.isfinite(result) or result <= 0:
            raise ValueError(f"{name} must be a finite positive scalar")
        return result

    def _reference(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (21,) or np.any(~np.isfinite(q)):
            raise ValueError("Joint reference must be a vector of 21 finite radians")
        if np.any(q < self.lower - 1e-12) or np.any(q > self.upper + 1e-12):
            raise ValueError("Joint reference exceeds the URDF position limits")
        return np.clip(q, self.lower, self.upper)

    @property
    def measured_q(self):
        return self.data.qpos[self.q_indices].copy()

    def reset(self, q):
        """Initialize the simulation's position and zero velocity; clear stats."""
        q = self._reference(q)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.q_indices] = q
        self.data.ctrl[self.actuator_indices] = q
        mujoco.mj_forward(self.model, self.data)
        self._max_error = 0.0
        self._error_integral = 0.0
        self._elapsed = 0.0
        self._max_violation = 0.0
        self._max_velocity = 0.0
        self._max_force = 0.0
        self._saturated_joint_steps = 0
        self._physics_steps = 0
        self._finite = True
        self._warnings = 0

    def step(self, q_reference, dt):
        """Apply bounded position control and return simulated joint positions.

        No measured state assignment occurs here. A numerical instability or
        a MuJoCo warning raises FloatingPointError, because MuJoCo can reset
        unstable state internally and otherwise leave apparently finite data.
        Soft position-limit excursions are measured and reported, not hidden.
        """
        reference = self._reference(q_reference)
        dt = self._positive_time(dt, "dt")
        count = max(1, math.ceil(dt / self.timestep))
        substep = dt / count
        self.data.ctrl[self.actuator_indices] = reference
        self.model.opt.timestep = substep
        try:
            for _ in range(count):
                mujoco.mj_step(self.model, self.data)
                warnings = int(np.sum(self.data.warning.number))
                finite = all(np.all(np.isfinite(values)) for values in (
                    self.data.qpos, self.data.qvel, self.data.qacc, self.data.actuator_force,
                ))
                self._finite = self._finite and finite
                self._warnings = warnings
                if not finite or warnings:
                    raise FloatingPointError("MuJoCo numerical failure; inspect simulation.report")
                measured = self.data.qpos[self.q_indices]
                error = measured - reference
                force = np.abs(self.data.actuator_force[self.actuator_indices])
                self._max_error = max(self._max_error, float(np.max(np.abs(error))))
                self._error_integral += float(np.mean(error**2)) * substep
                self._elapsed += substep
                self._max_violation = max(self._max_violation, float(np.max(np.maximum(
                    np.maximum(self.lower - measured, measured - self.upper), 0,
                ))))
                self._max_velocity = max(self._max_velocity, float(np.max(np.abs(
                    self.data.qvel[self.dof_indices]
                ))))
                self._max_force = max(self._max_force, float(np.max(force)))
                self._saturated_joint_steps += int(np.count_nonzero(
                    force >= self.effort_limit * (1 - 1e-8)
                ))
                self._physics_steps += 1
            # Refresh site_xpos and other derived quantities for the viewer at
            # the measured end-of-step state; mj_forward does not integrate.
            mujoco.mj_forward(self.model, self.data)
        finally:
            self.model.opt.timestep = self.timestep
        return self.measured_q

    @property
    def report(self):
        return {
            "physics_steps": self._physics_steps,
            "simulated_time_s": self._elapsed,
            "timestep_s": self.timestep,
            "gravity_m_s2": self.model.opt.gravity.tolist(),
            "contacts_enabled": False,
            "finite": self._finite,
            "mujoco_warning_count": self._warnings,
            "max_tracking_error_rad": self._max_error,
            "tracking_rmse_rad": math.sqrt(self._error_integral / self._elapsed)
            if self._elapsed else 0.0,
            "max_measured_limit_violation_rad": self._max_violation,
            "soft_limit_tolerance_rad": self.SOFT_LIMIT_TOLERANCE,
            "measured_limit_violation": self._max_violation > self.SOFT_LIMIT_TOLERANCE,
            "max_measured_velocity_rad_s": self._max_velocity,
            "max_actuator_force_nm": self._max_force,
            "saturated_joint_step_fraction": self._saturated_joint_steps / (21 * self._physics_steps)
            if self._physics_steps else 0.0,
            "effort_limit_nm": self.effort_limit.tolist(),
            "urdf_effort_metadata_nm": self.urdf_effort_limit.tolist(),
            "effort_limit_source": "Explicit simulation setting; not inferred from URDF or hardware",
            "position_gain_nm_per_rad": self.POSITION_GAIN,
            "velocity_gain_nm_s_per_rad": self.velocity_gain.tolist(),
            "joint_damping_nm_s_per_rad": self.JOINT_DAMPING,
            "zero_pose_inertia_diagonal_kg_m2": self.inertia_diagonal.tolist(),
            "limits_note": "Simulation tuning only; 0.35 N m default is not verified hardware torque or mA.",
        }

    def close(self):
        """No owned files or external connections; provided for adapter symmetry."""

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
