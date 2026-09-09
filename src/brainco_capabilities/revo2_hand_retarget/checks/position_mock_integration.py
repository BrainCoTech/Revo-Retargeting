"""Exercise the position publisher and real ROS forward controller with GenericSystem.

Run manually after sourcing install/position_test/local_setup.bash. Uses only
mock hardware and a dedicated localhost ROS domain; never opens a robot port.
"""
import os
import signal
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "87"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["ROS_LOG_DIR"] = tempfile.mkdtemp(prefix="revo2_position_mock.")

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from revo2_hand_retarget.revo2_joints import joint_names_for_side
from revo2_hand_retarget.teleop_controller_node import Revo2TeleopController


def main():
    log_path = os.path.join(os.environ["ROS_LOG_DIR"], "mock_driver.log")
    with open(log_path, "w") as log:
        driver = subprocess.Popen([
            "ros2", "launch", "revo2_driver", "revo2_system.launch.py",
            "hand_side:=left", "if_sim:=true", "update_rate:=100",
            "read_touch_status:=false", "launch_rsp:=false",
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            rclpy.init(args=["--ros-args", "-p", "hand_mode:=left", "-p", "output_mode:=position",
                            "-p", "left_target_joint_state_topic:=/revo2_left/retarget/target_joint_states"])
            output = Revo2TeleopController()
            probe = Node("position_mock_probe")
            executor = SingleThreadedExecutor()
            executor.add_node(output)
            executor.add_node(probe)
            commands, actual = [], []
            probe.create_subscription(Float64MultiArray,
                                      "/revo2_left/joint_forward_pos_controller/commands",
                                      lambda msg: commands.append(list(msg.data)), 10)
            probe.create_subscription(JointState, "/revo2_left/revo2_joint_state/joint_states",
                                      lambda msg: actual.append(msg), 10)
            pub = probe.create_publisher(JointState, "/revo2_left/retarget/target_joint_states", 10)
            client = probe.create_client(ListControllers, "/revo2_left/controller_manager/list_controllers")

            def spin_until(condition, timeout):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if driver.poll() is not None:
                        raise AssertionError("Mock driver exited")
                    executor.spin_once(timeout_sec=0.02)
                    if condition():
                        return
                raise AssertionError("Mock integration timed out")

            spin_until(lambda: actual and commands and client.service_is_ready(), 35)
            states = {}
            controller_deadline = time.monotonic() + 15
            while time.monotonic() < controller_deadline:
                request = client.call_async(ListControllers.Request())
                spin_until(request.done, 5)
                states = {c.name: c.state for c in request.result().controller}
                if (states.get("joint_forward_pos_controller") == "active" and
                        states.get("revo2_pid_controller") == "inactive" and
                        states.get("joint_forward_vel_controller") == "inactive"):
                    break
                executor.spin_once(timeout_sec=0.1)
            assert states["joint_forward_pos_controller"] == "active", states
            assert states.get("revo2_pid_controller") != "active", states
            assert states.get("joint_forward_vel_controller") != "active", states

            target = np.array([0.30, 0.40, 0.25, 0.35, 0.20, 0.30])
            names = joint_names_for_side("left")
            # Deliberately reverse message order to exercise named conversion.
            msg = JointState(name=list(names[::-1]), position=target[::-1].tolist())
            timer = probe.create_timer(0.02, lambda: pub.publish(msg))

            def reached():
                positions = dict(zip(actual[-1].name, actual[-1].position))
                return np.max(np.abs(np.array([positions[n] for n in names]) - target)) < 0.015

            spin_until(reached, 8)
            timer.cancel()
            hold_start = time.monotonic()
            spin_until(lambda: time.monotonic() - hold_start > 0.8, 2)
            assert len(commands) > 30
            assert np.all(np.isfinite(np.asarray(commands)))
            assert np.min(np.asarray(commands[-10:])) > 0.1  # No zero-on-timeout command.
            np.testing.assert_allclose(commands[-1], commands[-5], atol=1e-9)
            print(f"PASS: named targets -> position commands -> forward controller -> mock feedback; "
                  f"timeout holds. {len(commands)} command frames. Log: {log_path}")
            executor.shutdown()
            output.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()
        finally:
            os.killpg(driver.pid, signal.SIGINT) if driver.poll() is None else None
            try:
                driver.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(driver.pid, signal.SIGKILL)
                driver.wait()
            if rclpy.ok():
                rclpy.shutdown()
            print(f"Mock driver log: {log_path}")


if __name__ == "__main__":
    main()
