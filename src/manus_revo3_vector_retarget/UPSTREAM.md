# Source and workspace integration

Imported from `BrainCoTech/revoarm_teleoperation` commit
`f0e23030f907b7a66b167d25c146d7f32ab9e070`, directory
`revoarm_hardware/Revoarm_ws/src/brainco_capabilities/manus_revo3_vector_retarget`.
That package originally came from `1Jimmy-Chen/manus-revo3-vector-retargeting`,
branch `codex/official-revo3-hardware`, commit
`381e9687b8f9cfe2e5e18691d1467d6dc131529c`.

This is a regular source package, not a Git submodule. Upstream declares
Proprietary; the import does not relicense it under the workspace license.

Local integration:

- Keep the upstream vector objective, calibration, input validation, scheduling,
  MIT transport, pause/recalibrate services, and source-provenance checks.
- Use the existing `brainco_revo3_ros2@34528e75` May3 URDFs and meshes. Identify
  this model as `may3`; bind each profile to its actual side's URDF hash.
- Preserve named ROS joint coordinates, limits, and existing per-joint MIT gains.
  Do not import the newer-model MPR-axis migration or driver/SDK changes.
- Resolve source resources through this workspace's driver submodule layout;
  resolve installed resources through ament.
- Use the colcon-installed console entry point so runtime retains the workspace
  `revo_teleop` Python interpreter. Support the existing MuJoCo 3.3.7 environment.
- Add `retarget_backend:=vector` to the existing managed teleop script. MANUS
  raw skeleton input is required; legacy/DV1/other input paths remain available.
- Adapt geometry tests to the May3 assets, retaining independent XML FK,
  analytic Jacobian, hardware-boundary, input, solver, and ROS lifecycle tests.
  Replace source-repository robot bringup tests with this workspace's script tests.

The driver submodule and its existing local CMake modification are not part of
this import. Software tests do not establish physical calibration or hand performance.
