^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ai_sapiens_mujoco
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2026-09-01)
------------------
* None

0.2.0 (2026-08-26)
------------------
* Added the K1 MuJoCo simulation and viewer for sim2sim testing with
  ``ros2_control``.
* Added gantry controls for lowering, releasing, and restoring the robot to an
  upright hanging pose.
* Added viewer controls for geometry, collisions, centers of mass, contact
  forces, diagnostics, and mouse perturbations.
* Separated the MuJoCo ``ros2_control`` system interface into the
  ``mujoco_hardware_interface`` package.
* Integrated MuJoCo through the ROS 2 ``mujoco_vendor`` package.
* Contributors: Kiwoong Park
