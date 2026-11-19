^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ai_sapiens_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-11-19)
------------------
* Remove the rev2 cap visuals from both URDF DAE and MuJoCo OBJ assets;
  preserve the head visuals and inertial properties.

* Added a separate K1 rev2 description with the October 26 joint frames and
  link inertias, colored DAE visuals, and material-separated MuJoCo OBJ assets.
* Made rev2 the launch default. Select rev1 with ``model:=k1_rev1``.
* Refined the rev2 collision meshes.
* Preserved the 23-actuator training topology without linkage mimic joints
  or additional closed-loop constraints.
* Reduced rev2 visual triangles by 25.9% by omitting fasteners and internal
  actuator tubes, simplifying output flanges, and retaining only the battery
  exterior. Kept joint, inertial, actuator, sensor and collision data unchanged.

* Configured DYNAMIXEL homing offsets for the rev2 ankle-pitch actuators.
* Contributors: Woojin Wie, Wonho Yun

0.3.0 (2026-10-07)
------------------
* None

0.2.3 (2026-09-29)
------------------
* Removed the BMS remaining energy from the HAT sensor
* Contributors: Wonho Yun

0.2.2 (2026-09-14)
------------------
* Update K1 model with revised inertial properties and mesh files
* Contributors: Wonho Yun

0.2.1 (2026-09-01)
------------------
* None

0.2.0 (2026-08-26)
------------------
* Updated the K1 MJCF with per-actuator reflected armatures, IMU sensors, and
  MuJoCo Menagerie-compatible model and scene structure.
* Added a gantry scene and separate ``ros2_control`` descriptions for MuJoCo
  simulation and RadioMaster USB input.
* Added validation tests for the K1 MuJoCo scenes and ros2_control variants.
* Changed the IP address used for E2D2 UDP communication.
* Contributors: Kiwoong Park, Eunsung Kim

0.1.2 (2026-08-12)
------------------
* Added the current limit parameters to the DYNAMIXEL GPIOs
* Contributors: Wonho Yun

0.1.1 (2026-07-30)
------------------
* Updated the port name for the E2D2 UDP communication
* Contributors: Wonho Yun

0.1.0 (2026-07-24)
------------------
* Added the K1 rev1 ros2_control configuration: DYNAMIXEL limb buses over UDP, the HAT over TCP, and mock hardware support.
* Added the K1 MuJoCo model.
* Updated the K1 meshes, model parameters, and foot collision geometry.
* Contributors: Wonho Yun, Kiwoong Park, Woojin Wie

0.0.3 (2026-07-20)
------------------
* Updated the inertia values for the ankle
* Contributors: Wonho Yun

0.0.2 (2026-07-15)
------------------
* Separated the head from the torso and saved it as a separate mesh file
* Updated the inertia values for the ankle and torso
* Contributors: Wonho Yun

0.0.1 (2026-06-25)
------------------
* Initial release.
* Contributors: Woojin Wie
