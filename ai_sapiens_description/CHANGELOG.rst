^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ai_sapiens_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-08-26)
------------------
* Updated the K1 MJCF with per-actuator reflected armatures, IMU sensors, and
  MuJoCo Menagerie-compatible model and scene structure.
* Added a gantry scene and separate ``ros2_control`` descriptions for MuJoCo
  simulation and RadioMaster USB input.
* Added validation tests for the K1 MuJoCo scenes and ros2_control variants.
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
