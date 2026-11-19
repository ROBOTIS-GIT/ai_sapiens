# ROBOTIS AI Sapiens ROS 2 Packages

This repository contains the official ROS 2 packages for the ROBOTIS AI Sapiens K1 platform. These packages provide the robot description, bringup configuration, controller interfaces, and simulation-to-real tools used to control the robot and develop Physical AI applications.

For product information and documentation, visit:
  - [AI Sapiens](https://www.robotis.com/en/product/ecosystem-aisapiens.php)
  - [Documentation for AI Sapiens](https://docs.robotis.com/docs/systems/aisapiens/introduction)

To learn more about the Physical AI Tools, visit:
  - [Physical AI Tools](https://github.com/ROBOTIS-GIT/physical_ai_tools)

To explore open-source platforms in a simulation environment, visit:
  - [Simulation Models](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)

For usage instructions and demonstrations, check out:
  - [Tutorial Videos](https://www.youtube.com/@ROBOTISOpenSourceTeam)

To access datasets and pre-trained models for ROBOTIS open-source platforms, see:
  - [AI Models & Datasets](https://huggingface.co/ROBOTIS)

To use the Docker image for running ROS packages and Physical AI tools, visit:
  - [Docker Images](https://hub.docker.com/r/robotis/ros/tags)

## K1 model revisions

`k1_rev2` is the default model, with revised joint coordinates and link
inertias, colored DAE visuals, and matching MuJoCo OBJ assets. The rev2
meshes omit separate fasteners, internal actuator tubes, battery internals,
and the cap. Visual simplification preserves joint and inertial data.

```sh
ros2 launch ai_sapiens_description k1.launch.py
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py
```

Pass `model:=k1_rev1` to select the rev1 model.

See the [rev2 model notes](ai_sapiens_description/urdf/k1_rev2/README.md)
for source provenance, material support, linkage simplification and validation
limits. The revision does not add extra controlled joints or linkage constraints.
