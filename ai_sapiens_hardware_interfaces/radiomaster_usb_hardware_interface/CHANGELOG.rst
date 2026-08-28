^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package radiomaster_usb_hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-08-26)
------------------
* Added a ``ros2_control`` sensor interface that reads RadioMaster controllers
  directly through the Linux joystick API.
* Added configurable device, axis direction, RC channel defaults, and reconnect
  interval parameters.
* Added Linux joystick correction handling while preserving raw axis values for
  devices that report unusable correction data.
* Added safe HAT-compatible states while the controller is unavailable.
* Contributors: Eunsung Kim, Kiwoong Park
