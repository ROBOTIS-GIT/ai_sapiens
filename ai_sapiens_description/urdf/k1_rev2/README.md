# K1 rev2 description

This model uses the October 26, 2026 K1 coordinate and mass-property documents
and the corresponding STEP-derived assembly. `k1_rev2` is the launch default.
This package contains the derived simulation description and meshes.

The original STEP, PDFs, production Blender scene and reference photos remain
outside this repository.

## Select the model

```sh
ros2 launch ai_sapiens_description k1.launch.py model:=k1_rev2
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py model:=k1_rev2
```

The standalone description is `urdf/k1_rev2/k1.urdf`; the runtime wrapper is
`urdf/k1_rev2/k1.urdf.xacro`. MuJoCo uses `mujoco/k1_rev2/k1.xml`, `scene.xml`,
or `scene_gantry.xml`. It loads the OBJ assets rather than the DAE files.

## Frames and physics

- The 25 link names, 23 controlled joint names, axes, limits, effort limits,
  actuator classes and reflected motor armatures retain the rev1 convention.
- Origins follow `PR50_A02_K1_Coordinate_Relationships_20261026.pdf`, pages 2–3.
  Lower-body coordinates are unchanged from the official rev1 URDF. Arm origins
  include the revised offsets and the small left-arm numerical residuals.
- `ANKLE_PITCH_ROLL` is the shared ankle output frame. The pitch motor center
  185 mm above it is not substituted for the foot pitch joint.
- Link masses, centers of mass and COM inertia tensors use the matching named
  frames in `PR50_A02_K1_Mass_Properties_20261026.pdf`. No parallel-axis term is
  applied a second time to the COM tensor.
- The 25 link masses sum to **37.55070742 kg**. The PDF's whole assembly is
  **37.626662 kg**: a **75.95458 g** source discrepancy remains unallocated.
- URDF and MJCF share the collision meshes. Ankle and shoulder pitch links
  omit collision geometry. MuJoCo uses the convex hulls of mesh colliders
  for contact.
- Mechanical travel, hardware encoder zero, transmission losses and training
  performance have not been established by these CAD/file checks.

## Ankle linkage decision

The training description has **no additional mimic joints, passive joints or
loop constraints**. Its native MuJoCo model retains `nq=30`, `nv=29`, `nu=23`
(including the floating base), 26 bodies including world, and zero equality
constraints. The gantry variant retains its existing support weld.

The latest CAD has approximately 42 mm input/output crank radii and 185 mm
fixed/connecting lengths. Solving the ideal planar linkage at 1,801 angles per
side over ±45 degrees gave a maximum deviation from 1:1 of about 0.000165 degrees.
This supports a single effective ankle-pitch joint for kinematics. It does not
prove identical dynamics: rod/crank inertia, reflected rotor inertia, friction,
backlash and compliance need separate validation. The supplied link inertias are
treated as the CAD author's lumped-body approximation; part-level mass ownership
was not independently re-derived and no extra rod/crank mass is added.

URDF is a tree description. A linear mimic relation can approximate motion, but
does not portably represent a physical closed loop. A synthetic import test with
the installed MuJoCo 3.10.0 ignored the URDF mimic constraint. MJCF can explicitly
model loops, at the cost of additional state and constraint equations. Existing
ROS MuJoCo code maps configured joints by name; adding bodies does not inherently
break that mapping, but training environments and observations would still need
their own validation.

Crank and connecting-rod **visuals are fixed in their neutral shin-relative pose**
in this simplified model; they do not animate as a closed linkage when the ankle
moves. The foot joint remains articulated. Use the full Blender assembly for
mechanical linkage visualization. Do not treat these simplified visuals as
ground truth for camera-based training without a separate visual-motion solution.

References:

- [ROS joint and mimic semantics](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/joints_userdoc.html)
- [MuJoCo equality constraints](https://mujoco.readthedocs.io/en/stable/computation/index.html#equality)

## Visual assets

Per-link DAE meshes carry material groups for ROS viewers. MuJoCo uses the same
triangles and normals split into OBJ material groups. Units are meters. The
Blender object's actual material overrides are preserved, including black motor
housings, aluminum clamps/support covers, printed gray covers, black ankle
brackets and cyan chest insert. Shaders are approximations of the Blender PBR
materials. Logos are local geometric alpha cutouts, so no online texture assets
or external texture downloads are required.

The November 17 lightweight export contains **792,104 triangles** across 25 DAE
links and 110 OBJ material groups. This reduces triangles by **25.9%** from the
previous 1,069,465-triangle export. DAE and OBJ/MTL assets together occupy about
**89.7 MB**, down from 121.1 MB (decimal units).

- Remove 1,431 individually identified screw, bolt, nut and washer visuals.
  Structural standoffs, supports, locating pins and lifting eyes remain.
- Omit the cap, including its brim, seams, embroidery, strap and buckle; retain
  the original head geometry and inertial data.
- Replace all 23 motor output assemblies with simple black annular flanges.
  Preserve their measured outer diameters, axial seating, center bores and
  existing link ownership. Omit long internal hollow tubes and the embedded
  output-face fasteners. No additional joint or body is introduced.
- Replace the detailed battery assembly with its source exterior envelope,
  retaining the printed gray housing and exterior mounting/connector features.
  Cell and PCB internals from the detailed battery shell are omitted. The
  envelope has 6,000 triangles instead of the previous combined 79,182.
- Retain existing materials, logos, joint frames, mass, center of mass, inertia,
  actuator parameters, sensors and collision assets. Removed visual fasteners
  still contribute to the supplied aggregate link inertials; mass properties
  are not recomputed from the simplified meshes.

Unmodified objects reuse the previously reviewed geometry exactly. CAD seams
are welded before simplification, and weighted corner normals preserve flat
metal surfaces. The battery proxy has no boundary or nonmanifold edges; sampled
surface error against the exterior source is below 0.14 mm. Its intended omission
of internal components is not counted as a surface approximation error.
Positions remain on a 1 micrometer grid. These visual checks are not CAD
manufacturing-tolerance validation. Existing casing holes, recesses and embossed
surface details may remain even where separate fastener objects are removed.

The meshes retain some internal motor-housing features and torso/thigh
electronic component geometry; this export is not an exterior-only shell of
the complete robot. The battery assembly is the exterior-only exception.

These visuals are intended for simulation inspection, not the full-resolution
product-rendering scene. Detailed source CAD and the production Blender rig stay
outside this repository.

## Validation

The package registers `test/test_rev2_description.py` with ament/pytest. It checks
the retained joint contract, revised origins, asset paths, URDF/MJCF inertial
agreement, runtime Xacro variants and native MuJoCo loading/stepping. The MuJoCo
test is skipped if its Python package is unavailable. With Python, Xacro, NumPy
and MuJoCo installed, the same checks can be run from the repository root:

```sh
python3 -m unittest discover -s ai_sapiens_description/test -p test_rev2_description.py -v
```

Local macOS checks use a package resolver to expand Xacro without ROS. A full
ROS 2 build/launch, RViz rendering and trained-policy replay remain separate
checks; successful XML expansion and finite simulation steps do not establish
hardware or policy compatibility. Controller configuration is shared with rev1
because the joint names and command/state interfaces are unchanged; gains have
not been retuned for the new inertias.
