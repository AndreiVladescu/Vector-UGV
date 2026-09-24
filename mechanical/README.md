# Mechanical

The live design lives in Fusion 360. This folder holds exports taken at milestones:
- `cad/fusion/`: `.f3z` archives
- `cad/step/`: assembly and parts
- `cad/pcb/`: board outlines (DXF, Fusion to KiCad) and board STEPs (KiCad to Fusion)
- `stl/`: named `<part>_<material>_x<qty>.stl`
- `harness/`: cable drawings

Board outline loop: sketch the outline and holes on the body in Fusion, export DXF, import to KiCad Edge.Cuts. After layout, export STEP back and check fit through the full leg range.

The free Fusion licence allows 10 editable documents, so keep most parts as internal components of one assembly. Keep leg lengths as user parameters, since the same numbers go in the URDF.
