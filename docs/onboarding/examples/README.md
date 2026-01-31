# Onboarding Code Examples

These examples are extracted from the former `examples/demos/` folder and are intended for use with the onboarding pillar documents.

| File | Purpose | Referenced in |
|------|---------|----------------|
| `component_registry.py` | Component registration and health checks | PILLAR_2_COGNITION.md, API docs |
| `state_machine.py` | Standalone ROS2 state machine for frontend testing | PILLAR_2_COGNITION.md |
| `safety_system.py` | Standalone ROS2 safety (e-stop, recovery, health) | PILLAR_1_PERCEPTION, PILLAR_3_MOTION_CONTROL |

Run from the project root so that `src` is on the path (e.g. `python -m docs.onboarding.examples.component_registry` or `PYTHONPATH=src python docs/onboarding/examples/state_machine.py`). The ROS2 examples require a sourced ROS2 environment.
