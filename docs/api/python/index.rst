==================
Python API Reference
==================

This section contains the API documentation for all Python modules in the URC 2026 autonomy system.

.. toctree::
   :maxdepth: 2
   :caption: Calibration

   calibration/camera
   calibration/hand_eye
   calibration/extrinsics

.. toctree::
   :maxdepth: 2
   :caption: Autonomy Subsystems

   autonomy/navigation
   autonomy/computer_vision
   autonomy/slam
   autonomy/state_management
   autonomy/autonomous_typing

.. toctree::
   :maxdepth: 2
   :caption: Utilities

   utils/rosbridge
   utils/calibration_tools
   utils/validation

Core Modules
============

.. toctree::
   :maxdepth: 2

   core

Class Inheritance Diagrams
=========================

State Management Classes
-------------------------

.. inheritance-diagram:: src.core.adaptive_state_machine.AdaptiveStateMachine
   :parts: 1

.. inheritance-diagram:: src.core.adaptive_state_machine.SystemState
   :parts: 1

# Note: Update these when corresponding modules are available
# Navigation Classes
# ------------------
# .. inheritance-diagram:: src.autonomy.navigation.path_planner.PathPlanner
#    :parts: 1
# Computer Vision Classes
# -----------------------
# .. inheritance-diagram:: src.autonomy.computer_vision.computer_vision_node.ComputerVisionNode
#    :parts: 1
# Calibration Classes
# -------------------
# .. inheritance-diagram:: src.autonomy.calibration.camera.calibrate_from_markers
#    :parts: 1

Utility Functions
=================

# Note: Update these when corresponding modules are available
# .. autofunction:: src.scripts.velocity_tools.calculate_velocity_metrics
# .. autofunction:: src.scripts.validate_infrastructure.check_system_health

Calibration Classes
===================

# Note: Update these when corresponding modules are available
# .. autoclass:: src.autonomy.calibration.camera.CameraCalibrator
#    :members:
#    :undoc-members:
#    :show-inheritance:
# .. autoclass:: src.autonomy.calibration.extrinsics.HandEyeCalibrator
#    :members:
#    :undoc-members:
#    :show-inheritance:

Exception Classes
=================

# Note: Update these when corresponding modules are available
# .. autoexception:: src.core.state_management.exceptions.StateTransitionError
# .. autoexception:: src.autonomy.calibration.camera.CalibrationError

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
