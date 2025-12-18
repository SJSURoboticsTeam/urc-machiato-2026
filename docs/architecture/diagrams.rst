========
Diagrams
========

This section contains comprehensive PlantUML diagrams documenting the URC 2026 Mars Rover Autonomy System architecture.

.. contents::
   :local:
   :depth: 2

Critical Priority Diagrams
==========================

These diagrams document the most critical aspects of system operation and safety.

Core System State Machine
-------------------------

The fundamental state machine governing rover operation with emergency handling.

.. uml:: diagrams/critical/01_core_system_state_machine.puml
   :caption: Core System State Machine
   :align: center

Mission State Machines
----------------------

Execution state machines for different mission types with failure handling.

.. uml:: diagrams/critical/02_mission_state_machines.puml
   :caption: Mission Execution State Machines
   :align: center

Safety System Architecture
--------------------------

Critical safety monitoring and emergency response systems.

.. uml:: diagrams/critical/03_safety_system_architecture.puml
   :caption: Safety System Architecture
   :align: center

High Priority Diagrams
======================

Core architectural components and communication patterns.

System Component Architecture
------------------------------

Overall system structure showing layered architecture and component relationships.

.. uml:: diagrams/high/04_system_component_architecture.puml
   :caption: System Component Architecture
   :align: center

ROS2 Communication Architecture
--------------------------------

Detailed ROS2 topic, service, and action interactions between system components.

.. uml:: diagrams/high/05_ros2_communication_architecture.puml
   :caption: ROS2 Communication Architecture
   :align: center

Mission Behavior Classes
-------------------------

Object-oriented design of mission behaviors and their relationships.

.. uml:: diagrams/high/06_mission_behavior_classes.puml
   :caption: Mission Behavior Class Hierarchy
   :align: center

Medium Priority Diagrams
========================

Detailed design and interface specifications.

ROS2 Interface Definitions
---------------------------

Complete message, service, and action definitions for ROS2 communication.

.. uml:: diagrams/medium/07_ros2_interface_definitions.puml
   :caption: ROS2 Interface Definitions
   :align: center

Frontend Component Architecture
--------------------------------

React-based dashboard architecture and component relationships.

.. uml:: diagrams/medium/08_frontend_component_architecture.puml
   :caption: Frontend Component Architecture
   :align: center

Hardware Interface Architecture
--------------------------------

Hardware abstraction layers and device driver architecture.

.. uml:: diagrams/medium/09_hardware_interface_architecture.puml
   :caption: Hardware Interface Architecture
   :align: center

Configuration Management
-------------------------

Configuration loading, validation, and hot-reload architecture.

.. uml:: diagrams/medium/10_configuration_management.puml
   :caption: Configuration Management Architecture
   :align: center

Low Priority Diagrams
=====================

Supporting systems and operational views.

Data Flow Architecture
-----------------------

End-to-end data flow from sensors through processing to actuation.

.. uml:: diagrams/low/11_data_flow_architecture.puml
   :caption: Data Flow Architecture
   :align: center

Diagram Maintenance
===================

Version Control
---------------

All diagrams are stored in the ``docs/architecture/diagrams/`` directory and are version controlled alongside the codebase.

Update Process
--------------

1. **Identify Changes**: When system architecture changes, identify affected diagrams
2. **Update Diagrams**: Modify corresponding PlantUML files
3. **Review**: Have architecture changes reviewed by relevant stakeholders
4. **Regenerate**: Build documentation to ensure diagrams render correctly
5. **Commit**: Include diagram changes in the same commit as code changes

Generation
----------

Diagrams are automatically generated during documentation build using Sphinx PlantUML extension.

.. code-block:: bash

   # Build documentation with diagrams
   cd docs
   make html

   # View generated diagrams
   open _build/html/architecture/diagrams.html

Quality Standards
-----------------

**Diagram Standards:**
- Use consistent colors and styling
- Include creation date and source references
- Document stakeholders for each diagram
- Keep diagrams focused and readable
- Use PlantUML best practices

**Review Criteria:**
- Accuracy: Diagrams match current implementation
- Completeness: All major components included
- Clarity: Easy to understand relationships
- Consistency: Follow established patterns
- Maintenance: Easy to update as code changes

Legend
======

Common Elements
---------------

.. uml::

   @startuml Legend
   title Diagram Legend

   package "Package/Container" as pkg #lightblue
   class "Class" as cls
   interface "Interface" as intf
   enum "Enumeration" as enm

   pkg --> cls : contains
   cls ..|> intf : implements
   cls --> enm : uses

   note right
     **Relationships:**
     --> Solid: Strong dependency
     ..> Dotted: Weak dependency
     <|-- Inheritance
     <|..> Realization
   end note
   @enduml

Color Coding
------------

- **🔵 Light Blue**: User Interface Layer
- **🟢 Light Green**: Autonomy/Core Systems
- **🟡 Yellow**: Processing/Perception
- **🟠 Orange**: Hardware Interfaces
- **🔴 Red**: Safety/Emergency Systems
- **🟣 Purple**: Services/Configuration
- **⚫ Gray**: External Systems
