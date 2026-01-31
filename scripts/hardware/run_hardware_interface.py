#!/usr/bin/env python3
"""
Hardware Interface Launcher - Standalone

Works around ROS2 entry point issues by running directly.
"""

import sys
import os

# Add src to path
workspace = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(workspace, "src"))

# Import and run
from autonomy.autonomy_core.autonomy_core.control.hardware_interface_node import main

if __name__ == "__main__":
    main()
