.. _quickstart:

==================
Quick Start Guide
==================

**Goal: Get the URC 2026 rover running in your browser in 10 minutes**

This guide gets you from zero to a working rover development environment as quickly as possible. We'll focus on getting things running - details come later.

.. image:: big_picture.png
   :alt: URC 2026 System Overview
   :align: center
   :width: 60%

Step 1: System Requirements (2 minutes)
=====================================

**Required:**
- Ubuntu 22.04 LTS (or equivalent)
- ROS2 Humble Hawksbill installed
- Python 3.10+ installed
- Git installed

**Check your environment:**
```bash
# Check ROS2
source /opt/ros/humble/setup.bash
ros2 --version  # Should show 2.x

# Check Python
python3 --version  # Should show 3.10 or higher

# Check Git
git --version
```

**Missing something?** See the detailed :doc:`getting_started` guide.

Step 2: Clone & Install Dependencies (3 minutes)
===============================================

**Clone the repository:**
```bash
git clone --recurse-submodules https://github.com/your-org/urc-machiato-2026.git
cd urc-machiato-2026
```

**Install Python dependencies:**
```bash
# Development installation (includes testing tools)
pip install -e ".[dev]"

# Or production installation
pip install -e .
```

**Quick verification:**
```bash
python3 -c "import rclpy; print('✅ ROS2 Python works!')"
```

Step 3: Launch Development Environment (3 minutes)
=================================================

**Start the dashboard with backend:**
```bash
./start.py dev dashboard
```

**Alternative options:**
```bash
# Frontend only
./start.py dev frontend

# Full autonomy system
./start.py prod autonomy

# Simulation environment
./start.py dev simulation
```

**Wait for startup, then open:**
http://localhost:5173

**You should see:**
- 🤖 Rover status dashboard
- 🎯 Mission control panel
- 📊 Real-time telemetry
- 💚 System health indicators

Step 4: Run Basic Tests (2 minutes)
===================================

**In another terminal, run the smart test runner:**
```bash
# Fast development tests
python scripts/run_tests.py dev

# Comprehensive quality checks
python scripts/run_tests.py pre-commit
```

**Monitor system health:**
```bash
# Check ROS2 topics
ros2 topic list | head -10

# Check system diagnostics
ros2 topic echo /diagnostics
```

Step 5: Make Your First Change (Optional - 5 minutes)
====================================================

**Edit a simple file:**
```bash
# Open a utility file
code src/core/utilities.py
```

**Make a small change:**
- Add a comment
- Modify a logging message
- Save the file

**Test your change:**
```bash
# Run relevant tests
python scripts/run_tests.py dev

# Check formatting
python -m ruff check src/
python -m ruff format src/
```

What Just Happened?
==================

You now have a complete rover development environment with:

- **🤖 ROS2 Backend**: Autonomy, navigation, and control systems
- **💻 Web Dashboard**: Real-time monitoring and mission control
- **🧪 Smart Test Suite**: Fast feedback with 85% coverage requirement
- **🔧 Development Tools**: Build scripts, linters, and utilities
- **🎮 Simulation**: Gazebo-based testing environment

Next Steps
==========

**Choose your path:**

🏃 **I want to explore more:**
   - Read the :doc:`big_picture` for system overview
   - Try simulation: ``./start.py dev simulation``
   - Browse the codebase: ``src/`` directory

👥 **I know my team role:**
   - **Network Team**: Communication bridges and resilience
   - **SLAM/NAV Team**: Navigation and localization
   - **ARM Team**: Manipulator and science systems
   - **Testing Team**: Quality assurance and validation

📚 **I want to learn the details:**
   - Development workflow: :doc:`development/workflow`
   - Testing strategy: :doc:`development/testing`
   - Architecture deep dive: :doc:`architecture/overview`
   - API documentation: :doc:`api/index`

Troubleshooting
===============

**Common Issues:**

**Dashboard won't load?**
```bash
# Check processes
ps aux | grep -E "(start.py|node|python)"

# Kill and restart
pkill -f "start.py"
./start.py dev dashboard
```

**Tests failing?**
```bash
# Run with verbose output
python scripts/run_tests.py dev --verbose

# Check specific test
python -m pytest tests/unit/test_specific_file.py -v
```

**ROS2 issues?**
```bash
# Check ROS2 installation
source /opt/ros/humble/setup.bash
ros2 doctor

# Check workspace setup
ls src/ | head -10
```

**Build issues?**
```bash
# Use unified build system
./scripts/build.sh dev

# Clean and rebuild
./scripts/build.sh clean
./scripts/build.sh dev
```

**Still stuck?**
- Read the detailed :doc:`getting_started` guide
- Check :doc:`development/troubleshooting`
- Ask in team development channels
- File an issue on GitHub

**Remember**: This is a complex autonomous robotics system. It's normal to have questions - ask early and often!

Development Quick Reference
========================

**Essential Commands:**
```bash
# Development workflow
./scripts/build.sh dev           # Build system
./scripts/run_tests.py dev      # Quick tests
./start.py dev dashboard        # Start environment

# Code quality
python -m ruff check src/     # Lint
python -m ruff format src/    # Format
python -m mypy src/           # Type check

# ROS2 operations
ros2 topic list               # See topics
ros2 node list                # See nodes
ros2 service list            # See services
```

**Key Files to Know:**
- `./AGENTS.md` - AI agent development guidelines
- `./scripts/build.sh` - Unified build system
- `./scripts/run_tests.py` - Smart test runner
- `./start.py` - Unified launcher
- `./pyproject.toml` - Project configuration