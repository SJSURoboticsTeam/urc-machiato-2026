==========
Quick Start
==========

**Goal: Get the rover running in your browser in 10 minutes**

If you're new to the project, this guide will get you from zero to "I can see the rover working" as quickly as possible. Skip the details - we'll get to those later.

.. image:: big_picture.png
   :alt: URC 2026 System Overview
   :align: center
   :width: 60%

Step 1: System Requirements Check (2 minutes)
==============================================

**Ubuntu 22.04 with ROS2 Humble?** If not, see :doc:`installation`.

**Python 3.10+ installed?**
```bash
python3 --version  # Should show 3.10 or higher
```

**Git installed?**
```bash
git --version
```

Step 2: Clone & Setup (3 minutes)
=================================

**Clone the repository:**
```bash
git clone https://github.com/your-org/urc-machiato-2026.git
cd urc-machiato-2026
```

**Install Python dependencies:**
```bash
pip install -e .
```

**Quick test that everything works:**
```bash
python -c "import rclpy; print('ROS2 Python works!')"
```

Step 3: Launch Development Environment (2 minutes)
===================================================

**Start the dashboard (includes backend):**
```bash
./start.py dev dashboard
```

**Wait for startup messages, then open:**
http://localhost:3000

**You should see:**
- Rover status dashboard
- Mission control panel
- Real-time telemetry
- System health indicators

Step 4: Try Basic Operations (3 minutes)
=========================================

**Run a quick test:**
```bash
# In another terminal, run basic tests
python -m pytest tests/unit/test_utilities.py -v
```

**Monitor system health:**
```bash
# Check ROS2 topics (in another terminal)
ros2 topic list | head -10
```

**Try the web interface:**
- Click around the dashboard
- Check system status
- View available missions

Step 5: Your First Code Change (Optional - 5 minutes)
=====================================================

**Find a simple file to edit:**
```bash
# Look at a basic utility
code src/core/json_processor.py
```

**Make a small change:**
- Add a comment
- Save the file
- Check that tests still pass: ``python -m pytest tests/unit/ -k json``

What Just Happened?
===================

You now have a working rover development environment! The system includes:

- **ROS2 backend** running autonomy software
- **Web dashboard** for monitoring and control
- **Test suite** to verify everything works
- **Development tools** for making changes

Next Steps
==========

**Choose your path:**

🏃 **I want to explore more:**
   - Read the :doc:`big_picture` guide
   - Try running in simulation: ``./start.py dev simulation``

👥 **I know my role:**
   - Network team → :doc:`network_guide`
   - SLAM/NAV team → :doc:`slam_nav_guide`
   - ARM team → :doc:`arm_guide`
   - Testing team → :doc:`testing_guide`

📚 **I want to learn the details:**
   - Deep dive: :doc:`getting_started`
   - Architecture: :doc:`architecture/overview`
   - API docs: :doc:`api/python/index`

Troubleshooting
===============

**Dashboard won't load?**
```bash
# Check if processes are running
ps aux | grep python
# Kill and restart
pkill -f "start.py"
./start.py dev dashboard
```

**Tests failing?**
```bash
# Run with more verbose output
python -m pytest tests/unit/ -v -s
```

**ROS2 issues?**
```bash
# Check ROS2 installation
source /opt/ros/humble/setup.bash
ros2 doctor
```

**Still stuck?**
- Check the detailed :doc:`getting_started` guide
- Ask in team chat
- File an issue on GitHub

Remember: This is a complex robotics system. It's normal to have questions - ask early and often!
