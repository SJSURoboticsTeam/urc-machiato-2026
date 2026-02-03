# How to Contribute to URC 2026

## Contributing Guidelines

Thank you for wanting to contribute to the URC 2026 robotics platform! This guide will help you make meaningful contributions.

## Before You Start

### Prerequisites
- Familiarity with Python 3.10+, ROS2 Humble, and git
- Development environment set up: `pip install -e .`
- Access to this repository with write permissions
- ~2-3 hours for first contribution

### Getting Familiar
1. Read **AGENTS.md** for development guidelines and command reference
2. Review **docs/getting_started.rst** for project orientation
3. Check relevant **docs/onboarding/PILLAR_*.md** for your specialization
4. Review **CONTRIBUTING.md** for detailed processes

## The Contribution Workflow

```
1. Pick an Issue or Feature
        ↓
2. Create Feature Branch
        ↓
3. Develop & Test Locally
        ↓
4. Run Quality Checks
        ↓
5. Create Pull Request
        ↓
6. Code Review & Iteration
        ↓
7. Merge to Main
```

### Step 1: Pick Work

**Find issues:**
- GitHub Issues labeled: `good-first-issue`, `help-wanted`
- Project board: Assigned features and bugs
- AGENTS.md: "Common Tasks" section for inspiration

**Claim the work:**
- Comment on the issue to claim it
- Or create a new issue describing your feature

### Step 2: Create Feature Branch

```bash
# Pull latest changes
git pull origin main --recurse-submodules

# Create feature branch (use descriptive name)
git checkout -b feature/add-vision-detection
# or
git checkout -b fix/navigation-path-planning
# or
git checkout -b docs/update-camera-calibration
```

**Naming convention:**
- `feature/` - New features
- `fix/` - Bug fixes
- `docs/` - Documentation updates
- `refactor/` - Code refactoring
- `test/` - Test additions

### Step 3: Develop & Test

```bash
# Build the system
./scripts/build.sh dev

# If you modified ROS2 packages:
colcon build --symlink-install

# Test your changes
python -m pytest tests/unit/ -v

# Test specific test file
python -m pytest tests/unit/test_navigation.py -v

# Test with coverage
python -m pytest tests/ --cov=src --cov-report=html
```

**Code standards:**
```python
# ✓ Good: Type hints, docstrings, error handling
def calculate_trajectory(
    start: Point,
    end: Point,
    max_velocity: float = 1.0
) -> List[Point]:
    """Calculate optimal trajectory between two points.
    
    Args:
        start: Starting position
        end: Target position
        max_velocity: Maximum velocity constraint (m/s)
    
    Returns:
        List of waypoints forming the trajectory
        
    Raises:
        ValueError: If start or end position is invalid
    """
    if not isinstance(start, Point):
        raise ValueError("start must be a Point instance")
    
    # Implementation
    return trajectory

# ✗ Bad: No types, no docstring, bare exception
def calc_trajectory(start, end, vel=1):
    try:
        # some code
        return result
    except:
        pass
```

**Formatting:**
```bash
# Auto-format code
python -m ruff format src/

# Sort imports
python -m isort --profile=black src/

# Check types
python -m mypy src/ --ignore-missing-imports
```

### Step 4: Quality Checks

```bash
# Run comprehensive quality check
./scripts/check_quality.sh

# Or individual checks
pre-commit run --all-files              # All pre-commit hooks
python -m ruff check src/               # Linting
python -m ruff format src/              # Formatting
python -m mypy src/ --ignore-missing-imports  # Type checking
python -m pytest tests/ --cov=src       # Tests with coverage

# Run smart test runner
python scripts/run_tests.py pre-commit  # Pre-commit validation
```

### Step 5: Commit & Push

```bash
# Add changes
git add src/my_module.py tests/test_my_module.py

# Commit (concise message describing what and why)
git commit -m "Add vision-based object detection

- Implement YOLOv8 detection pipeline
- Add 92% detection accuracy for ArUco markers
- Include comprehensive unit tests
- Update documentation with usage examples

Fixes #123"

# Push to remote
git push origin feature/add-vision-detection
```

**Commit message guidelines:**
- First line: 50 chars max, imperative mood ("Add feature" not "Added feature")
- Blank line
- Body: Explain what and why (wrap at 72 chars)
- Reference issues: "Fixes #123", "Related to #456"

### Step 6: Create Pull Request

```bash
# After push, GitHub will show a "Compare & pull request" button

# Or create via CLI:
gh pr create --title "Add vision-based object detection" \
  --body "Implements YOLOv8 detection pipeline for ArUco marker identification"
```

**PR description template:**
```markdown
## Summary
Brief description of changes

## Changes Made
- Change 1
- Change 2
- Change 3

## Testing
- [ ] Added unit tests
- [ ] Added integration tests
- [ ] Tested in simulation
- [ ] Tested on hardware (if applicable)

## Checklist
- [ ] Code follows project style guidelines
- [ ] Documentation updated
- [ ] Tests pass locally (`./scripts/check_quality.sh`)
- [ ] No breaking changes introduced

## Screenshots (if UI changes)
...
```

### Step 7: Review & Merge

**During review:**
- Respond to reviewer comments
- Make requested changes
- Push updates (they appear automatically in PR)
- Request re-review when ready

**Example conversation:**
```
Reviewer: "This function is too complex. Consider breaking it up."
You: "Good point. I've refactored into three smaller functions with better names."

[You push updates]

Reviewer: "Looks good! Approving."
```

## Common Contribution Areas

### 1. Navigation & Planning
**Where**: `src/autonomy/autonomy_core/navigation/`

**Examples**:
- Add new path planning algorithm (A*, Dijkstra, RRT*)
- Improve SLAM integration
- Add obstacle avoidance
- Optimize motion control

**Skills**: Path planning, math, robotics

### 2. Vision & Perception
**Where**: `src/autonomy/autonomy_core/perception/`

**Examples**:
- Improve object detection accuracy
- Add new sensor support
- Enhance SLAM robustness
- Implement semantic segmentation

**Skills**: Computer vision, deep learning, OpenCV

### 3. Safety & Reliability
**Where**: `src/autonomy/autonomy_core/safety/`

**Examples**:
- Enhance emergency stop logic
- Improve watchdog monitoring
- Add sensor validation
- Implement auto-recovery

**Skills**: Safety systems, testing, error handling

### 4. Configuration & Infrastructure
**Where**: `src/infrastructure/`

**Examples**:
- Expand configuration options
- Improve bridge resilience
- Enhance monitoring capabilities
- Add diagnostic tools

**Skills**: Systems design, Python, configuration management

### 5. Testing & Quality
**Where**: `tests/`

**Examples**:
- Add performance tests
- Create hardware-in-the-loop tests
- Improve test coverage
- Add chaos engineering tests

**Skills**: Testing, pytest, performance analysis

### 6. Web Dashboard
**Where**: `src/dashboard/`

**Examples**:
- Add new monitoring views
- Improve real-time updates
- Enhance visualization
- Add telemetry analytics

**Skills**: React, TypeScript, WebSocket

### 7. Documentation
**Where**: `docs/`

**Examples**:
- Update API documentation
- Create tutorials
- Add architecture diagrams
- Write deployment guides

**Skills**: Technical writing, documentation tools

### 8. Missions & Tasks
**Where**: `missions/`

**Examples**:
- Implement new URC challenge mission
- Optimize mission execution
- Add mission recovery logic
- Improve task sequencing

**Skills**: Robotics, task planning, ROS2

## Priority Areas & Pain Points

### HIGH PRIORITY (Needed Now)

1. **Performance Optimization**
   - Navigation: Reduce latency in path planning
   - Perception: Speed up object detection
   - Dashboard: Improve WebSocket update frequency
   - *Impact*: Faster mission execution, better real-time control

2. **Sensor Reliability**
   - Add timeout handling for sensor failures
   - Implement sensor fusion fallbacks
   - Add quality metrics for sensor data
   - *Impact*: System resilience in field conditions

3. **Hardware Abstraction**
   - Better motor control patterns
   - Sensor reading standardization
   - Device driver interface
   - *Impact*: Easier hardware integration

4. **Test Coverage Improvement**
   - Increase from current 85% to 90%+
   - Add chaos engineering tests
   - Improve performance benchmarks
   - *Impact*: Fewer bugs in production

### MEDIUM PRIORITY (Useful)

5. **Dashboard Enhancements**
   - Add 3D visualization
   - Real-time telemetry charts
   - Mission replay capability
   - *Impact*: Better operator situational awareness

6. **Configuration Management**
   - Environment-specific profiles
   - Parameter tuning interface
   - Configuration validation
   - *Impact*: Easier deployment to different environments

7. **Monitoring & Diagnostics**
   - Enhanced system health dashboard
   - Diagnostic data collection
   - Performance profiling
   - *Impact*: Faster debugging and optimization

8. **Documentation Quality**
   - API reference completion
   - Architecture diagram updates
   - Troubleshooting guides
   - *Impact*: Better onboarding for new team members

### NICE TO HAVE (If Time)

9. **Simulation Enhancements**
   - More realistic sensor simulation
   - Terrain variations
   - Weather effects
   - *Impact*: Better validation before hardware

10. **Developer Tools**
    - Enhanced debugging utilities
    - Log analysis tools
    - Performance profiling tools
    - *Impact*: Faster development cycle

## Getting Help

### Resources
- **Codebase questions**: Check AGENTS.md and docs/getting_started.rst
- **Architecture questions**: See docs/architecture/
- **API questions**: Review docs/api/
- **Onboarding**: Check docs/onboarding/PILLAR_*.md for your specialization

### Communication
- **GitHub Issues**: Ask questions in issue threads
- **Team Chat**: Use development channels
- **Code Review**: Ask reviewers for guidance
- **Pair Programming**: Reach out to team members

## Code Review Expectations

### What Reviewers Look For
- **Correctness**: Does it work as intended?
- **Quality**: Is it maintainable and well-tested?
- **Style**: Does it follow project conventions?
- **Documentation**: Are changes documented?
- **Performance**: Does it have acceptable performance?
- **Safety**: Are edge cases handled?

### Typical Review Comments
- "This function is doing too much. Consider splitting it"
- "Missing docstring for public function"
- "Add error handling for this case"
- "Test coverage is only 60%, need more tests"
- "This could be more efficient using..."

### Responding to Reviews
1. **Don't take it personally** - Reviews are about the code
2. **Ask clarifying questions** - "Can you give an example?"
3. **Suggest alternatives** - "What if we did this instead?"
4. **Make requested changes** - And explain your approach
5. **Thank reviewers** - They're helping you improve

## Example: Adding a New Feature

**Scenario**: Add support for LiDAR sensor

### Step 1: Create Issue
```
Title: Add LiDAR sensor support
Description:
- Add LiDAR data reader in perception module
- Implement point cloud processing
- Integrate with navigation system
- Add unit and integration tests
```

### Step 2: Feature Branch
```bash
git checkout -b feature/add-lidar-support
```

### Step 3: Implement (iterative)
```bash
# File: src/autonomy/autonomy_core/perception/lidar_reader.py
# File: src/autonomy/autonomy_core/perception/point_cloud_processor.py
# File: src/autonomy/interfaces/msg/PointCloud.msg

./scripts/build.sh dev
python -m pytest tests/unit/test_lidar_reader.py -v
```

### Step 4: Quality
```bash
./scripts/check_quality.sh
```

### Step 5: Commit & Push
```bash
git add src/autonomy/autonomy_core/perception/
git commit -m "Add LiDAR sensor support

- Implement LiDAR data reader with 30Hz update rate
- Add point cloud processing pipeline
- Integrate with navigation for obstacle detection
- Achieve 95% obstacle detection accuracy

Fixes #456"

git push origin feature/add-lidar-support
```

### Step 6: Create PR
```
Title: Add LiDAR sensor support
Body:
## Summary
Adds comprehensive LiDAR support for obstacle detection and SLAM

## Changes
- New LiDAR reader with multi-beam support
- Point cloud filtering and downsampling
- Integration with navigation obstacle map
- Unit tests (95% coverage) and integration tests

## Testing
- [x] Unit tests pass (25 new tests)
- [x] Integration tests pass
- [x] Tested in simulation
- [x] Tested on hardware with actual LiDAR

## Performance
- Update rate: 30 Hz
- Processing latency: <50ms
- CPU usage: <15%
```

### Step 7: Review Cycle
```
Day 1: Reviewer requests changes
  - "Add error handling for sensor timeouts"
  - "Document the coordinate transformation"

Day 2: You make changes and push
  - Add timeout logic
  - Add detailed docstrings

Day 3: Reviewer approves
  - "Looks great! Merging now"
```

### Step 8: Merged!
Your feature is now part of the main codebase.

## Recognition

Contributors are recognized via:
- GitHub contributor graph (automatic)
- Project documentation acknowledgments
- Team announcements for major contributions
- Opportunities for additional team leadership roles

## Summary

**The key to successful contributions:**

1. **Start small** - Pick manageable issues first
2. **Communicate early** - Ask questions, discuss approaches
3. **Follow processes** - Build→test→review ensures quality
4. **Be thorough** - Good tests and documentation matter
5. **Be professional** - Respectful communication in all interactions
6. **Have fun** - This is an amazing project to work on!

Thank you for contributing to URC 2026!
