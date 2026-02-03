# Presentations Index

Welcome to the URC 2026 Presentations folder! These documents provide comprehensive overviews of the system architecture, codebase organization, and contribution opportunities.

## Presentation Materials

### 1. **Architecture Overview** (`01_architecture_overview.md`)
High-level system design and component relationships.

**Covers**:
- System architecture with ASCII diagrams
- Unified Infrastructure (configuration, bridges, monitoring)
- Autonomy Core (navigation, safety, control, perception)
- Mission execution and web dashboard
- Data flow and communication patterns
- Key technologies and versions

**Best for**: Understanding how everything fits together

---

### 2. **Codebase Organization** (`02_codebase_organization.md`)
Detailed directory structure and file organization.

**Covers**:
- Complete repository structure
- Key directories explained (src/infrastructure, autonomy_core, dashboard, missions, etc.)
- File navigation by task
- Import patterns and best practices
- Dependency flow
- Code navigation guide

**Best for**: Finding where things are and how they're organized

---

### 3. **How to Contribute** (`03_how_to_contribute.md`)
Step-by-step guide for making contributions.

**Covers**:
- Contribution workflow (8 steps)
- Creating feature branches
- Development and testing
- Quality checks (linting, formatting, testing)
- Committing and creating PRs
- Code review expectations
- Example: Adding a new feature (LiDAR support)
- Common contribution areas with skills needed

**Best for**: Getting started with your first contribution

---

### 4. **Priority Areas & Roadmap** (`04_priority_areas_roadmap.md`)
Development priorities and 3-month roadmap.

**Covers**:
- System overview with "hot spots" (critical, high, medium priority areas)
- Detailed priority areas:
  - 🔴 Critical: Safety, network resilience
  - 🟡 High: Navigation performance, sensor reliability, tests
  - 🟢 Medium: Perception, dashboard, monitoring, documentation
- 3-month development roadmap with weekly breakdown
- Estimated effort for different contribution types
- Success metrics
- How to pick your contribution area

**Best for**: Planning work and understanding system needs

---

### 5. **Interfaces & APIs Quick Reference** (`05_interfaces_apis_quick_ref.md`)
Code examples for all major APIs and common patterns.

**Covers**:
- Configuration API examples
- Navigation API usage
- Safety system API
- Control and sensor APIs
- Perception API (vision, SLAM)
- Bridge APIs (WebSocket, CAN)
- Monitoring API
- Mission API
- Common integration patterns
- Testing patterns
- Key configuration parameters
- Debug commands

**Best for**: Writing code and integrating components

---

## Quick Navigation

### I want to understand the system
Start here: **01_architecture_overview.md**

### I want to find specific code
Go to: **02_codebase_organization.md**

### I want to contribute
Follow: **03_how_to_contribute.md**

### I want to know what needs work
Check: **04_priority_areas_roadmap.md**

### I want to write code using the APIs
Reference: **05_interfaces_apis_quick_ref.md**

---

## Suggested Reading Order

For **new team members**:
1. 01_architecture_overview.md (30 min)
2. 02_codebase_organization.md (20 min)
3. 03_how_to_contribute.md (40 min)
4. 05_interfaces_apis_quick_ref.md (reference as needed)

For **existing members picking new tasks**:
1. 04_priority_areas_roadmap.md (20 min - find your area)
2. 03_how_to_contribute.md (refresh workflow)
3. 05_interfaces_apis_quick_ref.md (reference)

For **team leads**:
1. 01_architecture_overview.md
2. 04_priority_areas_roadmap.md (roadmap planning)
3. All presentations (for comprehensive overview)

---

## Key Diagrams & Visuals

### System Architecture
```
Dashboard (React) ↔ WebSocket Bridges ↔ ROS2 Autonomy Stack ↔ Hardware
```

### Core Subsystems
- Unified Infrastructure (Configuration, Bridges, Monitoring)
- Autonomy Core (Navigation, Safety, Control, Perception)
- Mission Execution
- Web Dashboard

### Priority Heat Map
🔴 **Critical**: Safety, Network Resilience
🟡 **High**: Navigation, Sensors, Testing
🟢 **Medium**: Perception, Dashboard, Monitoring

---

## Key Statistics

| Metric | Value |
|--------|-------|
| **Total Source Code** | ~15,000 lines |
| **Test Files** | 140+ |
| **Test Coverage** | 85% (target: 92%+) |
| **Documentation Files** | 50+ |
| **Core Modules** | 12 |
| **ROS2 Messages** | 20+ |
| **Configuration Options** | 50+ |

---

## Quick Links

### Documentation
- [Getting Started Guide](../getting_started.rst)
- [Architecture Deep Dive](../architecture/)
- [API Reference](../api/)
- [Development Workflow](../development/workflow.rst)

### Code
- [Infrastructure Code](../../src/infrastructure/)
- [Autonomy Core](../../src/autonomy/autonomy_core/)
- [Missions](../../missions/)
- [Tests](../../tests/)
- [Onboarding Examples](../onboarding/examples/)

### Project Metadata
- [AGENTS.md](../../AGENTS.md) - Development guidelines
- [README.md](../../README.md) - Project overview
- [DEPLOYMENT.md](../../DEPLOYMENT.md) - Production deployment
- [CONTRIBUTING.md](../../CONTRIBUTING.md) - Contribution process

---

## Key Takeaways

1. **System Design**: Unified infrastructure provides cross-cutting services; autonomy core handles robotics; missions orchestrate tasks.

2. **Code Organization**: Organized by functional domains (not layers); makes independent team work possible.

3. **Contribution Path**: Pick issue → branch → develop → test → review → merge.

4. **Priority Areas**: Focus on safety, resilience, performance; tests; then features.

5. **Getting Help**: Use these presentations first; then check docs/; then ask team members.

---

## Contributing Your Improvements

These presentations are living documents! If you find:
- Outdated information
- Confusing explanations
- Missing topics
- Better examples

**Please help improve them!**

Create an issue or PR with your suggestions. Every contribution improves the project for the next person.

---

## Questions?

- **Architecture questions** → Review 01_architecture_overview.md
- **Where is the code?** → Check 02_codebase_organization.md
- **How do I contribute?** → Follow 03_how_to_contribute.md
- **What should I work on?** → See 04_priority_areas_roadmap.md
- **How do I use the APIs?** → Reference 05_interfaces_apis_quick_ref.md
- **General questions** → Check AGENTS.md or ask in team chat

---

## Document Metadata

| Item | Value |
|------|-------|
| **Created** | 2026-01-30 |
| **Last Updated** | 2026-01-30 |
| **Version** | 1.0 |
| **Status** | Active |
| **Audience** | All contributors |

Good luck with the URC 2026 project!
